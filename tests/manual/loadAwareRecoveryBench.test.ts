import { mkdir, readFile, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { editedForceAtTime, type EditedForce } from "../../src/physics/forceSequence";
import { capturePointXY, insideConvexHullXY } from "../../src/control/balanceSensors";
import { rootTiltDegrees } from "../../src/physics/standingPush";
import type { SupportTransferReadiness } from "../../src/control/supportTransferDiagnostics";

type TraceSample = {
  timeS:number;phase:string;coordinator:string;com:readonly number[];
  velocity:readonly number[];capture:readonly number[];leftN:number;rightN:number;
  tiltDeg:number;pelvisM:number;swingSole:readonly number[]|null;supportSole:readonly number[]|null;
  comInsideSelectedSupport:boolean;readiness:SupportTransferReadiness|null;
  fsmReadyDurationS:number;strictReadyDurationS:number;knees:number[];
  control:ReturnType<MujocoSimulation["experimentalControlObservation"]>;qpos:number[];
};

const enabled = process.env.HPL_RUN_LOAD_AWARE === "1";
const options = {
  planningDelayS:process.env.HPL_PLANNING_DELAY ? Number(process.env.HPL_PLANNING_DELAY) : undefined,
  captureLookaheadS:process.env.HPL_CAPTURE_LOOKAHEAD ? Number(process.env.HPL_CAPTURE_LOOKAHEAD) : undefined,
  emergencyLateralShiftM:process.env.HPL_SHIFT_M ? Number(process.env.HPL_SHIFT_M) : undefined,
  emergencyShiftRampS:process.env.HPL_SHIFT_RAMP ? Number(process.env.HPL_SHIFT_RAMP) : undefined,
  transferHipRollRad:process.env.HPL_TRANSFER_HIP ? Number(process.env.HPL_TRANSFER_HIP) : undefined,
  transferAnkleDampingScale:process.env.HPL_ANKLE_DAMPING ? Number(process.env.HPL_ANKLE_DAMPING) : undefined,
};
const matrix = [
  ...[120,160,200].map(n => ({ label:`forward-${n}`, force:[n,0,0] as const })),
  ...[100,130,160].map(n => ({ label:`backward-${n}`, force:[-n,0,0] as const })),
  ...[160,220,300].map(n => ({ label:`left-${n}`, force:[0,n,0] as const })),
  ...[160,220,300].map(n => ({ label:`right-${n}`, force:[0,-n,0] as const })),
];

describe.skipIf(!enabled)("Issue 70: fixed 12-case physical campaign", () => {
  it("saves every trial including no-step and failed cases without changing gates", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const out = resolve(process.env.HPL_RESULTS_DIR ?? "work/load-aware");
    await mkdir(out,{recursive:true});
    const rows = [];
    for (const c of matrix.filter(c => !process.env.HPL_CASE || c.label.includes(process.env.HPL_CASE))) {
      const sim = await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalReactiveRecovery(options);
        const pulse:EditedForce = {id:c.label,site:"sternum_impact",forceN:c.force,
          startS:1,durationS:.16,waveform:"half-sine"};
        let peakN=0,impulseNs=0;
        const trace:TraceSample[] = [];
        const transitions:Array<{timeS:number;phase:string}> = [];
        let lastPhase="",firstWarningS:number|null=null,firstFallS:number|null=null;
        let firstActualCaptureExitS:number|null=null;
        let intentLoads:unknown=null;
        let strictReadySince:number|null=null,strictReadyDuration=0,maxStrictReadyDuration=0;
        let readySamples=0,maxReadySamples=0,flightSince:number|null=null,maxFlightS=0;
        let minPelvisM=Infinity,maxTiltDeg=0;
        let initialSwing:readonly number[]|null=null,initialSupport:readonly number[]|null=null;
        let supportDriftM=0,landingDriftM=0,firstTouch:readonly number[]|null=null;
        let postStabilitySince:number|null=null,maxPostStabilityS=0;
        for(let frame=0;frame<300;frame++) {
          sim.step(10,t => {
            const f=editedForceAtTime(t,pulse),n=Math.hypot(...f);
            if(n>0)sim.applyWorldForceAtSite(pulse.site,f);
            peakN=Math.max(peakN,n);impulseNs+=n*.002;
          });
          const state=sim.state(),b=sim.humanoidBalanceObservation();
          const control=sim.experimentalControlObservation();
          expect(control.freeRootApplied.every(n=>n===0)).toBe(true);
          expect(control.pelvisAppliedWrench.every(n=>n===0)).toBe(true);
          const g=sim.humanoidGroundSupportObservation();
          const r=sim.experimentalReactiveRecoveryStatus()!;
          const s=sim.experimentalStepStatus();
          const intent=r.lastIntent;
          const phase=s?.phase??r.phase;
          const tilt=rootTiltDegrees(state.qpos),pelvis=state.qpos[2]!;
          const speed=Math.hypot(b.comVelocityWorldMps[0],b.comVelocityWorldMps[1]);
          const comXY=[b.comWorldM[0],b.comWorldM[1]] as const;
          const cp=capturePointXY(comXY,[b.comVelocityWorldMps[0],b.comVelocityWorldMps[1]],b.comWorldM[2]);
          if(firstActualCaptureExitS===null && speed>=.12 &&
            !insideConvexHullXY(cp,g.loadedSupportPolygonXY,.004))firstActualCaptureExitS=state.time;
          minPelvisM=Math.min(minPelvisM,pelvis);maxTiltDeg=Math.max(maxTiltDeg,tilt);
          if(phase!==lastPhase){transitions.push({timeS:state.time,phase});lastPhase=phase;}
          if(r.phase==="WARNING" && firstWarningS===null)firstWarningS=state.time;
          if((r.phase==="FALL"||phase==="FALL") && firstFallS===null)firstFallS=state.time;
          const swing=intent ? sim.siteWorldPosition(intent.swingSide+"_sole") : null;
          const support=intent ? sim.siteWorldPosition(intent.supportSide+"_sole") : null;
          if(intent && !intentLoads){
            intentLoads={timeS:state.time,support:intent.supportSide,
              supportN:g[intent.supportSide].verticalLoadN,swingN:g[intent.swingSide].verticalLoadN};
            initialSwing=swing;initialSupport=support;
          }
          const swingN=intent ? g[intent.swingSide].verticalLoadN : null;
          // Report the stricter user criterion independently: COM over the
          // selected loaded sole, not merely anywhere in the bilateral hull.
          const comInsideSelectedSupport=!!intent && g[intent.supportSide].isLoaded &&
            insideConvexHullXY(comXY,g[intent.supportSide].footprintXY);
          const strictReady=!!s?.preLiftReadiness?.ready && comInsideSelectedSupport;
          if((phase==="SHIFT_WEIGHT" || phase==="LIFT") && strictReady){
            strictReadySince??=state.time;strictReadyDuration=state.time-strictReadySince;
            maxStrictReadyDuration=Math.max(maxStrictReadyDuration,strictReadyDuration);
            maxReadySamples=Math.max(maxReadySamples,++readySamples);
          }else{strictReadySince=null;strictReadyDuration=0;readySamples=0;}
          const validAir=!!swing && !!initialSwing && swing[2]-initialSwing[2]!>.03 &&
            swingN!==null && swingN<25 && pelvis>.82 && tilt<25 &&
            ["LIFT","SWING","LAND"].includes(phase);
          if(validAir){flightSince??=state.time;maxFlightS=Math.max(maxFlightS,state.time-flightSince);}
          else flightSince=null;
          if(support && initialSupport)supportDriftM=Math.max(supportDriftM,
            Math.hypot(support[0]-initialSupport[0]!,support[1]-initialSupport[1]!));
          if(s?.stepContactConfirmed && swing){
            firstTouch??=swing;
            landingDriftM=Math.max(landingDriftM,Math.hypot(swing[0]-firstTouch[0]!,swing[1]-firstTouch[1]!));
          }
          const stable=!!s?.stepContactConfirmed && g.left.verticalLoadN>40 && g.right.verticalLoadN>40 &&
            insideConvexHullXY(comXY,g.loadedSupportPolygonXY) && insideConvexHullXY(cp,g.loadedSupportPolygonXY) &&
            speed<.18 && pelvis>.82 && tilt<25 && supportDriftM<=.02 && landingDriftM<=.02 && s.targetErrorXYM<=.05;
          if(stable){postStabilitySince??=state.time;maxPostStabilityS=Math.max(maxPostStabilityS,state.time-postStabilitySince);}
          else postStabilitySince=null;
          trace.push({timeS:state.time,phase,coordinator:r.phase,com:b.comWorldM,velocity:b.comVelocityWorldMps,
            capture:cp,leftN:g.left.verticalLoadN,rightN:g.right.verticalLoadN,tiltDeg:tilt,pelvisM:pelvis,
            swingSole:swing,supportSole:support,comInsideSelectedSupport,
            readiness:s?.preLiftReadiness??null,fsmReadyDurationS:s?.preLiftReadyDurationS??0,
            strictReadyDurationS:strictReadyDuration,knees:[sim.jointAngleRad("left_knee"),sim.jointAngleRad("right_knee")],
            control,qpos:state.qpos});
        }
        const r=sim.experimentalReactiveRecoveryStatus()!,s=sim.experimentalStepStatus();
        const b=sim.humanoidBalanceObservation();
        const finalSpeedMps=Math.hypot(b.comVelocityWorldMps[0],b.comVelocityWorldMps[1]);
        const shiftStart=transitions.find(t=>t.phase==="SHIFT_WEIGHT")?.timeS;
        const shiftEnd=shiftStart===undefined ? undefined : transitions.find(t=>t.timeS>shiftStart)?.timeS;
        const shiftTrace=trace.filter(t=>t.phase==="SHIFT_WEIGHT");
        const request=r.lastIntent?.timeS;
        const requestLoadEvolution=request===undefined ? [] : [0,.02,.04,.08,.12].map(offset=>{
          const t=trace.find(t=>t.timeS>=request+offset-1e-8)!;
          return {offsetS:offset,timeS:t.timeS,leftN:t.leftN,rightN:t.rightN,com:t.com,
            velocity:t.velocity,capture:t.capture};
        });
        const row={label:c.label,options,totalMassKg:sim.experimentalControlObservation().totalMassKg,
          peakN,impulseNs,firstWarningS,firstActualCaptureExitS,intent:r.lastIntent,intentLoads,
          requestLoadEvolution,shiftDurationS:shiftStart===undefined?null:(shiftEnd??6)-shiftStart,
          maxSupportFractionDuringShift:Math.max(0,...shiftTrace.map(t=>t.readiness?.supportFraction??0)),
          firstLiftS:transitions.find(t=>t.phase==="LIFT")?.timeS??null,
          firstSwingS:transitions.find(t=>t.phase==="SWING")?.timeS??null,
          transitions,firstFallS,minPelvisM,maxTiltDeg,finalSpeedMps,
          strictTransferDurationS:maxStrictReadyDuration,maxReadySamples,maxFlightS,
          supportDriftM,landingDriftM,postStabilityS:maxPostStabilityS,step:s,
          finalOutcome:firstFallS!==null ? "FALL" : s?.phase ?? (r.attempts===0 ? "NO_STEP" : r.phase),
          complete:s?.phase==="COMPLETE" && s.stepContactConfirmed && firstFallS===null &&
            maxStrictReadyDuration>=.08 && maxReadySamples>=4 && maxFlightS>=.04 &&
            s.currentTargetProgressM>=.055 && maxPostStabilityS>=1 && supportDriftM<=.02 && landingDriftM<=.02,
        };
        await writeFile(resolve(out,c.label+".json"),JSON.stringify({summary:row,trace}));
        rows.push(row);
        console.log("LOAD-AWARE-PHYSICS "+JSON.stringify(row));
        expect(r.attempts).toBeLessThanOrEqual(1);
        expect(stateFinite(sim)).toBe(true);
        if(row.complete)expect(firstFallS).toBeNull();
      } finally {sim.dispose();}
    }
    await writeFile(resolve(out,"summary.json"),JSON.stringify({base:"90727886",timestepS:.002,
      durationS:6,forceSite:"sternum_impact",pulseDurationS:.16,rows},null,2));
    expect(rows.length).toBe(process.env.HPL_CASE ? rows.length : 12);
  },300000);
});

function stateFinite(sim:MujocoSimulation):boolean {
  return [...sim.state().qpos,...sim.state().qvel].every(Number.isFinite);
}
