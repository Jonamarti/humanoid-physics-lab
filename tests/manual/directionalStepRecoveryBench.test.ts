import {readFile} from "node:fs/promises";
import {describe,expect,it} from "vitest";
import {MujocoSimulation} from "../../src/physics/mujoco";
import {editedForceAtTime,editedForceImpulseNs,type EditedForce} from "../../src/physics/forceSequence";
import {capturePointXY,insideConvexHullXY} from "../../src/control/balanceSensors";
import {rootTiltDegrees} from "../../src/physics/standingPush";

/** Honest physical diagnostic rather than a passing animation.
 * Run explicitly:
 * HPL_RUN_DIRECTIONAL_PUSHES=1 npx vitest run tests/manual/directionalStepRecoveryBench.test.ts
 * One reactive step is currently the maximum per reset. Chained pulses
 * are NEGATIVE capability tests, not evidence of multi-step walking.
 */
const enabled=process.env.HPL_RUN_DIRECTIONAL_PUSHES==="1";
type Direction="forward"|"backward"|"left"|"right";
const directionForce=(dir:Direction,n:number):readonly [number,number,number]=>{
  switch(dir){
    case "forward":return [n,0,0];
    case "backward":return [-n,0,0];
    case "left":return [0,n,0];
    case "right":return [0,-n,0];
  }
};
const makePulse=(id:string,direction:Direction,n:number,timeS:number):EditedForce=>({
  id,site:"sternum_impact",forceN:directionForce(direction,n),
  startS:timeS,durationS:0.16,waveform:"half-sine",
});
type Case={
  label:string;forces:readonly EditedForce[];durationS:number;
};
const singleCases:Case[]=[
  ...([120,200,300] as const).map(n=>({
    label:"forward-"+n,forces:[makePulse("f","forward",n,1)],durationS:6,
  })),
  ...([100,160,240] as const).map(n=>({
    label:"backward-"+n,forces:[makePulse("b","backward",n,1)],durationS:6,
  })),
  ...([120,200,300] as const).map(n=>({
    label:"left-"+n,forces:[makePulse("l","left",n,1)],durationS:6,
  })),
  ...([120,200,300] as const).map(n=>({
    label:"right-"+n,forces:[makePulse("r","right",n,1)],durationS:6,
  })),
];
const chainedCases:Case[]=[
  {label:"lateral-opposite-chain",forces:[
    makePulse("l1","left",160,1),makePulse("r2","right",160,3.4)],durationS:7},
  {label:"forward-repeat-chain",forces:[
    makePulse("f1","forward",160,1),makePulse("f2","forward",160,3.4)],durationS:7},
];

async function physicalCase(xml:string,c:Case) {
  const sim=await MujocoSimulation.create(xml);
  try {
    sim.enableExperimentalStanding();
    sim.enableExperimentalReactiveRecovery();
    const initial=sim.humanoidBalanceObservation().comWorldM;
    let minPelvisM=Infinity,maxRootTiltDeg=0,maxSoleRiseM=0;
    let maxComDisplacementM=0,maxCaptureOutsideM=0;
    let maxStepReachM=0,peakFootLoadN=0;
    let seenPhysicalLift=false,seenPhysicalSwing=false;
    let wasOutside=false,firstCaptureExitS:number|null=null;
    let firstLiftS:number|null=null,firstLandingS:number|null=null;
    let firstFallS:number|null=null;
    let lastPhase="";const phaseHistory:string[]=[];
    let minAirborneSoleClearanceM=Infinity;
    let postTouchBilateralS=0;
    const transferTrace:unknown[]=[];
    const gateFailures:Record<string,number>={};
    let readyFrames=0,consecutiveReady=0,maxConsecutiveReady=0;
    let minSwingLoadN=Infinity,maxSupportFractionInShift=0;
    let maxFsmReadyDurationS=0;
    for(let i=0;i<Math.round(c.durationS/0.02);i++){
      sim.step(10,timeS=>{
        for(const f of c.forces){
          const force=editedForceAtTime(timeS,f);
          if(force.some(n=>n!==0))sim.applyWorldForceAtSite(f.site,force);
        }
      });
      const state=sim.state();
      const balance=sim.humanoidBalanceObservation();
      const support=sim.humanoidGroundSupportObservation();
      const step=sim.experimentalStepStatus();
      const reactive=sim.experimentalReactiveRecoveryStatus();
      if(!reactive)throw Error("Coordinador reactivo desactivado");
      minPelvisM=Math.min(minPelvisM,state.qpos[2]!);
      maxRootTiltDeg=Math.max(maxRootTiltDeg,rootTiltDegrees(state.qpos));
      maxComDisplacementM=Math.max(maxComDisplacementM,
        Math.hypot(balance.comWorldM[0]-initial[0],balance.comWorldM[1]-initial[1]));
      const cp=capturePointXY([balance.comWorldM[0],balance.comWorldM[1]],
        [balance.comVelocityWorldMps[0],balance.comVelocityWorldMps[1]],
        Math.max(0.05,balance.comWorldM[2]));
      const isOutside=!insideConvexHullXY(cp,support.loadedSupportPolygonXY);
      if(isOutside && !wasOutside && firstCaptureExitS===null)firstCaptureExitS=state.time;
      wasOutside=isOutside;
      if(isOutside) maxCaptureOutsideM=Math.max(maxCaptureOutsideM,
        Math.hypot(cp[0]-balance.comWorldM[0],cp[1]-balance.comWorldM[1]));
      peakFootLoadN=Math.max(peakFootLoadN,support.left.verticalLoadN,support.right.verticalLoadN);
      if(step?.phase==="SHIFT_WEIGHT" && reactive.lastIntent){
        const swing=reactive.lastIntent.swingSide;
        const stance=reactive.lastIntent.supportSide;
        const swingN=support[swing].verticalLoadN;
        const supportN=support[stance].verticalLoadN;
        const comInside=insideConvexHullXY(
          [balance.comWorldM[0],balance.comWorldM[1]],support.loadedSupportPolygonXY);
        // The FSM observed the same solved 50 Hz step; read its own
        // release gates/dwell rather than approximating them independently.
        const readiness=step.preLiftReadiness;
        if(!readiness)throw Error("Falta lectura de pre-despegue de la FSM");
        const failed=Object.entries(readiness.gates)
          .filter(([,pass])=>!pass).map(([name])=>name);
        for(const name of failed)gateFailures[name]=(gateFailures[name]??0)+1;
        if(readiness.ready){
          readyFrames++;
          consecutiveReady++;
          maxConsecutiveReady=Math.max(maxConsecutiveReady,consecutiveReady);
        }else consecutiveReady=0;
        minSwingLoadN=Math.min(minSwingLoadN,swingN);
        maxSupportFractionInShift=Math.max(maxSupportFractionInShift,readiness.supportFraction);
        maxFsmReadyDurationS=Math.max(maxFsmReadyDurationS,step.longestPreLiftReadyDurationS);
        if(transferTrace.length<100)transferTrace.push({
          t:Number(state.time.toFixed(3)),swing,stance,swingN,supportN,
          fraction:readiness.supportFraction,
          comXY:[balance.comWorldM[0],balance.comWorldM[1]],
          velocityXY:[balance.comVelocityWorldMps[0],balance.comVelocityWorldMps[1]],
          captureXY:cp,captureInside:!isOutside,comInside,
          pelvisM:state.qpos[2]!,tiltDeg:rootTiltDegrees(state.qpos),
          gates:readiness.gates,failed,
          fsmReadyDurationS:step.preLiftReadyDurationS,
        });
      }
      if(step){
        maxSoleRiseM=Math.max(maxSoleRiseM,step.maxSoleRiseM);
        const phase=step.phase;
        if(phase!==lastPhase){phaseHistory.push(phase);lastPhase=phase;}
        if(phase==="LIFT" && firstLiftS===null)firstLiftS=state.time;
        if(phase==="SWING")seenPhysicalSwing=true;
        if(["SWING","LAND","RECOVER","COMPLETE"].includes(phase) &&
          step.maxSoleRiseM>0.03)seenPhysicalLift=true;
        if(["LIFT","SWING"].includes(phase)){
          const side=reactive.lastIntent?.swingSide;
          if(side && support[side].verticalLoadN<25){
            // Sole site in world coordinates; detect clearance rather than
            // calling arbitrary whole-body displacement a lifted foot.
            const sole=sim.siteWorldPosition(side+"_sole");
            minAirborneSoleClearanceM=Math.min(minAirborneSoleClearanceM,sole[2]);
          }
        }
        if(step.stepContactConfirmed && firstLandingS===null)firstLandingS=state.time;
        if(step.stepContactConfirmed && support.left.verticalLoadN>40 &&
            support.right.verticalLoadN>40)postTouchBilateralS+=0.02;
      }
      if((step?.phase==="FALL"||state.qpos[2]!<0.65) && firstFallS===null)firstFallS=state.time;
      if(reactive.lastIntent){
        maxStepReachM=Math.max(maxStepReachM,Math.hypot(...reactive.lastIntent.deltaXY));
      }
    }
    const reactive=sim.experimentalReactiveRecoveryStatus()!;
    const physical=sim.experimentalStepStatus();
    const finalBalance=sim.humanoidBalanceObservation();
    const finalGround=sim.humanoidGroundSupportObservation();
    const finalSpeedMps=Math.hypot(...finalBalance.comVelocityWorldMps.slice(0,2));
    // This is an OBSERVATION, not a substitute for the original step FSM:
    const stablePosture=minPelvisM>0.82&&maxRootTiltDeg<10&&finalSpeedMps<0.12 &&
      finalGround.left.verticalLoadN>40&&finalGround.right.verticalLoadN>40;
    const verifiedStep=physical?.phase==="COMPLETE"&&physical.stepContactConfirmed &&
      physical.settledDurationS>=1&&stablePosture;
    return {
      label:c.label,
      impulsesNs:c.forces.map(f=>({
        id:f.id,vector:editedForceImpulseNs(f),
        peakN:Math.hypot(...f.forceN),startS:f.startS,
      })),
      attempts:reactive.attempts,plannerPhase:reactive.phase,
      firstCaptureExitS,firstLiftS,firstLandingS,firstFallS,
      plannedStepM:reactive.lastIntent?.deltaXY??null,
      maxStepReachM,maxSoleRiseM,
      minAirborneSoleClearanceM:Number.isFinite(minAirborneSoleClearanceM)?
        minAirborneSoleClearanceM:null,
      seenPhysicalLift,seenPhysicalSwing,peakFootLoadN,
      maxComDisplacementM,maxCaptureOutsideM,minPelvisM,maxRootTiltDeg,
      postTouchBilateralS,finalSpeedMps,
      transferDiagnosis:{
        samples:transferTrace.length,readyFrames,maxConsecutiveReady,
        minSwingLoadN:Number.isFinite(minSwingLoadN)?minSwingLoadN:null,
        maxSupportFractionInShift,maxFsmReadyDurationS,gateFailures,trace:transferTrace,
      },
      phaseHistory,physicalPhase:physical?.phase??null,
      landed:physical?.stepContactConfirmed??false,
      settledS:physical?.settledDurationS??0,
      stablePosture,verifiedStep,
    };
  }finally{sim.dispose();}
}

describe.skipIf(!enabled)("Physical four-way push strength and chained pulses: report true recovery",()=>{
  it("runs forward, backward, left and right at graded impulses",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const rows=[];
    for(const c of singleCases){
      const row=await physicalCase(xml,c);
      rows.push(row);
      console.log("DIRECTIONAL-STEP-PHYSICS "+JSON.stringify(row));
      expect(row.attempts).toBeLessThanOrEqual(1);
      expect(row.maxStepReachM).toBeLessThanOrEqual(0.38+1e-8);
      if(!row.landed)expect(row.verifiedStep).toBe(false);
      if(row.verifiedStep)expect(row.settledS).toBeGreaterThanOrEqual(1);
    }
    expect(rows).toHaveLength(12);
  },300000);
  it("measures opposite-side and repeated push sequences without pretending multi-step works",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const rows=[];
    for(const c of chainedCases){
      const row=await physicalCase(xml,c);rows.push(row);
      console.log("CHAINED-STEP-PHYSICS "+JSON.stringify(row));
      // Currently single-attempt limitation: document with a negative
      // capability test; a second step is NOT implemented or certified.
      expect(row.attempts).toBeLessThanOrEqual(1);
      if(!row.landed)expect(row.verifiedStep).toBe(false);
    }
    expect(rows).toHaveLength(2);
  },180000);
});
