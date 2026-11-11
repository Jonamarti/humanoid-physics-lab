import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { rootTiltDegrees } from "../../src/physics/standingPush";
import { editedForceAtTime, type EditedForce } from "../../src/physics/forceSequence";

const enabled=process.env.HPL_RUN_REACTIVE_BENCH==="1";
type Outcome={
  label:string;strategy:"whole-body"|"reactive-step";
  minPelvisM:number;maxTiltDeg:number;finalComDisplacementM:number;
  stepAttempts:number;stepPhases:string[];plannerPhase:string|null;
  swingFoot:string|null;plannedDeltaXYM:readonly number[]|null;
  maxSoleRiseM:number;stepContactConfirmed:boolean;
  settledDurationS:number;finalPhase:string|null;recovered:boolean;
};
async function trial(xml:string,label:string,forces:readonly EditedForce[],
  reactive:boolean):Promise<Outcome>{
  const sim=await MujocoSimulation.create(xml);
  try{
    sim.enableExperimentalStanding();
    if(reactive)sim.enableExperimentalReactiveRecovery();
    else sim.enableExperimentalWholeBodyBalance();
    const origin=sim.humanoidBalanceObservation().comWorldM;
    let minPelvisM=Infinity,maxTiltDeg=0;
    const phaseChanges:string[]=[];
    let lastPhase="";
    for(let i=0;i<350;i++){
      sim.step(10,t=>{
        for(const f of forces){
          const vec=editedForceAtTime(t,f);
          if(vec.some(v=>v!==0))sim.applyWorldForceAtSite(f.site,vec);
        }
      });
      const state=sim.state();
      minPelvisM=Math.min(minPelvisM,state.qpos[2]!);
      maxTiltDeg=Math.max(maxTiltDeg,rootTiltDegrees(state.qpos));
      const step=sim.experimentalStepStatus();
      if(step && step.phase!==lastPhase){
        phaseChanges.push(step.phase);
        lastPhase=step.phase;
      }
    }
    const b=sim.humanoidBalanceObservation();
    const state=sim.state(), step=sim.experimentalStepStatus();
    const r=sim.experimentalReactiveRecoveryStatus();
    const comDelta=Math.hypot(b.comWorldM[0]-origin[0],b.comWorldM[1]-origin[1]);
    const speed=Math.hypot(b.comVelocityWorldMps[0],b.comVelocityWorldMps[1]);
    return {
      label,strategy:reactive?"reactive-step":"whole-body",
      minPelvisM,maxTiltDeg,finalComDisplacementM:comDelta,
      stepAttempts:r?.attempts??0,stepPhases:phaseChanges,
      plannerPhase:r?.phase??null,swingFoot:r?.lastIntent?.swingSide??null,
      plannedDeltaXYM:r?.lastIntent?.deltaXY??null,
      maxSoleRiseM:step?.maxSoleRiseM??0,
      stepContactConfirmed:step?.stepContactConfirmed??false,
      settledDurationS:step?.settledDurationS??0,
      finalPhase:step?.phase??null,
      recovered:minPelvisM>.86 && maxTiltDeg<25 && speed<.08 &&
        comDelta<.03 && (!step || step.phase==="COMPLETE"),
    };
  }finally{sim.dispose();}
}
const push=(id:string,site:EditedForce["site"],
  forceN:EditedForce["forceN"],startS=1,durationS=.16):EditedForce=>({
    id,site,forceN,startS,durationS,waveform:"half-sine",
});
const scenarios=[
  ["backward-240",[push("a","sternum_impact",[-240,0,0])]],
  ["forward-240",[push("a","sternum_impact",[240,0,0])]],
  ["left-240",[push("a","sternum_impact",[0,240,0])]],
  ["backward-diagonal-370",[push("a","sternum_impact",[-370,30,0],.8,.18),
    push("b","left_shoulder_impact",[0,70,0],1,.1)]],
] as const;

describe.skipIf(!enabled)("Manual v3 reactive step vs whole-body without step",()=>{
  it("reports paired physical stepping outcomes and never invents recovered steps",async()=>{
    const xml=await readFile(
      new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    for(const [name,forces] of scenarios){
      const old=await trial(xml,name,forces,false);
      const attempted=await trial(xml,name,forces,true);
      console.log("REACTIVE-V3-BENCH "+JSON.stringify({baseline:old,reactive:attempted}));
      expect(attempted.stepAttempts).toBeLessThanOrEqual(1);
      expect(attempted.minPelvisM).toBeGreaterThanOrEqual(0);
      if(attempted.stepAttempts>0){
        expect(attempted.plannedDeltaXYM).not.toBeNull();
        if(attempted.finalPhase!=="COMPLETE")expect(attempted.recovered).toBe(false);
        if(attempted.stepContactConfirmed===false)
          expect(attempted.finalPhase).not.toBe("COMPLETE");
      } else {
        expect(attempted.stepPhases).toEqual([]);
      }
    }
  },300000);
});
