import {readFile} from "node:fs/promises";
import {describe,expect,it} from "vitest";
import {MujocoSimulation} from "../../src/physics/mujoco";
import {editedForceAtTime,type EditedForce} from "../../src/physics/forceSequence";
import {rootTiltDegrees} from "../../src/physics/standingPush";

const enabled=process.env.HPL_RUN_ARM_TRANSFER_AB==="1";
type Case={name:string;forceN:readonly [number,number,number]};
const cases:Case[]=[
  {name:"forward-200",forceN:[200,0,0]},
  {name:"backward-160",forceN:[-160,0,0]},
  {name:"left-300",forceN:[0,300,0]},
  {name:"right-300",forceN:[0,-300,0]},
];

async function trial(xml:string,c:Case,armsDuringTransfer:boolean){
  const sim=await MujocoSimulation.create(xml);
  const pulse:EditedForce={
    id:"push",site:"sternum_impact",forceN:c.forceN,
    startS:1,durationS:0.16,waveform:"half-sine",
  };
  try{
    sim.enableExperimentalStanding();
    sim.enableExperimentalReactiveRecovery(armsDuringTransfer);
    let firstFallS:number|null=null,firstLiftS:number|null=null;
    let firstSwingS:number|null=null,firstLandingS:number|null=null;
    let maxSupportFraction=0,maxFsmReadyDurationS=0,minPelvisM=Infinity;
    let maxTiltDeg=0,peakArmTargetRad=0;
    const phases:string[]=[];let lastPhase="";
    for(let frame=0;frame<300;frame++){
      sim.step(10,t=>{
        const force=editedForceAtTime(t,pulse);
        if(force.some(v=>v!==0))sim.applyWorldForceAtSite(pulse.site,force);
      });
      const physical=sim.experimentalStepStatus();
      const reactive=sim.experimentalReactiveRecoveryStatus();
      const state=sim.state();
      const tilt=rootTiltDegrees(state.qpos);
      minPelvisM=Math.min(minPelvisM,state.qpos[2]!);
      maxTiltDeg=Math.max(maxTiltDeg,tilt);
      if(physical){
        maxSupportFraction=Math.max(maxSupportFraction,physical.maxSupportLoadFraction);
        maxFsmReadyDurationS=Math.max(maxFsmReadyDurationS,physical.longestPreLiftReadyDurationS);
        if(physical.phase!==lastPhase){lastPhase=physical.phase;phases.push(physical.phase);}
        if(physical.phase==="LIFT"&&firstLiftS===null)firstLiftS=state.time;
        if(physical.phase==="SWING"&&firstSwingS===null)firstSwingS=state.time;
        if(physical.stepContactConfirmed&&firstLandingS===null)firstLandingS=state.time;
      }
      const wb=sim.experimentalWholeBodyBalanceStatus();
      if(wb?.active){
        for(const [name,rad] of Object.entries(wb.targetsRad)){
          if(name.includes("shoulder"))peakArmTargetRad=Math.max(peakArmTargetRad,Math.abs(rad));
        }
      }
      if((physical?.phase==="FALL"||state.qpos[2]!<0.65||tilt>50)&&firstFallS===null){
        firstFallS=state.time;
      }
      if(!reactive)throw new Error("Missing reactive status");
    }
    const physical=sim.experimentalStepStatus();
    const reactive=sim.experimentalReactiveRecoveryStatus()!;
    return {name:c.name,armsDuringTransfer,
      attempts:reactive.attempts,phases,firstFallS,firstLiftS,firstSwingS,
      firstLandingS,maxSupportFraction,maxFsmReadyDurationS,
      minPelvisM,maxTiltDeg,peakArmTargetRad,
      finalPhase:physical?.phase??null,
      confirmedLanding:physical?.stepContactConfirmed??false,
      settledS:physical?.settledDurationS??0,
      verifiedRecovery:physical?.phase==="COMPLETE"&&physical.stepContactConfirmed&&
        physical.settledDurationS>=1&&firstFallS===null};
  }finally{sim.dispose();}
}

describe.skipIf(!enabled)("v3 opt-in shoulder compensation while weight-shifting: paired physical A/B",()=>{
  it("reports matching 6s MuJoCo push cases without claiming false recovery",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const rows=[];
    for(const c of cases){
      const baseline=await trial(xml,c,false);
      const transferArms=await trial(xml,c,true);
      rows.push({baseline,transferArms});
      console.log("ARM-TRANSFER-AB "+JSON.stringify({baseline,transferArms}));
      for(const trialResult of [baseline,transferArms]){
        expect(trialResult.attempts).toBeLessThanOrEqual(1);
        expect(trialResult.minPelvisM).toBeGreaterThanOrEqual(0);
        expect(trialResult.peakArmTargetRad).toBeLessThanOrEqual(0.33+1e-8);
        if(!trialResult.confirmedLanding)expect(trialResult.verifiedRecovery).toBe(false);
      }
    }
    expect(rows).toHaveLength(4);
  },300000);
});
