import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { pushForceAtTime, expectedPushImpulseNs, rootTiltDegrees,
  type PushDirection, type PushTrial } from "../../src/physics/standingPush";

/** One-off on CI or explicitly local: HPL_RUN_WB_BENCH=1 npx vitest run
 * tests/manual/wholeBodyBalanceBench.test.ts
 * The test reports measured gains AND regressions; success means execution,
 * never that a policy has been proven to recover a perturbation.
 */
const enabled=process.env.HPL_RUN_WB_BENCH==="1";
const round=(v:number)=>+v.toFixed(4);

interface Result {
  readonly policy:"ankles"|"whole-body";
  readonly direction:PushDirection;
  readonly peakForceN:number;
  readonly impulseNs:number;
  readonly minimumPelvisM:number;
  readonly maxRootTiltDeg:number;
  readonly finalComSpeedMps:number;
  readonly finalComDisplacementM:number;
  readonly rearfootDriftM:number;
  readonly lateSettled:boolean;
  readonly fall:boolean;
  readonly recovered:boolean;
  readonly activeWholeBodySamples:number;
  readonly maxRequestedShoulderRad:number;
}

async function tryPush(xml:string,direction:PushDirection,peakForceN:number,
  wholeBody:boolean):Promise<Result>{
  const sim=await MujocoSimulation.create(xml);
  const trial:PushTrial={direction,peakForceN,startS:1,durationS:0.12};
  try {
    sim.enableExperimentalStanding();
    if(wholeBody)sim.enableExperimentalWholeBodyBalance();
    const initial=sim.humanoidBalanceObservation();
    const rearfootIds=sim.geometrySpecs().filter(g=>
      g.shape==="box" && Math.abs(g.size[0]-0.0825)<1e-8).map(g=>g.id);
    if(rearfootIds.length!==2)throw new Error("Se requieren dos retropiés v3");
    const feet=()=>new Map(sim.geometryPoses().filter(p=>
      rearfootIds.includes(p.id)).map(p=>[p.id,p.position]));
    const startingFeet=feet();
    let minimumPelvisM=Infinity,maxRootTiltDeg=0,lateSettled=true;
    let maxRequestedShoulderRad=0,activeWholeBodySamples=0;
    for(let i=0;i<300;i++){
      const state=sim.step(10,t=>{
        const f=pushForceAtTime(t,trial);
        if(f.some(x=>x!==0))sim.applyWorldForceAtSite("sternum_impact",f);
      });
      minimumPelvisM=Math.min(minimumPelvisM,state.qpos[2]!);
      const tilt=rootTiltDegrees(state.qpos);
      maxRootTiltDeg=Math.max(maxRootTiltDeg,tilt);
      const obs=sim.humanoidBalanceObservation();
      const speed=Math.hypot(obs.comVelocityWorldMps[0],obs.comVelocityWorldMps[1]);
      const displacement=Math.hypot(obs.comWorldM[0]-initial.comWorldM[0],
        obs.comWorldM[1]-initial.comWorldM[1]);
      if(state.time>5 && (speed>0.08 || displacement>0.03 || tilt>8))
        lateSettled=false;
      const cmd=sim.experimentalWholeBodyBalanceStatus();
      if(cmd?.active){
        activeWholeBodySamples++;
        maxRequestedShoulderRad=Math.max(maxRequestedShoulderRad,
          Math.abs(cmd.targetsRad.left_shoulder_pitch??0),
          Math.abs(cmd.targetsRad.left_shoulder_roll??0),
          Math.abs(cmd.targetsRad.left_shoulder_yaw??0));
      }
    }
    const last=sim.humanoidBalanceObservation();
    const finalComSpeedMps=Math.hypot(last.comVelocityWorldMps[0],
      last.comVelocityWorldMps[1]);
    const finalComDisplacementM=Math.hypot(last.comWorldM[0]-initial.comWorldM[0],
      last.comWorldM[1]-initial.comWorldM[1]);
    const lastFeet=feet();
    const rearfootDriftM=Math.max(...Array.from(lastFeet,([id,p])=>
      Math.hypot(p[0]-startingFeet.get(id)![0],p[1]-startingFeet.get(id)![1])));
    const fall=minimumPelvisM<0.7 || maxRootTiltDeg>50;
    return {
      policy:wholeBody?"whole-body":"ankles",direction,peakForceN,
      impulseNs:round(expectedPushImpulseNs(trial)),
      minimumPelvisM:round(minimumPelvisM),
      maxRootTiltDeg:round(maxRootTiltDeg),
      finalComSpeedMps:round(finalComSpeedMps),
      finalComDisplacementM:round(finalComDisplacementM),
      rearfootDriftM:round(rearfootDriftM),
      lateSettled,fall,
      recovered:!fall && minimumPelvisM>0.86 && maxRootTiltDeg<25 &&
        rearfootDriftM<0.02 && lateSettled,
      activeWholeBodySamples,
      maxRequestedShoulderRad:round(maxRequestedShoulderRad),
    };
  }finally{sim.dispose();}
}

describe.skipIf(!enabled)("Manual v3 whole-body balance — evidence against unmodified baseline",()=>{
  it("verifies nominal 10-second stance is not spoiled by the opt-in controller",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const sim=await MujocoSimulation.create(xml);
    try{
      sim.enableExperimentalStanding();
      sim.enableExperimentalWholeBodyBalance();
      let lowest=Infinity,maxTilt=0;
      for(let i=0;i<500;i++){
        const state=sim.step(10);
        lowest=Math.min(lowest,state.qpos[2]!);
        maxTilt=Math.max(maxTilt,rootTiltDegrees(state.qpos));
      }
      console.log("WHOLE-BODY-STAND-10S "+JSON.stringify({
        pelvisMinM:round(lowest),maxTiltDeg:round(maxTilt),
        finalPelvisM:round(sim.state().qpos[2]!),
      }));
      expect(sim.state().time).toBeCloseTo(10,5);
    }finally{sim.dispose();}
  },120000);
  it("measures mirror-direction push recovery for both controllers",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const all:Result[]=[];
    for(const direction of ["forward","backward","left","right"] as const){
      for(const peakForceN of [120,240]){
        const baseline=await tryPush(xml,direction,peakForceN,false);
        const enhanced=await tryPush(xml,direction,peakForceN,true);
        all.push(baseline,enhanced);
        console.log("WHOLE-BODY-PUSH "+JSON.stringify({
          direction,peakForceN,baseline,enhanced,
          deltaMaxTiltDeg:round(enhanced.maxRootTiltDeg-baseline.maxRootTiltDeg),
          deltaMinPelvisM:round(enhanced.minimumPelvisM-baseline.minimumPelvisM),
        }));
      }
    }
    expect(all).toHaveLength(16);
    expect(all.every(row=>Object.values(row).every(v=>
      typeof v==="string" || typeof v==="boolean" || Number.isFinite(v)))).toBe(true);
    // No assert on recovery count: FAIL and regressions are legitimate results.
  },300000);
});
