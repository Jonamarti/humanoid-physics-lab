import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { WHOLE_BODY_JOINTS, wholeBodyBalanceCommand, armOnlyBalanceTargets } from "../../src/control/wholeBodyBalance";
import { pushForceAtTime, type PushTrial } from "../../src/physics/standingPush";

const reading=(h:readonly [number,number,number])=>({
  angularMomentumWorldNms:h,
  rootAngularVelocityWorldRadS:[0,0,0] as const,
  totalMassKg:75,solvedFootLoadN:735,
});

describe("Opt-in whole-body momentum redistribution",()=>{
  it("allows only six finite internal shoulder targets during reactive transfer",()=>{
    const body=wholeBodyBalanceCommand(reading([10,12,8])).targetsRad;
    const arms=armOnlyBalanceTargets(body);
    expect(Object.keys(arms).sort()).toEqual([
      "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
      "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
    ].sort());
    expect(arms.left_hip_pitch).toBeUndefined();
    expect(arms.lumbar_roll).toBeUndefined();
    expect(arms.pelvis_root).toBeUndefined();
    expect(Object.values(arms).every(v=>Number.isFinite(v)&&Math.abs(v)<=0.33)).toBe(true);
    expect(armOnlyBalanceTargets({pelvis_root:999,left_knee:999})).toEqual({});
    expect(()=>armOnlyBalanceTargets({left_shoulder_pitch:NaN})).toThrow(RangeError);
  });
  it("keeps the reactive arm-transfer experiment explicit and v3-only",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const sim=await MujocoSimulation.create(xml);
    try{
      sim.enableExperimentalStanding();
      expect(()=>sim.enableExperimentalReactiveRecovery(1 as unknown as boolean)).toThrow(RangeError);
      sim.enableExperimentalReactiveRecovery(true);
      sim.step(25);
      expect(sim.experimentalReactiveRecoveryStatus()?.attempts).toBe(0);
      expect(sim.experimentalStepStatus()).toBeNull();
      sim.disableExperimentalReactiveRecovery();
      sim.enableExperimentalReactiveRecovery();
      expect(sim.experimentalReactiveRecoveryStatus()?.attempts).toBe(0);
    }finally{sim.dispose();}
  });

  it("does not move limbs at zero momentum or with no solved floor support",()=>{
    expect(wholeBodyBalanceCommand(reading([0,0,0])).targetsRad).toEqual({});
    expect(wholeBodyBalanceCommand({...reading([10,0,0]),
      solvedFootLoadN:0}).targetsRad).toEqual({});
    expect(wholeBodyBalanceCommand({...reading([10,0,0]),
      solvedFootLoadN:10}).active).toBe(false);
  });
  it("chooses distinct shoulder, hip and spine corrections for 3 axes",()=>{
    for(const [axis,primary] of [
      [0,"left_shoulder_roll"],[1,"left_shoulder_pitch"],
      [2,"left_shoulder_yaw"],
    ] as const){
      const h=[0,0,0] as [number,number,number];
      h[axis]=10;
      const positive=wholeBodyBalanceCommand(reading(h));
      h[axis]=-10;
      const negative=wholeBodyBalanceCommand(reading(h));
      expect(positive.active).toBe(true);
      expect(positive.targetsRad[primary]).toBeGreaterThan(0);
      expect(negative.targetsRad[primary]).toBeCloseTo(-positive.targetsRad[primary]!,9);
      expect(Object.keys(positive.targetsRad).every(k=>
        (WHOLE_BODY_JOINTS as readonly string[]).includes(k))).toBe(true);
    }
    const pitch=wholeBodyBalanceCommand(reading([0,10,0])).targetsRad;
    expect(pitch.left_hip_pitch).toBeGreaterThan(0);
    expect(pitch.right_hip_pitch).toBeCloseTo(pitch.left_hip_pitch!,9);
    expect(pitch.lumbar_pitch).toBeGreaterThan(0);
    expect(pitch.left_shoulder_pitch).toBeGreaterThan(pitch.left_hip_pitch!);
    expect(pitch.pelvis_root).toBeUndefined();
  });
  it("limits all internal joint target angles under arbitrarily high impulses",()=>{
    const a=wholeBodyBalanceCommand({
      ...reading([100_000,100_000,100_000]),
      rootAngularVelocityWorldRadS:[50,50,50],
    });
    expect(Object.values(a.targetsRad).every(v=>Number.isFinite(v) && Math.abs(v)<=0.33)).toBe(true);
    expect(a.normalizedMomentumXYZ).toEqual([1,1,1]);
    expect(()=>wholeBodyBalanceCommand({...reading([1,0,0]),solvedFootLoadN:-1}))
      .toThrow(RangeError);
    expect(()=>wholeBodyBalanceCommand({...reading([1,0,0]),
      angularMomentumWorldNms:[NaN,0,0]})).toThrow(RangeError);
  });
  it("never enables on v1/v2 and retains opt-in after reset of v3",async()=>{
    const xml=async(file:string)=>readFile(
      new URL("../../public/models/"+file,import.meta.url),"utf8");
    const [v1,v3]=await Promise.all([
      MujocoSimulation.create(await xml("humanoid_v1.xml")),
      MujocoSimulation.create(await xml("humanoid_gait_v3.xml")),
    ]);
    try{
      v1.enableExperimentalStanding();
      expect(()=>v1.enableExperimentalWholeBodyBalance()).toThrow();
      expect(v1.experimentalWholeBodyBalanceStatus()).toBeNull();
      v3.enableExperimentalStanding();
      v3.enableExperimentalWholeBodyBalance();
      v3.step(50);
      expect(v3.experimentalWholeBodyBalanceStatus()).not.toBeNull();
      const t=v3.state().time;
      expect(t).toBeCloseTo(0.1,8);
      v3.reset();
      expect(v3.experimentalWholeBodyBalanceStatus()).toBeNull();
      v3.step(50);
      expect(v3.experimentalWholeBodyBalanceStatus()).not.toBeNull();
      // No automatic step while this experimental controller owns the arms.
      expect(()=>v3.enableExperimentalStepTrial("left")).toThrow();
      v3.disableExperimentalWholeBodyBalance();
      v3.enableExperimentalStepTrial("left");
      expect(v3.experimentalStepStatus()?.phase).toBe("STAND");
    }finally{v1.dispose();v3.dispose();}
  },45000);
  it("receives a physical perturbation and generates only bounded limb targets",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const sim=await MujocoSimulation.create(xml);
    const trial:PushTrial={direction:"forward",peakForceN:160,startS:0.45,durationS:0.12};
    try{
      sim.enableExperimentalStanding();
      sim.enableExperimentalWholeBodyBalance();
      let observedNonzero=false;
      for(let n=0;n<100;n++){
        sim.step(10,t=>{
          const force=pushForceAtTime(t,trial);
          if(force[0]!==0)sim.applyWorldForceAtSite("sternum_impact",force);
        });
        const cmd=sim.experimentalWholeBodyBalanceStatus();
        if(cmd && Math.abs(cmd.targetsRad.left_shoulder_pitch??0)>1e-4)
          observedNonzero=true;
      }
      expect(observedNonzero).toBe(true);
      expect(sim.state().qpos.slice(0,7).every(Number.isFinite)).toBe(true);
      expect(sim.state().time).toBeCloseTo(2,8);
    }finally{sim.dispose();}
  },45000);
});
