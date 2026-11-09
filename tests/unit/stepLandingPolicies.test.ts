import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { ExperimentalStepSequence, type LandingJointPolicy,
  type StepFeedback } from "../../src/control/experimentalStep";

const POSES: readonly LandingJointPolicy[]=[
  "old-target","neutral-pd","hold-contact-pose",
];

describe("Contact-aware swing joint policy",()=>{
  it("reuses actual solved contact pose only when a real landing has occurred",()=>{
    const contactAngles={
      left_hip_pitch:-0.12,
      left_hip_roll:0.04,
      left_knee:-0.31,
      left_ankle_pitch:0.08,
    };
    const feedback=(timeS:number, overrides:Partial<StepFeedback>={}): StepFeedback=>({
      timeS,comWorldM:[0.025,0,0.95],comVelocityWorldMps:[0,0,0],
      comProjectionInsideLoadedSupport:true,capturePointInsideLoadedSupport:true,
      pelvisHeightM:0.99,rootTiltDeg:2,
      swingSoleWorldM:[0.065,0.105,0.001],
      supportSoleWorldM:[0.065,-0.105,0.001],
      supportLoadN:370,swingLoadN:370,
      swingJointAnglesRad:contactAngles,
      ...overrides,
    });
    for (const policy of POSES) {
      const step=new ExperimentalStepSequence("left",[0.10,0.08],0.8,policy);
      step.observe(feedback(0.05));
      step.observe(feedback(0.8));
      step.observe(feedback(1.15,{supportLoadN:605,swingLoadN:115}));
      expect(step.snapshot().phase).toBe("LIFT");
      step.observe(feedback(1.25,{supportLoadN:735,swingLoadN:0,
        swingSoleWorldM:[0.065,0.105,0.061]}));
      expect(step.snapshot().phase).toBe("SWING");
      step.observe(feedback(1.48,{supportLoadN:735,swingLoadN:0,
        swingSoleWorldM:[0.16,0.18,0.062]}));
      expect(step.snapshot().phase).toBe("LAND");
      step.observe(feedback(1.62,{supportLoadN:510,swingLoadN:170,
        swingSoleWorldM:[0.16,0.18,0.003]}));
      expect(step.snapshot().phase).toBe("RECOVER");
      const command=step.command(1.64,[0.025,0]);
      if(policy==="hold-contact-pose") expect(command.targetsRad).toEqual(contactAngles);
      if(policy==="neutral-pd") expect(command.targetsRad).toEqual({});
      if(policy==="old-target") expect(command.targetsRad.left_knee).toBe(-0.05);
      expect(command.cartesianBlend).toBe(0);
    }
    expect(()=>new ExperimentalStepSequence("left",[0.1,0],0,"teleport" as LandingJointPolicy))
      .toThrow(RangeError);
  });

  const diagnostic = process.env.RUN_LONG_BENCHMARKS === "1" ? it : it.skip;
  diagnostic("compares three landing policies with actual MuJoCo contacts", async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows=[];
    for(const policy of POSES) {
      for(const servo of [0,0.2]) {
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo,policy);
          sim.step(2600);
          const result=sim.experimentalStepStatus()!;
          const row={policy,servo,phase:result.phase,
            landing:result.stepContactConfirmed,
            slip:+result.maxLandingSlipM.toFixed(4),
            supportSlip:+result.maxSupportFootSlipM.toFixed(4),
            error:+result.targetErrorXYM.toFixed(4),
            maxTilt:+result.maxRootTiltDeg.toFixed(2),
            minPelvis:+result.minPelvisM.toFixed(3),
            stableS:+result.settledDurationS.toFixed(2)};
          rows.push(row);
          expect(Object.values(row).every(v=>typeof v==="string"||
            typeof v==="boolean"||Number.isFinite(v))).toBe(true);
          if(result.phase==="COMPLETE") {
            expect(result.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(result.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(result.settledDurationS).toBeGreaterThanOrEqual(1);
          }
        } finally {sim.dispose();}
      }
    }
    console.log("LANDING-POLICY-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(6);
  },120000);
});
