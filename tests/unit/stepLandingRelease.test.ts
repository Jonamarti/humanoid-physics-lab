import { describe, expect, it } from "vitest";
import { ExperimentalStepSequence, type StepFeedback } from "../../src/control/experimentalStep";

const observation = (timeS: number, overrides: Partial<StepFeedback> = {}): StepFeedback => ({
  timeS, comWorldM:[0,0,0.96], comVelocityWorldMps:[0,0,0],
  comProjectionInsideLoadedSupport:true, capturePointInsideLoadedSupport:true,
  pelvisHeightM:0.99, rootTiltDeg:2,
  swingSoleWorldM:[0.065,0.105,0.001],
  supportSoleWorldM:[0.065,-0.105,0.001],
  supportLoadN:370, swingLoadN:370, ...overrides,
});

describe("Optional smooth joint release after REAL loaded landing", () => {
  const landed = (releaseS: number) => {
    const s=new ExperimentalStepSequence("left",[0.12,0.08],0.8,releaseS);
    s.observe(observation(0.05));
    s.observe(observation(0.8));
    s.observe(observation(1.08,{supportLoadN:610,swingLoadN:110}));
    s.observe(observation(1.2,{supportLoadN:735,swingLoadN:0,
      swingSoleWorldM:[0.065,0.105,0.061]}));
    s.observe(observation(1.4,{supportLoadN:735,swingLoadN:0,
      swingSoleWorldM:[0.185,0.185,0.061]}));
    s.observe(observation(1.56,{supportLoadN:500,swingLoadN:230,
      swingSoleWorldM:[0.185,0.185,0.003]}));
    expect(s.snapshot().phase).toBe("RECOVER");
    return s;
  };
  it("preserves current behavior unless explicitly enabled", () => {
    const nominal=landed(0);
    const atContact=nominal.command(1.56,[0,0]).targetsRad.left_hip_pitch!;
    const later=nominal.command(2.2,[0,0]).targetsRad.left_hip_pitch!;
    expect(atContact).toBe(later);
  });
  it("smoothly decays all residual swing-joint targets toward the neutral PD", () => {
    const step=landed(0.8);
    const start=step.command(1.56,[0,0]);
    const mid=step.command(1.96,[0,0]);
    const end=step.command(2.36,[0,0]);
    for(const name of ["left_hip_pitch","left_hip_roll","left_knee","left_ankle_pitch"]) {
      const initial=start.targetsRad[name]!;
      expect(mid.targetsRad[name]).toBeCloseTo(initial*0.5,8);
      expect(end.targetsRad[name]).toBeCloseTo(0,8);
    }
    expect(start.soleTargetWorldM).toBeNull();
    expect(end.cartesianBlend).toBe(0);
  });
  it("rejects invalid release windows", () => {
    expect(() => new ExperimentalStepSequence("left",[0.1,0],0.8,-1)).toThrow(RangeError);
    expect(() => new ExperimentalStepSequence("left",[0.1,0],0.8,Infinity)).toThrow(RangeError);
  });
});
