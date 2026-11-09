import { describe, expect, it } from "vitest";
import { ExperimentalStepSequence, type StepFeedback } from "../../src/control/experimentalStep";

const sample = (timeS: number, overrides: Partial<StepFeedback> = {}): StepFeedback => ({
  timeS, comWorldM:[0,0,0.95],pelvisHeightM:0.99,rootTiltDeg:2,
  swingSoleWorldM:[0.065,0.105,0.001],supportLoadN:370,swingLoadN:370,
  ...overrides,
});

describe("Recovery-step FSM requires verified contact events", () => {
  it("moves through phases only with load, take-off, swing and re-contact", () => {
    const step = new ExperimentalStepSequence("left",[0.10,0.10],0.8);
    step.observe(sample(0.05));
    expect(step.snapshot().phase).toBe("STAND");
    step.observe(sample(0.8));
    expect(step.snapshot().phase).toBe("SHIFT_WEIGHT");
    const shift = step.command(1.0,[0,0]);
    expect(shift.referenceComXY[1]).toBeLessThan(0); // transfer to right support
    step.observe(sample(1.08,{supportLoadN:600,swingLoadN:120}));
    expect(step.snapshot().phase).toBe("LIFT");
    const lift = step.command(1.09,[0,0]);
    expect(lift.targetsRad.left_knee).toBeLessThan(0);
    step.observe(sample(1.2,{
      supportLoadN:735,swingLoadN:0,swingSoleWorldM:[0.065,0.105,0.06],
    }));
    expect(step.snapshot().phase).toBe("SWING");
    step.observe(sample(1.4,{
      supportLoadN:735,swingLoadN:0,swingSoleWorldM:[0.17,0.19,0.06],
    }));
    expect(step.snapshot().phase).toBe("LAND");
    step.observe(sample(1.56,{
      supportLoadN:590,swingLoadN:150,swingSoleWorldM:[0.17,0.19,0.003],
    }));
    expect(step.snapshot().phase).toBe("RECOVER");
    step.observe(sample(2.04,{
      supportLoadN:410,swingLoadN:330,swingSoleWorldM:[0.17,0.19,0.001],
    }));
    expect(step.snapshot().phase).toBe("COMPLETE");
    expect(step.snapshot().stepContactConfirmed).toBe(true);
  });
  it("reports ABORT if no measured transfer; never labels a timed pose as a step", () => {
    const step = new ExperimentalStepSequence("right",[0.12,-0.08],0.3);
    step.observe(sample(0.3));
    step.observe(sample(1.6));
    expect(step.snapshot().phase).toBe("ABORT");
    expect(step.snapshot().stepContactConfirmed).toBe(false);
  });
  it("reports true physical fall separately from a motor-controller abort", () => {
    const step = new ExperimentalStepSequence("left",[0.1,0],0);
    step.observe(sample(0));
    step.observe(sample(0.2,{pelvisHeightM:0.25}));
    expect(step.snapshot().phase).toBe("FALL");
  });
  it("rejects impossible displacements and invalid measurements", () => {
    expect(() => new ExperimentalStepSequence("left",[1,0])).toThrow(RangeError);
    const step = new ExperimentalStepSequence("left",[0.1,0]);
    expect(() => step.observe(sample(NaN))).toThrow(RangeError);
    expect(() => step.command(NaN,[0,0])).toThrow(RangeError);
  });
});
