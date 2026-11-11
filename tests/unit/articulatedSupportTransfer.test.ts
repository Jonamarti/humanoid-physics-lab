import { describe, expect, it } from "vitest";
import { ExperimentalStepSequence, type StepFeedback } from "../../src/control/experimentalStep";

const feedback=(timeS:number, overrides:Partial<StepFeedback>={}):StepFeedback=>({
  timeS,comWorldM:[0,0,0.95],comVelocityWorldMps:[0,0,0],
  comProjectionInsideLoadedSupport:true,capturePointInsideLoadedSupport:true,
  pelvisHeightM:0.99,rootTiltDeg:0.2,
  swingSoleWorldM:[0.065,0.105,0],supportSoleWorldM:[0.065,-0.105,0],
  swingLoadN:370,supportLoadN:370,...overrides,
});
const gait=(guarded:boolean)=>new ExperimentalStepSequence(
  "left",[0.12,0],0.8,1,0.48,guarded);

describe("V3 solved support-load/capture gate, opt-in only",()=>{
  it("legacy v2 still releases lift at 70% without capture gating",()=>{
    const v2=gait(false);
    v2.observe(feedback(0.8));
    v2.observe(feedback(1.08,{supportLoadN:540,swingLoadN:190,
      capturePointInsideLoadedSupport:false}));
    expect(v2.snapshot().phase).toBe("LIFT");
  });
  it("does not release v3 on one high load spike or with capture point outside",()=>{
    const v3=gait(true);
    v3.observe(feedback(0.8));
    v3.observe(feedback(1.08,{supportLoadN:665,swingLoadN:65,
      capturePointInsideLoadedSupport:false}));
    expect(v3.snapshot().phase).toBe("SHIFT_WEIGHT");
    v3.observe(feedback(1.10,{supportLoadN:665,swingLoadN:65}));
    expect(v3.snapshot().phase).toBe("SHIFT_WEIGHT");
    v3.observe(feedback(1.12,{supportLoadN:530,swingLoadN:200}));
    v3.observe(feedback(1.14,{supportLoadN:665,swingLoadN:65}));
    expect(v3.snapshot().phase).toBe("SHIFT_WEIGHT");
    for(let t=1.16;t<=1.30+1e-8;t+=0.02)
      v3.observe(feedback(+t.toFixed(3),{supportLoadN:665,swingLoadN:65}));
    expect(v3.snapshot().phase).toBe("LIFT");
  });
  it("drops its continuity timer when samples are missing or COM moving",()=>{
    const v3=gait(true);
    v3.observe(feedback(0.8));
    v3.observe(feedback(1.02,{supportLoadN:650,swingLoadN:80}));
    v3.observe(feedback(1.04,{supportLoadN:650,swingLoadN:80}));
    v3.observe(feedback(1.12,{supportLoadN:650,swingLoadN:80}));
    v3.observe(feedback(1.14,{supportLoadN:650,swingLoadN:80,
      comVelocityWorldMps:[0.3,0,0]}));
    v3.observe(feedback(1.16,{supportLoadN:650,swingLoadN:80}));
    expect(v3.snapshot().phase).toBe("SHIFT_WEIGHT");
    v3.observe(feedback(1.30,{supportLoadN:650,swingLoadN:80}));
    expect(v3.snapshot().phase).toBe("SHIFT_WEIGHT");
  });
  it("aborts safely if no stable transfer without inventing lift",()=>{
    const v3=gait(true);
    v3.observe(feedback(0.8));
    for(let t=0.82;t<2.08;t+=0.02)
      v3.observe(feedback(+t.toFixed(3),{supportLoadN:520,swingLoadN:210}));
    expect(v3.snapshot().phase).toBe("ABORT");
    expect(v3.snapshot().stepContactConfirmed).toBe(false);
  });
  it("bounds v3 lateral COM target while preserving legacy command",()=>{
    const v3=gait(true),v2=gait(false);
    v3.observe(feedback(0.8));v2.observe(feedback(0.8));
    for(const step of [v3,v2])
      step.observe(feedback(1.0,{supportLoadN:540,swingLoadN:190}));
    const a=v3.command(1.0,[0,0]);
    const b=v2.command(1.0,[0,0]);
    expect(a.referenceComXY[1]).toBeLessThan(0);
    expect(Math.abs(a.referenceComXY[1])).toBeLessThan(0.075);
    expect(b.referenceComXY[1]).toBeCloseTo(-0.075*0.2/0.35,8);
    expect(a.targetsRad.pelvis_root).toBeUndefined();
    expect(a.referenceComXY[0]).toBeCloseTo(0,10);
    expect(a.targetsRad.lumbar_pitch).toBe(0);
    expect(a.targetsRad.thorax_pitch).toBe(0);
    expect(b.referenceComXY[0]).toBeGreaterThan(0);
    expect(b.targetsRad.lumbar_pitch).toBeGreaterThan(0);
  });
});
