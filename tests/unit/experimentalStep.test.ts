import { describe, expect, it } from "vitest";
import { ExperimentalStepSequence, applyStanceToeRollAdditiveTorque, type StepFeedback } from "../../src/control/experimentalStep";

const sample = (timeS: number, overrides: Partial<StepFeedback> = {}): StepFeedback => ({
  timeS, comWorldM:[0,0,0.95],comVelocityWorldMps:[0,0,0],
  comProjectionInsideLoadedSupport:true,
  capturePointInsideLoadedSupport:true,
  pelvisHeightM:0.99,rootTiltDeg:2,
  swingSoleWorldM:[0.065,0.105,0.001],
  supportSoleWorldM:[0.065,-0.105,0.001],
  supportLoadN:370,swingLoadN:370,
  ...overrides,
});

describe("Stance heel rise preserves the existing COM/ankle torque",()=>{
  it("adds bounded torque without replacing balance control or acting on root",()=>{
    const model={
      njnt:2,jnt_type:[0,3],jnt_dofadr:[0,6],jnt_qposadr:[0,7],
      jnt:(name:string)=>({id:name==="right_ankle_pitch"?1:-1}),
    } as unknown as Parameters<typeof applyStanceToeRollAdditiveTorque>[0];
    const applied=new Float64Array(7);
    const data={qfrc_applied:applied,qpos:new Float64Array(8),
      qvel:new Float64Array(7)};
    const gains=new Map([[1,{targetRad:0,stiffnessNmPerRad:350,
      dampingNmsPerRad:32,maxTorqueNm:170}]]);
    applied[6]=42;
    expect(applyStanceToeRollAdditiveTorque(model,data,gains,"right",0.06))
      .toBeCloseTo(21);
    expect(applied[6]).toBeCloseTo(63);
    expect(Array.from(applied.subarray(0,6))).toEqual([0,0,0,0,0,0]);
    applied[6]=165;
    expect(applyStanceToeRollAdditiveTorque(model,data,gains,"right",0.12))
      .toBeCloseTo(5);
    expect(applied[6]).toBe(170);
    expect(()=>applyStanceToeRollAdditiveTorque(model,data,gains,"right",-0.1))
      .toThrow(RangeError);
  });
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
    for (let i=0;i<57;i++) {
      step.observe(sample(1.58+i*0.02,{
        supportLoadN:410,swingLoadN:330,swingSoleWorldM:[0.17,0.19,0.001],
      }));
      if (i<49) expect(step.snapshot().phase).toBe("RECOVER");
    }
    expect(step.snapshot().phase).toBe("COMPLETE");
    expect(step.snapshot().settledDurationS).toBeGreaterThanOrEqual(1);
    expect(step.snapshot().stepContactConfirmed).toBe(true);
    // After a forward/side step, the new support centroid is no longer
    // the original bilateral midpoint. Never drag the COM back to it.
    const after = step.command(2.70,[0,0]);
    expect(after.referenceComXY[0]).toBeGreaterThan(0.04);
    expect(after.referenceComXY[1]).toBeGreaterThan(0.03);
    expect(step.snapshot().maxSupportFootSlipM).toBe(0);
    expect(step.snapshot().targetErrorXYM).toBeLessThan(0.03);
  });
  it("detects contact after forward knee extension even if toe height is already low", () => {
    const step=new ExperimentalStepSequence("left",[0.12,0],0.8,1);
    step.observe(sample(0.1));
    step.observe(sample(0.8));
    step.observe(sample(1.1,{supportLoadN:640,swingLoadN:90}));
    expect(step.snapshot().phase).toBe("LIFT");
    step.observe(sample(1.2,{supportLoadN:720,swingLoadN:0,
      swingSoleWorldM:[0.065,0.105,0.055]}));
    expect(step.snapshot().phase).toBe("SWING");
    step.observe(sample(1.44,{supportLoadN:600,swingLoadN:130,
      swingSoleWorldM:[0.16,0.105,0.009]}));
    expect(step.snapshot().phase).toBe("LAND");
    step.observe(sample(1.52,{supportLoadN:530,swingLoadN:180,
      swingSoleWorldM:[0.16,0.105,0.004]}));
    expect(step.snapshot().phase).toBe("RECOVER");
    expect(step.snapshot().stepContactConfirmed).toBe(true);
  });

  it("fades the swing servo on physical landing, never pushes the foot after contact", () => {
    const step=new ExperimentalStepSequence("left",[0.1,0],0.8);
    step.observe(sample(0.05));
    step.observe(sample(0.8));
    step.observe(sample(1.1,{supportLoadN:610,swingLoadN:110}));
    expect(step.snapshot().phase).toBe("LIFT");
    step.observe(sample(1.18,{supportLoadN:735,swingLoadN:0,
      swingSoleWorldM:[0.065,0.105,0.05]}));
    expect(step.snapshot().phase).toBe("SWING");
    const airborne=step.command(1.2,[0,0]);
    expect(airborne.cartesianBlend).toBeCloseTo(1);
    step.observe(sample(1.3,{supportLoadN:710,swingLoadN:0,
      swingSoleWorldM:[0.17,0.105,0.07],
      swingSoleVelocityWorldMps:[0.38,0.16,-0.24]}));
    expect(step.snapshot().phase).toBe("LAND");
    const beforeLoad=step.command(1.31,[0,0]);
    expect(beforeLoad.cartesianBlend).toBeCloseTo(1);
    step.observe(sample(1.35,{supportLoadN:690,swingLoadN:30,
      swingSoleWorldM:[0.17,0.105,0.03]}));
    const partial=step.command(1.36,[0,0]);
    expect(partial.cartesianBlend).toBeCloseTo(0.5);
    step.observe(sample(1.40,{supportLoadN:560,swingLoadN:160,
      swingSoleWorldM:[0.17,0.105,0.002],
      swingFrictionUtilization:0.91}));
    expect(step.snapshot().phase).toBe("RECOVER");
    const telemetry=step.snapshot();
    expect(telemetry.preLandingTangentialSpeedMps).toBeCloseTo(Math.hypot(0.38,0.16),8);
    expect(telemetry.preLandingDownwardSpeedMps).toBeCloseTo(0.24,8);
    expect(telemetry.frictionUtilizationAtLanding).toBeCloseTo(0.91,8);
    expect(telemetry.firstLandingTimeS).toBeCloseTo(1.40,8);
    expect(step.command(1.42,[0,0]).cartesianBlend).toBe(0);
    expect(step.command(1.42,[0,0]).soleTargetWorldM).toBeNull();
  });

  it("requires consecutive measurements instead of treating a short contact as success", () => {
    const step=new ExperimentalStepSequence("right",[0.1,-0.08],0.5);
    step.observe(sample(0.02));
    step.observe(sample(0.5));
    step.observe(sample(0.9,{supportLoadN:620,swingLoadN:100}));
    step.observe(sample(1.0,{supportLoadN:740,swingLoadN:0,
      swingSoleWorldM:[0.065,0.105,0.06]}));
    step.observe(sample(1.2,{supportLoadN:740,swingLoadN:0,
      swingSoleWorldM:[0.168,0.023,0.06]}));
    expect(step.snapshot().phase).toBe("LAND");
    step.observe(sample(1.3,{supportLoadN:570,swingLoadN:160,
      swingSoleWorldM:[0.168,0.023,0.003]}));
    expect(step.snapshot().phase).toBe("RECOVER");
    // 500 ms of valid support are insufficient.
    for (let i=0;i<25;i++) step.observe(sample(1.32+i*0.02,{
      supportLoadN:380,swingLoadN:355,
      swingSoleWorldM:[0.168,0.023,0.001]}));
    expect(step.snapshot().phase).toBe("RECOVER");
    // Even one short loss of capture-point support resets the clock.
    step.observe(sample(1.82,{supportLoadN:380,swingLoadN:355,
      swingSoleWorldM:[0.168,0.023,0.001],
      capturePointInsideLoadedSupport:false}));
    expect(step.snapshot().settledDurationS).toBe(0);
    for(let i=0;i<22;i++) step.observe(sample(1.84+i*0.02,{
      supportLoadN:380,swingLoadN:355,
      swingSoleWorldM:[0.168,0.023,0.001]}));
    expect(step.snapshot().phase).toBe("RECOVER");
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
