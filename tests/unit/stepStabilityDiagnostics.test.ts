import { describe,expect,it } from "vitest";
import { assessStepStability,STEP_STABILITY_CHECKS,
  type StepStabilityInputs } from "../../src/control/stepStabilityDiagnostics";

const stable:StepStabilityInputs={
  comProjectionInsideLoadedSupport:true,
  capturePointInsideLoadedSupport:true,
  comVelocityWorldMps:[0.01,0.02,0],
  pelvisHeightM:0.99,
  rootTiltDeg:3,
  targetErrorXYM:0.02,
  maxSupportFootSlipM:0.005,
  maxLandingSlipM:0.005,
  supportLoadN:380,
  swingLoadN:370,
};
describe("Explain ALL physical gates required to accept a settled step",()=>{
  it("accepts a fully measured, supported and stationary result",()=>{
    const result=assessStepStability(stable);
    expect(result.passes).toBe(true);
    expect(result.failures).toEqual([]);
    expect(Object.values(result.checks).every(Boolean)).toBe(true);
    expect(result.comSpeedXYMps).toBeCloseTo(Math.hypot(0.01,0.02));
  });
  it("reports every independent failed criterion without relaxing thresholds",()=>{
    const bad:StepStabilityInputs={
      ...stable,
      comProjectionInsideLoadedSupport:false,
      capturePointInsideLoadedSupport:false,
      comVelocityWorldMps:[0.14,0,0],
      pelvisHeightM:0.6,
      rootTiltDeg:15,
      targetErrorXYM:0.07,
      maxSupportFootSlipM:0.03,
      maxLandingSlipM:0.04,
      supportLoadN:35,
      swingLoadN:0,
    };
    const r=assessStepStability(bad);
    expect(r.passes).toBe(false);
    expect(r.failures).toEqual(STEP_STABILITY_CHECKS);
  });
  it("keeps original strict inequality and inclusive distance boundaries",()=>{
    const atLimits=assessStepStability({
      ...stable,
      comVelocityWorldMps:[0.12,0,0],
      pelvisHeightM:0.82,
      rootTiltDeg:10,
      targetErrorXYM:0.05,
      maxSupportFootSlipM:0.02,
      maxLandingSlipM:0.02,
      supportLoadN:40,
      swingLoadN:40,
    });
    expect(atLimits.failures).toEqual([
      "comSpeed","pelvisHeight","trunkTilt","supportLoad","landingLoad",
    ]);
    expect(atLimits.checks.targetError).toBe(true);
    expect(atLimits.checks.supportFootDrift).toBe(true);
    expect(atLimits.checks.landingFootDrift).toBe(true);
  });
  it("rejects non-finite sensor values",()=>{
    expect(()=>assessStepStability({...stable,comVelocityWorldMps:[NaN,0,0]})).toThrow(RangeError);
    expect(()=>assessStepStability({...stable,supportLoadN:-1})).toThrow(RangeError);
    expect(()=>assessStepStability({...stable,targetErrorXYM:Infinity})).toThrow(RangeError);
  });
});
