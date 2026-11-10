import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { controlledStepPosture, smoothStep } from "../../src/control/controlledStepPosture";

const input=(phase:"SHIFT_WEIGHT"|"LIFT"|"SWING"|"LAND"|"RECOVER",elapsedS:number,dx=0.12)=>({
  phase,elapsedS,swingSide:"left" as const,
  targetDeltaXY:[dx,0] as const,
});
describe("Corrected knee anatomy and controlled step postures",()=>{
  it("makes knee folding biomechanically posterior with the existing negative-q targets",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    for (const side of ["left","right"]) {
      expect(xml).toContain(`<joint name="${side}_knee" type="hinge" axis="0 -1 0" range="-2.45 0"/>`);
    }
    // Rodrigues rotation around -Y at q = -60 deg is positive Y
    // rotation, so a shank pointing down turns backwards (negative X).
    const kneeRad=-Math.PI/3;
    const effectiveRotation=-kneeRad;
    const shinX=-Math.sin(effectiveRotation);
    expect(shinX).toBeLessThan(-0.8);
  });
  it("leans the trunk before lifting and shifts COM reference forward, never root qpos",()=>{
    const shift=controlledStepPosture(input("SHIFT_WEIGHT",0.4));
    expect(shift.forwardComBiasM).toBeGreaterThan(0.03);
    expect(shift.requestedTrunkPitchRad).toBeGreaterThan(0.06);
    expect(shift.targetsRad.lumbar_pitch).toBeGreaterThan(0);
    expect(shift.targetsRad.thorax_pitch).toBeGreaterThan(0);
    expect(Object.keys(shift.targetsRad).some(k=>k.includes("root"))).toBe(false);
    const backward=controlledStepPosture(input("SHIFT_WEIGHT",0.4,-0.12));
    expect(backward.forwardComBiasM).toBeLessThan(-0.03);
    expect(backward.requestedTrunkPitchRad).toBeLessThan(-0.11);
  });
  it("flexes knee strongly to shorten leg while raising thigh, then extends gradually",()=>{
    const lift=controlledStepPosture(input("LIFT",0.3));
    const early=controlledStepPosture(input("SWING",0));
    const late=controlledStepPosture(input("SWING",0.42));
    const landingStart=controlledStepPosture(input("LAND",0));
    const landingEnd=controlledStepPosture(input("LAND",0.45));
    expect(lift.targetsRad.left_knee).toBeCloseTo(-1.0,10);
    expect(lift.targetsRad.left_hip_pitch).toBeLessThan(-0.3);
    expect(lift.targetsRad.left_knee).toBeLessThan(late.targetsRad.left_knee!);
    expect(late.targetsRad.left_knee).toBeLessThan(landingEnd.targetsRad.left_knee!);
    expect(landingStart.targetsRad.left_knee).toBeLessThan(landingEnd.targetsRad.left_knee!);
    expect(early.targetsRad.left_knee).toBeLessThan(-0.7);
    // 60 deg flexion is a reference from adult gait studies, not a guarantee.
    expect(-lift.targetsRad.left_knee!).toBeGreaterThan(0.9);
  });
  it("never creates forward lean for purely lateral step",()=>{
    const side=controlledStepPosture({...input("LIFT",0.3),targetDeltaXY:[0,0.12]});
    expect(side.forwardComBiasM).toBe(0);
    expect(side.targetsRad.lumbar_pitch).toBe(0);
    expect(side.targetsRad.left_knee).toBeLessThan(-0.9);
  });
  it("controls mode intensity, ramp boundaries and rejects invalid input",()=>{
    const none=controlledStepPosture({...input("LIFT",0.3),intensity:0});
    expect(none.targetsRad.left_knee).toBe(0);
    expect(none.forwardComBiasM).toBe(0);
    expect(smoothStep(0,0.2)).toBe(0);
    expect(smoothStep(0.2,0.2)).toBe(1);
    expect(()=>controlledStepPosture({...input("LIFT",0.3),intensity:1.1})).toThrow(RangeError);
    expect(()=>smoothStep(NaN,0.2)).toThrow(RangeError);
  });
});
