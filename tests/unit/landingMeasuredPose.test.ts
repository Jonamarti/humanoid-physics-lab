import { describe,expect,it } from "vitest";
import { blendMeasuredLandingPose, LANDING_POSE_JOINTS } from "../../src/control/landingMeasuredPose";

describe("Joint posture retained only after verified landing (pure targets)",()=>{
  const nominal={left_hip_pitch:0,left_hip_roll:0,left_hip_yaw:0,
    left_knee:0,left_ankle_pitch:0,left_ankle_roll:0,
    right_hip_pitch:-0.15,lumbar_pitch:0.03};
  const measured=Object.fromEntries(
    LANDING_POSE_JOINTS.map((part,i)=>["left_"+part,-0.15*(i+1)]));
  it("matches actual physical angles at t=0 without acting on support or root",()=>{
    const t=blendMeasuredLandingPose(nominal,measured,"left",0,1);
    for(const name of Object.keys(measured))expect(t[name]).toBe(measured[name]);
    expect(t.right_hip_pitch).toBe(-0.15);
    expect(t.lumbar_pitch).toBe(0.03);
    expect(Object.keys(t).some(k=>k.includes("root"))).toBe(false);
  });
  it("fades continuously back to the existing controller and leaves defaults unchanged",()=>{
    expect(blendMeasuredLandingPose(nominal,measured,"left",0,0)).toEqual(nominal);
    expect(blendMeasuredLandingPose(nominal,measured,"left",2,1)).toEqual(nominal);
    const half=blendMeasuredLandingPose(nominal,measured,"left",0.725,1);
    expect(half.left_knee).toBeCloseTo(measured.left_knee!/2,9);
    expect(half.right_hip_pitch).toBe(nominal.right_hip_pitch);
  });
  it("refuses invalid gains, time or measurements",()=>{
    expect(()=>blendMeasuredLandingPose(nominal,measured,"left",0,1.1)).toThrow(RangeError);
    expect(()=>blendMeasuredLandingPose(nominal,measured,"left",-1,0.2)).toThrow(RangeError);
    expect(()=>blendMeasuredLandingPose(nominal,{}, "left",0,1)).toThrow(RangeError);
  });
});
