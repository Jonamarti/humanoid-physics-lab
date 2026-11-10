import { describe,expect,it } from "vitest";
import { MaterialContactTracker } from "../../src/physics/materialContactTracker";
import type { MeasuredFootPose } from "../../src/physics/contactPatchSlip";

const identity=[1,0,0,0,1,0,0,0,1] as const;
const pose=(x=0,y=0,z=0.035,
  rotationWorld:readonly number[]=identity):MeasuredFootPose=>({
    centerWorldM:[x,y,z],rotationWorld,
  });
const obs=(tracker:MaterialContactTracker,timeS:number,
  footPose:MeasuredFootPose,verticalLoadN=450,
  contactWorldM:readonly [number,number,number]|null=[0,0,0])=>
  tracker.observe({timeS,footPose,verticalLoadN,contactWorldM});

describe("Stance contact hysteresis, first-impact guard and material-point rolling",()=>{
  it("waits for continuous loaded physics frames, then ramps without anchoring the sole",()=>{
    const tracker=new MaterialContactTracker();
    expect(obs(tracker,0,pose())).toBeNull();
    expect(obs(tracker,0.002,pose(0.002))).toBeNull();
    const engaged=obs(tracker,0.004,pose(0.004))!;
    expect(engaged.consecutiveLoadedSamples).toBe(3);
    expect(engaged.materialSlipXYMps[0]).toBeCloseTo(1,8);
    expect(engaged.feedbackRamp01).toBeCloseTo(0.1,8);
    let result=engaged;
    for(let n=3;n<=12;n++) result=obs(tracker,n*0.002,pose(n*0.002))!;
    expect(result.feedbackRamp01).toBe(1);
    expect(result.materialSlipXYMps[0]).toBeCloseTo(1,6);
    expect(result.tangentialSpeedMps).toBeCloseTo(1,6);
  });
  it("allows heel-to-toe pivot around a fixed contact without misclassifying slipping",()=>{
    const tracker=new MaterialContactTracker();
    const angle=0.16,c=Math.cos(angle),s=Math.sin(angle);
    const pivotRotation=[c,0,s,0,1,0,-s,0,c] as const;
    const first=pose(0,0,1);
    const rotated=pose(s,0,c,pivotRotation);
    expect(obs(tracker,0,first,450,[0,0,0])).toBeNull();
    expect(obs(tracker,0.002,rotated,450,[0,0,0])).toBeNull();
    const stable=obs(tracker,0.004,rotated,450,[0,0,0])!;
    expect(stable.tangentialSpeedMps).toBeLessThan(1e-9);
  });
  it("keeps feedback through load fluctuation, releases below 35 N and primes again",()=>{
    const tracker=new MaterialContactTracker();
    obs(tracker,0,pose(),80);
    obs(tracker,0.002,pose(),80);
    expect(obs(tracker,0.004,pose(),80)).not.toBeNull();
    expect(obs(tracker,0.006,pose(),42)).not.toBeNull();
    expect(obs(tracker,0.008,pose(),34)).toBeNull();
    expect(obs(tracker,0.010,pose(),50)).toBeNull(); // requires >=60N to re-engage
    expect(obs(tracker,0.012,pose(),80)).toBeNull();
    expect(obs(tracker,0.014,pose(),80)).toBeNull();
    expect(obs(tracker,0.016,pose(),80)).not.toBeNull();
  });
  it("rejects stale pose after a gap rather than applying a spurious touchdown impulse",()=>{
    const tracker=new MaterialContactTracker();
    obs(tracker,0,pose(),80);
    obs(tracker,0.002,pose(),80);
    expect(obs(tracker,0.05,pose(0.5),80)).toBeNull();
    expect(obs(tracker,0.052,pose(0.5),80)).toBeNull();
    expect(obs(tracker,0.054,pose(0.5),80)).not.toBeNull();
    expect(obs(tracker,0.056,pose(0.5),0)).toBeNull();
  });
  it("rejects bad time and sensor data",()=>{
    const tracker=new MaterialContactTracker();
    expect(()=>new MaterialContactTracker(20,35)).toThrow(RangeError);
    expect(()=>new MaterialContactTracker(60,35,0)).toThrow(RangeError);
    expect(()=>obs(tracker,0,pose(),-5)).toThrow(RangeError);
    expect(()=>obs(tracker,Number.NaN,pose(),50)).toThrow(RangeError);
    obs(tracker,0,pose());
    expect(()=>obs(tracker,0,pose())).toThrow(RangeError);
  });
});
