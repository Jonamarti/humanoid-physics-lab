import { describe, expect, it } from "vitest";
import { estimateContactPatchSlip, materialContactLocalM, type MeasuredFootPose } from "../../src/physics/contactPatchSlip";

const identity=[1,0,0,0,1,0,0,0,1] as const;
const pose=(x:number,y:number,z:number,rot:readonly number[]=identity):MeasuredFootPose=>
  ({centerWorldM:[x,y,z],rotationWorld:rot});

describe("Sliding versus rolling of real foot-ground contact material",()=>{
  it("detects true horizontal translation independently of the selected contact point",()=>{
    const last=pose(0,0,0.035);
    const next=pose(0.02,0,0.035);
    const r=estimateContactPatchSlip(last,next,[0.03,0,0],0.02);
    expect(r.tangentialVelocityWorldMps[0]).toBeCloseTo(1,10);
    expect(r.tangentialVelocityWorldMps[1]).toBeCloseTo(0,10);
    expect(r.estimatedSlipDistanceThisSampleM).toBeCloseTo(0.02,10);
  });

  it("does not mislabel a rigid rotation about stationary contact as slipping",()=>{
    const rad=0.16,cs=Math.cos(rad),sn=Math.sin(rad);
    const rotation=[cs,0,sn, 0,1,0, -sn,0,cs] as const;
    const previous=pose(0,0,1);
    const current=pose(sn,0,cs,rotation); // pivot about world [0,0,0]
    const result=estimateContactPatchSlip(previous,current,[0,0,0],0.02);
    expect(result.tangentialSpeedMps).toBeLessThan(1e-10);
    // A contact material point translating 2 cm still measures motion.
    const shifted=pose(sn+0.02,0,cs,rotation);
    const sliding=estimateContactPatchSlip(previous,shifted,[0.02,0,0],0.02);
    expect(sliding.tangentialSpeedMps).toBeCloseTo(1,10);
  });

  it("allows a contact centroid to move across the foot during rolling",()=>{
    const last=pose(0,0,0.035);
    const next=pose(0,0,0.035);
    const speedA=estimateContactPatchSlip(last,next,[0.04,0,0],0.02);
    const speedB=estimateContactPatchSlip(last,next,[0.08,0,0],0.02);
    expect(speedA.tangentialSpeedMps).toBe(0);
    expect(speedB.tangentialSpeedMps).toBe(0);
  });

  it("tracks heel-to-toe contact migration in the foot frame separately from slip",()=>{
    const stationary=pose(0,0,0.035);
    const heel=materialContactLocalM(stationary,[-0.07,0,0]);
    const toe=materialContactLocalM(stationary,[0.12,0,0]);
    expect(heel[0]).toBeLessThan(0);
    expect(toe[0]).toBeGreaterThan(0);
    expect(toe[0]-heel[0]).toBeCloseTo(0.19);
    // Different point of contact can move along a rigid sole without
    // implying material motion of the point currently supporting weight.
    expect(estimateContactPatchSlip(stationary,stationary,[-0.07,0,0],0.002)
      .tangentialSpeedMps).toBe(0);
    expect(estimateContactPatchSlip(stationary,stationary,[0.12,0,0],0.002)
      .tangentialSpeedMps).toBe(0);
  });

  it("rejects corrupt coordinates or time resolution",()=>{
    const p=pose(0,0,1);
    expect(()=>estimateContactPatchSlip(p,p,[0,0,0],0)).toThrow(RangeError);
    expect(()=>estimateContactPatchSlip(p,p,[NaN,0,0],0.02)).toThrow(RangeError);
    expect(()=>estimateContactPatchSlip(p,{...p,rotationWorld:[1,2]},[0,0,0],0.02)).toThrow(RangeError);
  });
});
