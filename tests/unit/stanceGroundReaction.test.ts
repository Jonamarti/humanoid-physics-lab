import { describe,expect,it } from "vitest";
import { summarizeGroundSupport } from "../../src/control/footContacts";
import type { FootGeom } from "../../src/control/balanceSensors";

const rotation=[1,0,0,0,1,0,0,0,1] as const;
const makeFoot=(y:number):FootGeom=>({
  centerM:[0.065,y,0.035],
  rotationWorld:rotation,
  halfSizeM:[0.135,0.055,0.035],
});

describe("Actual solved foot-ground friction signs and capacity",()=>{
  it("measures the reaction on the foot even if contact geom order flips",()=>{
    const ground=summarizeGroundSupport({
      floorGeomId:0,leftFootGeomId:1,rightFootGeomId:2,
      leftFoot:makeFoot(0.105),rightFoot:makeFoot(-0.105),
      contacts:[
        {geom1:0,geom2:1,positionWorldM:[0.03,0.105,0],
          contactNormalForceN:200,contactNormalWorld:[0,0,1],
          tangentForceWorldN:[20,0,0],contactSlideFrictionMin:0.9,
          frictionUtilization:0.15},
        {geom1:1,geom2:0,positionWorldM:[0.10,0.105,0],
          contactNormalForceN:120,contactNormalWorld:[0,0,-1],
          tangentForceWorldN:[2,0,0],contactSlideFrictionMin:0.7,
          frictionUtilization:0.25},
      ],
    });
    expect(ground.left.verticalLoadN).toBe(320);
    expect(ground.left.tangentialReactionXYN[0]).toBe(18);
    expect(ground.left.effectiveSlideFrictionMin).toBeCloseTo(0.7);
    expect(ground.left.frictionUtilizationPeak).toBeCloseTo(0.25);
    expect(ground.right.effectiveSlideFrictionMin).toBeNull();
    expect(ground.left.verticalLoadCentroidWorldM![0]).toBeCloseTo(
      (0.03*200+0.10*120)/320);
  });
  it("rejects corrupted contact friction instead of trusting it",()=>{
    expect(()=>summarizeGroundSupport({
      floorGeomId:0,leftFootGeomId:1,rightFootGeomId:2,
      leftFoot:makeFoot(0.105),rightFoot:makeFoot(-0.105),
      contacts:[{geom1:0,geom2:1,positionWorldM:[0,0,0],
        contactNormalForceN:100,contactNormalWorld:[0,0,1],
        contactSlideFrictionMin:-0.1}],
    })).toThrow(RangeError);
  });
});
