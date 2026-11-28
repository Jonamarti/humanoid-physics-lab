import { describe,it,expect } from "vitest";
import { coronalCommand,lateralCapturePoint,swingHipTarget,hipLimitNm,ankleLimitNm,type CoronalObservation } from "../../src/control/classical/coronalControl";
const base:CoronalObservation={swing:1,phaseElapsedS:.1,lifting:true,comY:0,comZ:1,velocityY:0,torsoRoll:0,torsoRollRate:0,
  hipQ:[0,0],hipRate:[0,0],ankleQ:[0,0],ankleRate:[0,0],
  ankleY:[.1,-.1],hipToAnkleJacobianY:[.8,.8],footRoll:[0,0],soleY:[.1,-.1],
  loadN:[600,0],filteredLoadN:[600,0]};
describe("Coronal reference controller",()=>{
  it("uses LIPM capture position and the actual sign of the swing Jacobian",()=>{
    expect(lateralCapturePoint(0,.3,1)).toBeCloseTo(.3/Math.sqrt(9.81));
    expect(swingHipTarget(0,0,.1,.8)).toBeCloseTo(.125);
    expect(swingHipTarget(0,0,.1,-.8)).toBeCloseTo(-.125);
    expect(swingHipTarget(0,0,10,.8)).toBeCloseTo(.25);
  });
  it("keeps stance hip reaction internal and ankle CoP moment bounded",()=>{
    const r=coronalCommand("capture",{...base,torsoRoll:.1,velocityY:.2});
    expect(r.hipNm[0]+r.hipNm[1]).toBeCloseTo(18);
    expect(r.hipResidualNm).toBeCloseTo(0);
    expect(Math.abs(r.ankleNm[0])).toBeLessThanOrEqual(600*.035);
    expect(r.hipNm.every(v=>Math.abs(v)<=hipLimitNm)).toBe(true);
    expect(r.ankleNm.every(v=>Math.abs(v)<=ankleLimitNm)).toBe(true);
  });
  it("mirrors the lateral controller and changes no sign convention between feet",()=>{
    const a=coronalCommand("capture",base);
    const b=coronalCommand("capture",{...base,swing:0,ankleY:[.1,-.1],soleY:[.1,-.1],loadN:[0,600],filteredLoadN:[0,600]});
    expect(a.targetFootY).toBeCloseTo(-b.targetFootY);
    expect(a.hipNm[0]).toBeCloseTo(-b.hipNm[1]);
    expect(a.hipNm[1]).toBeCloseTo(-b.hipNm[0]);
    expect(a.ankleNm[0]).toBeCloseTo(-b.ankleNm[1]);
  });
  it("does not apply contact-dependent moments when both feet are unloaded",()=>{
    const r=coronalCommand("capture",{...base,loadN:[0,0],torsoRoll:1,hipRate:[100,-100],footRoll:[1,-1]});
    expect(r.hipResidualNm).toBe(0);
    expect(r.hipNm.every(v=>Math.abs(v)<=hipLimitNm)).toBe(true);
    expect(r.ankleNm.every(v=>Math.abs(v)<=ankleLimitNm)).toBe(true);
  });
  it("posture ablation ignores capture point for the swing target",()=>{
    const r=coronalCommand("posture",{...base,velocityY:2});
    expect(r.targetHipQ).toBe(0);
    expect(r.hipNm).toEqual([0,0]);
  });
});
