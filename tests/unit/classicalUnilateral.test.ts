import { describe,expect,it } from "vitest";
import { initialUnilateral,advanceUnilateral,unilateralTargets,lateralLoadTarget,kneeLoadCorrection,type UnilateralObservation } from "../../src/control/classical/unilateral";

const base:UnilateralObservation={comY:0,velocityY:0,loadN:[375,375],clearanceM:[0,0],supportY:.105,bodyWeightN:750,fallen:false};
describe("C2 planner",()=>{
  it("does not lift before the real weight shift qualifies",()=>{
    let s=initialUnilateral();
    for(let i=0;i<60;i++) s=advanceUnilateral(s,base,"left",true,.01);
    expect(s.phase).toBe("shift");
    for(let i=0;i<450;i++) s=advanceUnilateral(s,base,"left",true,.01);
    expect(s.phase).toBe("timeout");
    expect(s.reachedShift).toBe(false);
  });
  it("qualifies two continuous seconds of real support, then lands and recenters",()=>{
    let s=initialUnilateral();
    const shifted={...base,comY:.105,loadN:[650,100] as const};
    const lifted={...shifted,loadN:[750,0] as const,clearanceM:[0,.06] as const};
    for(let i=0;i<205;i++) s=advanceUnilateral(s,shifted,"left",true,.01);
    expect(s.phase).toBe("lift");
    for(let i=0;i<55;i++) s=advanceUnilateral(s,lifted,"left",true,.01);
    expect(s.phase).toBe("hold");
    for(let i=0;i<100;i++) s=advanceUnilateral(s,lifted,"left",true,.01);
    s=advanceUnilateral(s,shifted,"left",true,.01);
    for(let i=0;i<120;i++) s=advanceUnilateral(s,lifted,"left",true,.01);
    expect(s.phase).toBe("hold");
    for(let i=0;i<90;i++) s=advanceUnilateral(s,lifted,"left",true,.01);
    expect(s.phase).toBe("lower");
    expect(s.reachedHold).toBe(true);
    for(let i=0;i<70;i++) s=advanceUnilateral(s,shifted,"left",true,.01);
    expect(s.phase).toBe("recenter");
    for(let i=0;i<160;i++) s=advanceUnilateral(s,base,"left",true,.01);
    expect(s.phase).toBe("done");
    expect(s.completed).toBe(true);
  });
  it("mirrors the lateral target without changing flexion",()=>{
    const s={...initialUnilateral(),phase:"shift" as const,elapsedS:.75};
    expect(unilateralTargets(s,0,.105).refY).toBeCloseTo(.0525);
    expect(unilateralTargets(s,0,-.105).refY).toBeCloseTo(-.0525);
    expect(unilateralTargets(s,0,.105).flexion).toBe(0);
  });
});

describe("C2b lateral load allocation",()=>{
  it("allocates nonnegative fractions with the requested centroid and mirrors correctly",()=>{
    const a=lateralLoadTarget(.0525,[.105,-.105],[true,true]);
    expect(a[0]).toBeCloseTo(.75);
    expect(a[1]).toBeCloseTo(.25);
    expect(lateralLoadTarget(-.0525,[.105,-.105],[true,true])[1]).toBeCloseTo(.75);
    expect(lateralLoadTarget(.3,[.105,-.105],[true,true])).toEqual([1,0]);
    expect(lateralLoadTarget(0,[.105,-.105],[false,true])).toEqual([0,1]);
    expect(lateralLoadTarget(0,[.105,-.105],[false,false])).toEqual([0,0]);
  });
  it("limits the joint load correction and applies opposite reaction",()=>{
    expect(kneeLoadCorrection(500,300,.04)).toBeCloseTo(-8);
    expect(kneeLoadCorrection(300,500,.04)).toBeCloseTo(8);
    expect(kneeLoadCorrection(1000,0,.1)).toBe(-30);
  });
});
