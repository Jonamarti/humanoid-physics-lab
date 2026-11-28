import { describe,expect,it } from "vitest";
import { virtualForce,proximalMoment,loadedHipReaction,measuredShares,weightedCenter } from "../../src/control/classical/genbiconCore";

describe("GENBICON-inspired support-chain primitives",()=>{
  it("uses the source horizontal force limits with HPL world axes",()=>{
    expect(virtualForce(75,[0,0,1],[0,0,0],[1,1])).toEqual([60,100,0]);
    expect(virtualForce(75,[0,0,1],[0,.1,0],[0,0])[1]).toBeCloseTo(-67.5);
    expect(virtualForce(75,[0,0,1],[0,0,0],[0,-1])[1]).toBe(-100);
  });
  it("matches numerical virtual work for a fixed distal link and positive child hinge",()=>{
    const h=1e-6;
    // Inverse rotation of a point 1 m above the planted ankle about +X.
    // The upper body's y displacement is +sin(q).
    const force=37;
    const derivative=force*(Math.sin(h)-Math.sin(-h))/(2*h);
    expect(proximalMoment([1,0,0],[0,0,0],[0,0,1],[0,force,0])).toBeCloseTo(derivative,8);
    expect(proximalMoment([0,1,0],[0,0,0],[0,0,1],[force,0,0])).toBeCloseTo(-derivative,8);
  });
  it("allocates only the residual torso reaction by measured foot load",()=>{
    expect(measuredShares([750,250])).toEqual([.75,.25]);
    expect(measuredShares([0,0])).toEqual([0,0]);
    const a=loadedHipReaction([10,-10],40,[.75,.25],100);
    expect(a).toEqual({left:-20,right:-20,residualNm:0});
    const b=loadedHipReaction([10,-10],40,[1,0],100);
    expect(b).toEqual({left:-30,right:-10,residualNm:0});
    expect(loadedHipReaction([0,0],250,[.5,.5],100)).toEqual({left:-100,right:-100,residualNm:50});
  });
  it("uses actual mass-weighted proximal center rather than geometric midpoint",()=>{
    expect(weightedCenter([{mass:3,position:[0,0,1]},{mass:1,position:[0,0,0]}])).toEqual([0,0,.75]);
  });
});
