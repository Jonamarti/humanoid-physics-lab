import { describe,expect,it } from "vitest";
import { decodeSolvedContactForce } from "../../src/physics/contactForces";

describe("MuJoCo solved contact wrench (matches mju_decodePyramid)",()=>{
  it("recovers normal and both tangents from pyramidal friction dim=3",()=>{
    const result=decodeSolvedContactForce({
      solverRows:new Float64Array([999,30,10,20,5,888]),
      efcAddress:1,dim:3,pyramidal:true,
      friction:new Float64Array([0.8,0.6]),adhesionN:0,
    });
    expect(result.normalN).toBeCloseTo(65);
    expect(result.tangentN[0]).toBeCloseTo(16);
    expect(result.tangentN[1]).toBeCloseTo(9);
    expect(result.frictionUtilization).toBeCloseTo(35/65);
  });
  it("decodes an elliptic cone and non-adhesive frictionless contact",()=>{
    const smooth=decodeSolvedContactForce({
      solverRows:new Float64Array([200,60,-40]),
      efcAddress:0,dim:3,pyramidal:false,
      friction:[1,0.5],adhesionN:0,
    });
    expect(smooth.tangentN).toEqual([60,-40]);
    expect(smooth.frictionUtilization).toBeCloseTo(0.5);
    const one=decodeSolvedContactForce({
      solverRows:[750],efcAddress:0,dim:1,pyramidal:true,
      friction:[],adhesionN:0,
    });
    expect(one.normalN).toBe(750);
    expect(one.frictionUtilization).toBeNull();
  });
  it("rejects unsolved/out-of-bounds constraints and NaN friction",()=>{
    const base={solverRows:[1,2,3,4],efcAddress:0,dim:3,
      pyramidal:true,friction:[1,1],adhesionN:0};
    expect(()=>decodeSolvedContactForce({...base,efcAddress:3})).toThrow(RangeError);
    expect(()=>decodeSolvedContactForce({...base,friction:[NaN,1]})).toThrow(RangeError);
    expect(()=>decodeSolvedContactForce({...base,dim:0})).toThrow(RangeError);
  });
});
