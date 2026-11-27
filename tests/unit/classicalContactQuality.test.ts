import { describe,it,expect } from "vitest";
import { LandingQualityAudit,materialPointTravel,type SoleTransform } from "../../src/control/classical/contactQuality";

describe("Independent physical landing audit",()=>{
  it("requires continuous supported flight and sustained landing load",()=>{
    const a=new LandingQualityAudit(.005);
    for(let i=1;i<=5;i++)a.observe(i*.005,[0,300],[.05,0],0,false);
    a.touchdown(.025,0,true);
    // Geometry alone is not enough.
    for(let i=1;i<=3;i++)a.observe(.025+i*.005,[200,300],[0,0],1,false);
    a.observe(.045,[0,300],[0,0],1,false);
    for(let i=1;i<=8;i++)a.observe(.045+i*.005,[200,300],[0,0],1,false);
    expect(a.accepted).toBe(1);
    expect(a.longestConsecutive).toBe(1);
    expect(a.events[0]!.loadedS).toBeCloseTo(.04);
  });
  it("rejects unloaded landings and airborne feet without a supporting partner",()=>{
    const a=new LandingQualityAudit(.005);
    for(let i=1;i<=5;i++)a.observe(i*.005,[0,0],[.05,0],0,false);
    a.touchdown(.025,0,true);
    expect(a.events[0]!.accepted).toBe(false);
    for(let i=1;i<=5;i++)a.observe(.025+i*.005,[300,0],[0,.05],1,false);
    a.touchdown(.05,1,true);
    for(let i=1;i<=32;i++)a.observe(.05+i*.005,[300,0],[0,0],0,false);
    expect(a.accepted).toBe(0);
    expect(a.events[1]!.accepted).toBe(false);
  });
  it("does not count a pending final landing or a landing after falling",()=>{
    const a=new LandingQualityAudit(.005);
    for(let i=1;i<=5;i++)a.observe(i*.005,[0,300],[.05,0],0,false);
    a.touchdown(.025,0,true);a.finish();
    expect(a.accepted).toBe(0);
    expect(a.events[0]!.accepted).toBe(false);
  });
});
describe("Material contact travel",()=>{
  const original:SoleTransform={position:[0,0,1],rotation:[1,0,0,0,1,0,0,0,1]};
  it("measures a translated sole's material point",()=>{
    const translated:SoleTransform={...original,position:[.02,0,1]};
    expect(materialPointTravel(original,translated,[.02,0,0])).toBeCloseTo(.02,12);
  });
  it("does not mistake rotation around a fixed material point for slip",()=>{
    const rotated:SoleTransform={position:[1,0,0],rotation:[0,0,1,0,1,0,-1,0,0]};
    expect(materialPointTravel(original,rotated,[0,0,0])).toBeCloseTo(0,12);
  });
});
