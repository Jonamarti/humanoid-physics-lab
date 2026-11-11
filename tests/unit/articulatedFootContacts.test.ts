import { describe, expect, it } from "vitest";
import { summarizeGroundSupport, type GroundContactSample } from "../../src/control/footContacts";
import { insideConvexHullXY, type FootGeom } from "../../src/control/balanceSensors";

const R = [1,0,0,0,1,0,0,0,1] as const;
const rear = (y:number):FootGeom => ({
  centerM:[0,y,0.035],rotationWorld:R,halfSizeM:[0.075,0.05,0.035],
});
const toe = (y:number):FootGeom => ({
  centerM:[0.125,y,0.035],rotationWorld:R,halfSizeM:[0.05,0.05,0.035],
});
const contact = (geom1:number,geom2:number,normal:number,
  normalZ:number,position:[number,number,number],
  tangent:[number,number,number]=[0,0,0]):GroundContactSample => ({
  geom1,geom2,contactNormalForceN:normal,
  contactNormalWorld:[0,0,normalZ],
  positionWorldM:position,tangentForceWorldN:tangent,
  frictionUtilization:0.25,
});
const base={
  floorGeomId:0,leftFootGeomId:1,rightFootGeomId:2,
  leftToeGeomId:3,rightToeGeomId:4,
  leftFoot:rear(0.10),rightFoot:rear(-0.10),
  leftToe:toe(0.10),rightToe:toe(-0.10),
};

describe("Independent rearfoot/forefoot contact aggregation",()=>{
  it("sums solver reactions even when geom order and contact count differ",()=>{
    const result=summarizeGroundSupport({...base,contacts:[
      contact(0,3,200,1,[0.14,0.10,0],[20,-10,0]),
      contact(3,0,150,-1,[0.16,0.10,0],[-5,2,0]),
      contact(0,2,250,1,[0,-0.10,0],[-20,3,0]),
      contact(0,1,0,1,[0,0.10,0]),
      contact(0,99,100,1,[0,0,0]), // unrelated geometry
    ]});
    expect(result.totalVerticalLoadN).toBe(600);
    expect(result.activeFootContacts).toBe(3);
    expect(result.left.verticalLoadN).toBe(350);
    expect(result.left.segments!.forefoot.verticalLoadN).toBe(350);
    expect(result.left.segments!.rearfoot.verticalLoadN).toBe(0);
    expect(result.left.segments!.forefoot.activeContacts).toBe(2);
    expect(result.left.tangentialReactionXYN[0]).toBe(25);
    expect(result.left.tangentialReactionXYN[1]).toBe(-12);
    expect(result.left.segments!.forefoot.verticalLoadCentroidWorldM![0])
      .toBeCloseTo((0.14*200+0.16*150)/350,9);
    expect(result.right.verticalLoadN).toBe(250);
    expect(result.right.segments!.rearfoot.verticalLoadN).toBe(250);
    expect(result.right.segments!.forefoot.verticalLoadN).toBe(0);
    expect(result.right.segments!.forefoot.isLoaded).toBe(false);
    expect(result.left.segments!.rearfoot.isLoaded).toBe(false);
    expect(insideConvexHullXY([0.15,0.10],result.loadedSupportPolygonXY)).toBe(true);
    expect(insideConvexHullXY([-0.06,0.10],result.loadedSupportPolygonXY)).toBe(false);
  });
  it("never upgrades small independent contacts into loaded segments",()=>{
    const value=summarizeGroundSupport({...base,contacts:[
      contact(0,1,6,1,[0,0.10,0]),
      contact(0,3,6,1,[0.15,0.10,0]),
      contact(0,2,100,1,[0,-0.10,0]),
    ]});
    expect(value.left.verticalLoadN).toBe(12);
    expect(value.left.isLoaded).toBe(true);
    expect(value.left.segments!.rearfoot.isLoaded).toBe(false);
    expect(value.left.segments!.forefoot.isLoaded).toBe(false);
    expect(value.loadedSupportPolygonXY.every(([x])=>x<=0.075)).toBe(true);
  });
  it("preserves v1/v2 shape and rejects partial or duplicate v3 mappings",()=>{
    const plain=summarizeGroundSupport({
      floorGeomId:0,leftFootGeomId:1,rightFootGeomId:2,
      leftFoot:rear(0.10),rightFoot:rear(-0.10),
      contacts:[contact(0,1,250,1,[0,0.10,0])],
    });
    expect(plain.left.segments).toBeUndefined();
    expect(plain.left.verticalLoadN).toBe(250);
    expect(()=>summarizeGroundSupport({...base,rightToe:undefined,contacts:[]}))
      .toThrow(RangeError);
    expect(()=>summarizeGroundSupport({...base,leftToeGeomId:2,contacts:[]}))
      .toThrow(RangeError);
  });
});
