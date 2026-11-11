import {describe,expect,it} from "vitest";
import {convexHullXY, type XY} from "../../src/control/balanceSensors";
import type {GroundSupportObservation,FootSupport} from "../../src/control/footContacts";
import {ReactiveStepCoordinator} from "../../src/control/reactiveStepCoordinator";

const rectangle=(y:number):XY[]=>[
  [-0.115,y-0.055],[0.17,y-0.055],[0.17,y+0.055],[-0.115,y+0.055],
];
const foot=(y:number):FootSupport=>({
  centerXY:[0,y],footprintXY:rectangle(y),verticalLoadN:368,
  tangentialReactionXYN:[0,0],frictionUtilizationMean:0,
  frictionUtilizationPeak:0,activeContacts:2,verticalLoadCentroidXY:[0,y],
  verticalLoadCentroidWorldM:[0,y,0],isLoaded:true,
});
function ground():GroundSupportObservation {
  const left=foot(.11),right=foot(-.11);
  return {left,right,
    totalVerticalLoadN:736,activeFootContacts:4,
    loadedSupportPolygonXY:convexHullXY([...left.footprintXY,...right.footprintXY]),
  };
}
const sample=(timeS:number,v:XY=[.62,0],extra:Record<string,unknown>={})=>({
  timeS,comWorldM:[0,0,.99] as const,
  comVelocityWorldMps:[v[0],v[1],0] as const,
  totalMassKg:75,rootTiltDeg:1,pelvisHeightM:.99,
  ground:ground(),...extra,
});
const runUntilIntent=(v:XY,dt=.02)=>{
  const seq=new ReactiveStepCoordinator();
  let intent=null;
  for(let i=0;i<20;i++){
    intent=seq.observe(sample(+(i*dt).toFixed(3),v));
    if(intent)break;
  }
  return {seq,intent};
};
describe("Reactive capture-point recovery step planner",()=>{
  it("does not move a foot during standing or short-lived capture-point noise",()=>{
    const c=new ReactiveStepCoordinator();
    for(let i=0;i<70;i++)expect(c.observe(sample(i*.02,[0,0]))).toBeNull();
    expect(c.snapshot().attempts).toBe(0);
    expect(c.observe(sample(1.42,[.62,0]))).toBeNull();
    expect(c.observe(sample(1.44,[.62,0]))).toBeNull();
    expect(c.observe(sample(1.46,[0,0]))).toBeNull();
    expect(c.snapshot().phase).toBe("MONITORING");
    expect(c.snapshot().attempts).toBe(0);
  });
  it.each([
    ["forward",[.62,0] as XY,0,1],
    ["backward",[-.62,0] as XY,0,-1],
    ["left",[0,.78] as XY,1,1],
    ["right",[0,-.78] as XY,1,-1],
  ])("requests a reachable physical step toward %s COM motion",(_name,v,axis,sign)=>{
    const {seq,intent}=runUntilIntent(v);
    expect(intent).not.toBeNull();
    expect(seq.snapshot().attempts).toBe(1);
    expect(seq.snapshot().phase).toBe("STEP_ATTEMPT");
    expect(intent!.deltaXY[axis]*sign).toBeGreaterThan(0);
    expect(Math.hypot(...intent!.deltaXY)).toBeLessThanOrEqual(.38);
    expect(intent!.capturePointXY[axis]*sign).toBeGreaterThan(0);
    expect(intent!.supportSide).not.toBe(intent!.swingSide);
    // No duplicate re-plans even while the capture point keeps escaping.
    for(let i=0;i<20;i++) expect(seq.observe(sample(1+i*.02,v))).toBeNull();
    expect(seq.snapshot().attempts).toBe(1);
  });
  it("requires solved support and has a real FALL outcome",()=>{
    const seq=new ReactiveStepCoordinator();
    const unloaded={...ground(),totalVerticalLoadN:0,
      left:{...foot(.11),verticalLoadN:0,isLoaded:false},
      right:{...foot(-.11),verticalLoadN:0,isLoaded:false},
      loadedSupportPolygonXY:[]};
    for(let i=0;i<20;i++)expect(seq.observe(sample(i*.02,[.62,0],{
      ground:unloaded}))).toBeNull();
    expect(seq.snapshot().attempts).toBe(0);
    expect(seq.observe(sample(.42,[.62,0],{pelvisHeightM:.6}))).toBeNull();
    expect(seq.snapshot().phase).toBe("FALL");
  });
  it("records ABORT and later FALL, without treating either as a successful step",()=>{
    const {seq,intent}=runUntilIntent([.62,0]);
    expect(intent).not.toBeNull();
    seq.observePhysicalStep("LIFT","start");
    expect(seq.snapshot().physicalStepPhase).toBe("LIFT");
    seq.observePhysicalStep("ABORT","no loaded landing");
    expect(seq.snapshot().phase).toBe("STEP_ABORT");
    expect(seq.snapshot().attempts).toBe(1);
    expect(seq.observe(sample(2,[.62,0],{pelvisHeightM:.48}))).toBeNull();
    expect(seq.snapshot().phase).toBe("FALL");
  });
  it("rejects time regression and invalid measured COM",()=>{
    const seq=new ReactiveStepCoordinator();
    seq.observe(sample(1,[0,0]));
    expect(()=>seq.observe(sample(.9,[0,0]))).toThrow(RangeError);
    expect(()=>seq.observe(sample(1.1,[NaN,0]))).toThrow(RangeError);
  });
});
