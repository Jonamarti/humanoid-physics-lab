import { describe, expect, it } from "vitest";
import { applyStanceContactCommand, planStanceContactCommand } from "../../src/control/stanceContact";
import type { FootSupport } from "../../src/control/footContacts";

const foot: FootSupport = {
  centerXY:[0,0],
  footprintXY:[[0,0],[0.1,0],[0.1,0.08],[0,0.08]],
  verticalLoadN:450,
  tangentialReactionXYN:[32,0],
  frictionUtilizationMean:0.1,
  frictionUtilizationPeak:0.1,
  effectiveSlideFrictionMin:0.9,
  activeContacts:2,
  verticalLoadCentroidXY:[0,0],
  verticalLoadCentroidWorldM:[0,0,0],
  isLoaded:true,
};
const sample=(patch:Partial<Parameters<typeof planStanceContactCommand>[0]>={})=>
  planStanceContactCommand({
    phase:"SWING",requestedStepXY:[0.12,0],comWorldM:[0.10,0,0.94],
    contactWorldM:[0,0,0],foot,materialSlipXYMps:[0,0],strength:1,
    ...patch,
  });

describe("Experimental ground-contact feedback, not a welded foot",()=>{
  it("preserves old controller and refuses to actuate airborne or frictionless feet",()=>{
    expect(sample({strength:0})).toBeNull();
    expect(sample({phase:"STAND"})).toBeNull();
    expect(sample({foot:{...foot,verticalLoadN:0}})).toBeNull();
    expect(sample({foot:{...foot,effectiveSlideFrictionMin:null}})).toBeNull();
    expect(sample({foot:{...foot,frictionUtilizationPeak:0.99}})).toBeNull();
  });
  it("requests backward foot motion during late forward swing but reports real GRF separately",()=>{
    const result=sample()!;
    expect(result.virtualForceWorldN[0]).toBeLessThan(0);
    expect(result.virtualForceWorldN[2]).toBe(0);
    expect(result.intendedPushOffN).toBeCloseTo(12);
    expect(result.reactionForwardN).toBe(32); // measured, not desired
    expect(sample({comWorldM:[-0.04,0,0.94]})?.intendedPushOffN ?? 0).toBe(0);
    expect(sample({requestedStepXY:[0,0.12]})?.intendedPushOffN ?? 0).toBe(0);
  });
  it("opposes the velocity of the material contact, not the sole marker",()=>{
    const forward=sample({materialSlipXYMps:[0.3,0]})!;
    const backward=sample({materialSlipXYMps:[-0.7,0]})!;
    expect(forward.virtualForceWorldN[0]).toBeLessThan(-12);
    expect(backward.virtualForceWorldN[0]).toBeGreaterThan(0);
    const lateral=sample({requestedStepXY:[0,0.12],materialSlipXYMps:[0,0.3]})!;
    expect(lateral.virtualForceWorldN[1]).toBeLessThan(0);
    const recovery=sample({phase:"RECOVER",materialSlipXYMps:[0.2,0]})!;
    expect(recovery.intendedPushOffN).toBe(0);
    expect(recovery.virtualForceWorldN[0]).toBeLessThan(0);
  });
  it("isolates posterior impulse from slip and recovery torques",()=>{
    const push=sample({mode:"push-only",materialSlipXYMps:[0.25,0]})!;
    expect(push.virtualForceWorldN[0]).toBeCloseTo(-12);
    expect(push.intendedPushOffN).toBeCloseTo(12);
    expect(sample({mode:"push-only",phase:"RECOVER",
      materialSlipXYMps:[0.25,0]})).toBeNull();
    const damping=sample({mode:"slip-only",materialSlipXYMps:[0.25,0]})!;
    expect(damping.intendedPushOffN).toBe(0);
    expect(damping.virtualForceWorldN[0]).toBeLessThan(0);
    expect(sample({mode:"slip-only",materialSlipXYMps:[0,0]})).toBeNull();
    const recovery=sample({mode:"recovery-only",phase:"RECOVER",
      materialSlipXYMps:[0.25,0]})!;
    expect(recovery.intendedPushOffN).toBe(0);
    expect(recovery.virtualForceWorldN[0]).toBeLessThan(0);
    expect(sample({mode:"recovery-only",materialSlipXYMps:[0.25,0]})).toBeNull();
  });
  it("gates terminal propulsion behind a forefoot contact and measured swing",()=>{
    expect(sample({mode:"toe-gated",contactLocalXM:0.02,
      swingProgress01:0.9,materialSlipXYMps:[0,0]})).toBeNull();
    expect(sample({mode:"toe-gated",contactLocalXM:0.10,
      swingProgress01:0.3,materialSlipXYMps:[0,0]})).toBeNull();
    const toe=sample({mode:"toe-gated",contactLocalXM:0.10,
      swingProgress01:0.9,materialSlipXYMps:[0,0]})!;
    expect(toe.intendedPushOffN).toBeCloseTo(3);
    expect(toe.virtualForceWorldN[0]).toBeCloseTo(-3);
    const sliding=sample({mode:"toe-gated",contactLocalXM:0.10,
      swingProgress01:0.9,materialSlipXYMps:[0.07,0]})!;
    expect(sliding.intendedPushOffN).toBe(0); // never push through sliding toes
    expect(sliding.virtualForceWorldN[0]).toBeLessThan(0);
    expect(Math.abs(sliding.virtualForceWorldN[0])).toBeLessThanOrEqual(2);
    expect(()=>sample({mode:"toe-gated",contactLocalXM:Number.NaN})).toThrow(RangeError);
  });
  it("caps virtual correction conservatively by loaded friction headroom",()=>{
    const low=sample({foot:{...foot,verticalLoadN:50,
      tangentialReactionXYN:[20,0],effectiveSlideFrictionMin:0.5,
      frictionUtilizationPeak:0.7},materialSlipXYMps:[1,1]})!;
    expect(Math.hypot(...low.virtualForceWorldN)).toBeLessThanOrEqual(low.forceHeadroomN+1e-10);
    expect(low.forceHeadroomN).toBeLessThan(5);
    expect(()=>sample({strength:Infinity})).toThrow(RangeError);
  });
  it("validates non-finite feedback and keeps root DOFs untouched by internal torques",()=>{
    expect(()=>sample({materialSlipXYMps:[NaN,0]})).toThrow(RangeError);
    expect(()=>sample({strength:-1})).toThrow(RangeError);
    const names=["hip_pitch","hip_roll","knee","ankle_pitch","ankle_roll"];
    const ids=new Map(names.map((x,i)=>["left_"+x,i+1] as const));
    const model={
      njnt:6,
      jnt_type:[0,3,3,3,3,3],
      jnt_dofadr:[0,6,7,8,9,10],
      jnt_qposadr:[0,6,7,8,9,10],
      jnt:(name:string)=>({id:ids.get(name)??-1}),
    } as unknown as Parameters<typeof applyStanceContactCommand>[0];
    const qfrc_applied=new Float64Array(11);
    const xaxis=new Float64Array(18), xanchor=new Float64Array(18);
    for(let id=1;id<=5;id++) xaxis[id*3+1]=1;
    const state={
      qfrc_applied,qpos:new Float64Array(11),
      qvel:new Float64Array(11),xaxis,xanchor,
    };
    const gains=new Map(Array.from({length:5},(_,i)=>[i+1,{
      targetRad:0,stiffnessNmPerRad:1,dampingNmsPerRad:1,maxTorqueNm:20,
    }] as const));
    applyStanceContactCommand(model,state,gains,"left",[0,0,-1],[-100,0,0]);
    expect(Array.from(qfrc_applied.slice(0,6))).toEqual([0,0,0,0,0,0]);
    expect(qfrc_applied[6]).toBe(20);
    expect(qfrc_applied[10]).toBe(20);
  });
});
