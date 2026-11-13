import { describe, expect, it } from "vitest";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { validateScenario } from "../../src/scenario/validate";
import { buildThrowPhysicsScenario, physicsScenarioHash } from "../../public/throw-axis-physics.js";

// Synthetic 23-point pose; no proprietary rig or animation data.
const BASE = [
  [.18,.12,.02],[.18,-.12,.02],[-.06,.12,.03],[-.06,-.12,.03],[0,.12,.10],[0,-.12,.10],
  [.04,.12,.52],[.04,-.12,.52],[0,.11,.93],[0,-.11,.93],
  [.02,.22,1.37],[.02,-.22,1.37],[.12,.38,1.25],[.12,-.38,1.25],
  [.25,.42,1.15],[.25,-.42,1.15],[.28,.42,1.10],[.28,-.42,1.10],
  [.33,.42,1.10],[.33,-.42,1.10],[0,0,1.12],[0,0,1.59],[.05,0,1.72],
];
const uke = BASE.map(p=>[...p]);
function frames(technique: "tai" | "osoto") {
  const out = [];
  for (let i=0;i<=96;i++){
    const t=i/30;
    const yaw=technique==="tai" ? Math.PI-t*.6 : Math.PI+t*.08;
    const c=Math.cos(yaw),s=Math.sin(yaw);
    const tori=BASE.map(p=>[.6+c*p[0]-s*p[1],.03+s*p[0]+c*p[1],p[2]]);
    if(technique==="tai"){
      tori[5]=[.14,-.25,.08];
      tori[7]=[.23,-.08,.51];
    }else{
      tori[5]=[.35,-.28,Math.max(.08,.5*Math.sin(t))];
      tori[7]=[.28,-.25,.57];
    }
    out.push({t,tori,uke});
  }
  return out;
}
describe("experimento físico aislado de Tai Otoshi / O Soto Gari",()=>{
  it.each(["tai","osoto"] as const)("genera escenarios válidos y modelos físicos para %s",technique=>{
    const data=frames(technique);
    for(const model of ["full","support"] as const){
      const spec=buildThrowPhysicsScenario(data,{technique,model,support:"leg"});
      const scenario=validateScenario(spec);
      const physics=buildScenarioModel(scenario);
      expect(physics.mjcf).toContain("<mujoco");
      expect(physics.bodies).toContain("pelvis");
      if(model==="full")expect(physics.bodies).toContain("tori_trunk");
      else expect(physics.bodies).toContain("tori_contact");
      expect(scenario.actors).toHaveLength(1);
      expect(scenario.id).toMatch(/^throw_axis_/);
      expect(physicsScenarioHash(spec)).toMatch(/^#scn=/);
    }
  });
  it.each(["tai","osoto"] as const)("sin apoyo mantiene exactamente las fuerzas (%s)",technique=>{
    const data=frames(technique);
    const withLeg=buildThrowPhysicsScenario(data,{technique,model:"support",support:"leg"});
    const control=buildThrowPhysicsScenario(data,{technique,model:"support",support:"leg",withoutSupport:true});
    expect(withLeg.effectors).toEqual(control.effectors);
    expect(withLeg.actors).toEqual(control.actors);
    expect(withLeg.obstacles).toHaveLength(1);
    expect(control.obstacles).toBeUndefined();
    expect(validateScenario(control).effectors).toHaveLength(2);
  });
  it("no modifica fotogramas de entrada ni grabaciones existentes",()=>{
    const data=frames("tai"),before=JSON.stringify(data);
    const spec=buildThrowPhysicsScenario(data,{technique:"tai",model:"full"});
    expect(JSON.stringify(data)).toBe(before);
    expect(spec.tori?.body.keys.length).toBeGreaterThan(5);
  });
  it("rechaza modelos y secuencias imposibles",()=>{
    expect(()=>buildThrowPhysicsScenario(frames("tai").slice(0,2),{technique:"tai"})).toThrow();
    expect(()=>buildThrowPhysicsScenario(frames("tai"),{technique:"tai",model:"full",withoutSupport:true})).toThrow();
  });
});
