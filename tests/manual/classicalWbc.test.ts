import { describe,it,expect } from "vitest";
import { constrainedLeastSquares,solveContactDynamics } from "../../src/control/classical/contactDynamics";
import { articulatedMass,pointJacobian,type ArticulatedGeometry } from "../../src/control/classical/articulatedDynamics";
import { runWbcStaticProbe } from "../../src/control/classical/wbcStaticProbe";

describe("Deterministic constrained dynamics",()=>{
  it("finds the constrained optimum and is repeatable",()=>{
    const p={n:2,tasks:[{a:[1,0],b:3},{a:[0,1],b:1}],constraints:[{a:[1,1],lo:2,hi:2},{a:[1,0],lo:0,hi:1.5}]};
    const a=constrainedLeastSquares(p),b=constrainedLeastSquares(p);
    expect(a).toEqual(b);
    expect(a.converged).toBe(true);
    expect(a.x[0]).toBeCloseTo(1.5,5);
    expect(a.x[1]).toBeCloseTo(.5,5);
    expect(a.maxViolation).toBeLessThan(1e-6);
  });
  it("reports an inconsistent constraint rather than declaring success",()=>{
    const r=constrainedLeastSquares({n:1,tasks:[],constraints:[{a:[1],lo:2,hi:Infinity},{a:[1],lo:-Infinity,hi:1}],iterations:100});
    expect(r.converged).toBe(false);
    expect(r.maxViolation).toBeGreaterThan(.1);
  });
  it("balances a 10 kg floating coordinate through contact, not a root motor",()=>{
    const r=solveContactDynamics({mass:[[10,0],[0,1]],bias:[98.1,0],rootDofs:1,torqueLimits:[150],
      contacts:[{jacobian:[[0,0],[0,0],[1,1]],mu:.3,maxNormalN:150}],
      tasks:[{a:[1,0],b:0,weight:10},{a:[0,1],b:0,weight:10}]});
    expect(r.feasible).toBe(true);
    expect(r.forces[0]![2]).toBeCloseTo(98.1,1);
    expect(r.torques[0]).toBeCloseTo(-98.1,1);
    expect(r.maxRootResidual).toBeLessThan(.05);
    expect(r.maxFrictionViolation).toBeLessThan(.05);
  });
  it("never creates a friction-infeasible contact to achieve an acceleration task",()=>{
    const r=solveContactDynamics({mass:[[10,0,0],[0,10,0],[0,0,1]],bias:[0,98.1,0],rootDofs:2,torqueLimits:[150],
      contacts:[{jacobian:[[1,0,0],[0,0,0],[0,1,1]],mu:.2,maxNormalN:100}],
      tasks:[{a:[1,0,0],b:100,weight:10},{a:[0,1,0],b:0,weight:10},{a:[0,0,1],b:0,weight:10}]});
    expect(r.maxFrictionViolation).toBeLessThan(.05);
    expect(r.acceleration[0]).toBeLessThan(2.01);
    expect(r.maxRootResidual).toBeLessThan(.05);
  });
});
describe("Articulated mass and Jacobian",()=>{
  const I=[1,0,0,0,1,0,0,0,1];
  const g:ArticulatedGeometry={nv:7,rootBody:1,rootPosition:[0,0,0],rootRotation:I,
    bodies:[{parent:0,mass:0,inertia:[0,0,0],center:[0,0,0],inertialRotation:I},
      {parent:0,mass:2,inertia:[1,1,1],center:[0,0,0],inertialRotation:I},
      {parent:1,mass:3,inertia:[1,1,1],center:[0,0,1],inertialRotation:I}],
    hinges:[{body:2,dof:6,axis:[0,1,0],anchor:[0,0,0]}],armature:[0,0,0,0,0,0,.2]};
  it("builds symmetric mass matrix and correct gravity/COM",()=>{
    const d=articulatedMass(g);
    expect(d.totalMass).toBe(5);
    expect(d.com).toEqual([0,0,.6]);
    expect(d.gravity[2]).toBeCloseTo(49.05);
    expect(d.mass[0]![6]).toBeCloseTo(3);
    expect(d.mass[6]![6]).toBeCloseTo(4.2);
    expect(d.mass[6]![0]).toBeCloseTo(3);
    expect(d.mass[2]![2]).toBe(5);
    expect(d.comJacobian[0]![6]).toBeCloseTo(.6);
    expect(pointJacobian(g,1,[0,0,0]).linear[0]![6]).toBe(0);
    expect(pointJacobian(g,2,[0,0,1]).linear[0]![6]).toBe(1);
  });
});
const enabled=process.env.HPL_REFERENCE==="1"||
  (process.env.GITHUB_ACTIONS==="true"&&process.env.GITHUB_HEAD_REF==="experiment/classical-c0-simbicon");
describe("Native full-body inverse dynamics integration",()=>{
  it.skipIf(!enabled)("audits native mass/COM and attempts a physically feasible static support solve",async()=>{
    const r=await runWbcStaticProbe();
    console.log("CLASSICAL_WBC_STATIC "+JSON.stringify(r));
    expect(r.nv).toBe(28);
    expect(r.massKg).toBeCloseTo(63.5,8);
    expect(r.gravityError).toBeLessThan(1e-5);
    expect(r.massEquationError).toBeLessThan(1e-5);
    expect(r.comError).toBeLessThan(1e-8);
    expect(r.supportPoints).toBe(8);
    expect(r.maxRootForce).toBe(0);
    if(r.applied){
      expect(r.solution.feasible).toBe(true);
      expect(r.finalPelvisDisplacementM).not.toBeNull();
      expect(r.finalComDisplacementM).not.toBeNull();
    }
    // A nonconvergent solver or poor static task is a diagnostic failure:
    // report it rather than apply its forces or claim standing success.
  },120000);
});
