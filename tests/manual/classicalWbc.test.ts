import { describe,it,expect,afterAll } from "vitest";
import { createHash } from "node:crypto";
import { gzipSync } from "node:zlib";
import { constrainedLeastSquares,solveContactDynamics } from "../../src/control/classical/contactDynamics";
import { articulatedMass,pointJacobian,type ArticulatedGeometry } from "../../src/control/classical/articulatedDynamics";
import { runWbcStaticProbe,lateralReference,swingReference,type WbcFeedback,type WbcMode } from "../../src/control/classical/wbcStaticProbe";

describe("Deterministic constrained dynamics",()=>{
  it("finds the constrained optimum and is repeatable",()=>{
    const p={n:2,tasks:[{a:[1,0],b:3},{a:[0,1],b:1}],constraints:[{a:[1,1],lo:2,hi:2},{a:[1,0],lo:0,hi:1.5}]};
    const a=constrainedLeastSquares(p),b=constrainedLeastSquares(p);
    expect(a).toEqual(b);
    expect(a.converged).toBe(true);
    expect(a.method).toBe("active-set");
    expect(a.dualResidual).toBeLessThan(1e-6);
    expect(a.x[0]).toBeCloseTo(1.5,5);
    expect(a.x[1]).toBeCloseTo(.5,5);
    expect(a.maxViolation).toBeLessThan(1e-6);
  });
  it("handles redundant equalities through the independently checked fallback",()=>{
    const r=constrainedLeastSquares({n:1,tasks:[{a:[1],b:2}],constraints:[{a:[1],lo:1,hi:1},{a:[1],lo:1,hi:1}]});
    expect(r.method).toBe("admm");
    expect(r.converged).toBe(true);
    expect(r.x[0]).toBeCloseTo(1,5);
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
describe("Smooth mirrored weight-transfer references",()=>{
  it("preserves nominal standing and has zero endpoint derivatives",()=>{
    for(const t of [0,.5,2.5,4,5.5,6]){
      const r=lateralReference("standing",t);
      expect(r.offsetM).toBe(0);
      expect(r.velocityMps).toBe(0);
      expect(r.accelerationMps2).toBe(0);
      const left=lateralReference("shift-left",t),right=lateralReference("shift-right",t);
      expect(left.offsetM+right.offsetM).toBeCloseTo(0,12);
      expect(left.velocityMps).toBeCloseTo(0,12);
      expect(left.accelerationMps2).toBeCloseTo(0,12);
    }
    expect(lateralReference("shift-left",3).offsetM).toBeCloseTo(.085);
    expect(lateralReference("shift-right",3).offsetM).toBeCloseTo(-.085);
    expect(lateralReference("shift-left",1.5).offsetM).toBeCloseTo(.0425);
    expect(lateralReference("shift-left",4.75).offsetM).toBeCloseTo(.0425);
  });
  it("matches the analytic trajectory derivatives at interior times",()=>{
    const h=1e-5;
    for(const t of [.8,1.5,2.2,4.2,4.75,5.2]){
      const a=lateralReference("shift-left",t-h),b=lateralReference("shift-left",t),c=lateralReference("shift-left",t+h);
      expect((c.offsetM-a.offsetM)/(2*h)).toBeCloseTo(b.velocityMps,6);
      expect((c.velocityMps-a.velocityMps)/(2*h)).toBeCloseTo(b.accelerationMps2,6);
      expect(lateralReference("shift-right",t).offsetM+b.offsetM).toBeCloseTo(0,12);
    }
  });
});
describe("Unilateral references",()=>{
  it("extends both shifts smoothly and lowers the whole-foot target",()=>{
    for(const time of [0,.5,2.5,4,5.5,6.5,7.5,10,11,11.5,13.5,14]){
      const l=lateralReference("lift-left",time),r=lateralReference("lift-right",time);
      expect(l.offsetM+r.offsetM).toBeCloseTo(0,12);
      expect(l.velocityMps+r.velocityMps).toBeCloseTo(0,12);
      expect(l.accelerationMps2+r.accelerationMps2).toBeCloseTo(0,12);
      expect(l.velocityMps).toBeCloseTo(0,12);
      expect(l.accelerationMps2).toBeCloseTo(0,12);
    }
    expect(lateralReference("lift-left",4).offsetM).toBeCloseTo(.085);
    expect(lateralReference("lift-left",6.5).offsetM).toBeCloseTo(.115);
    expect(lateralReference("lift-left",14).offsetM).toBe(0);
    for(const time of [6.5,7.5,10,11]){
      expect(swingReference(time).velocityMps).toBeCloseTo(0,12);
      expect(swingReference(time).accelerationMps2).toBeCloseTo(0,12);
    }
    expect(swingReference(7).offsetM).toBeCloseTo(.02);
    expect(swingReference(8).offsetM).toBeCloseTo(.04);
    expect(swingReference(10.5).offsetM).toBeCloseTo(.02);
    expect(swingReference(11).offsetM).toBe(0);
  });
});
// Emit all numerical summaries before bulky lossless replay chunks.
const pendingReplay:Array<{f:WbcFeedback;mode:WbcMode;duration:number}>=[];
afterAll(()=>{
  for(const {f,mode,duration} of pendingReplay){
    const bytes=Buffer.from(JSON.stringify({partNames:f.partNames,recordHz:f.recordHz,frames:f.frames}),"utf8");
    const packed=gzipSync(bytes).toString("base64"),sha256=createHash("sha256").update(bytes).digest("hex");
    const chunk=3000,count=Math.ceil(packed.length/chunk);
    for(let i=0;i<count;i++)console.log("CLASSICAL_WBC_REPLAY "+JSON.stringify({mode,duration,index:i,count,sha256,data:packed.slice(i*chunk,(i+1)*chunk)}));
  }
});
function recordFeedback(f:WbcFeedback,mode:WbcMode,duration:number){
  console.log("CLASSICAL_WBC_FEEDBACK "+JSON.stringify({...f,trace:undefined,frames:undefined}));
  const trace=f.trace.filter((x,i)=>i%20===0||i===f.trace.length-1||i>0&&x.phase!==f.trace[i-1]!.phase);
  console.log("CLASSICAL_WBC_TRACE "+JSON.stringify({mode,duration,trace}));
  expect(f.maxRootForce).toBe(0);
  expect(f.maxExternalForce).toBe(0);
  expect(f.maxTorqueRatio).toBeLessThanOrEqual(1+1e-8);
  expect(f.acceptedUpdates).toBeLessThanOrEqual(f.updates);
  expect(f.frames.length).toBeGreaterThan(0);
  expect(f.frames.every(p=>p.pose.length===17&&p.pose.every(q=>q.length===7&&q.every(Number.isFinite)))).toBe(true);
  expect([f.maxComDisplacementM,f.finalComSpeedMps,f.maxTiltDeg,f.minPelvisM,f.maxSampledMaterialTravelM,
    f.maxGroundPenetrationM,f.maxContactResidual,f.maxRootResidual,...f.minFootLoadN].every(Number.isFinite)).toBe(true);
  pendingReplay.push({f,mode,duration});
}
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
    // Require numerical feasibility for the static integration milestone.
    // Motion quality during the 40ms native probe is reported separately.
    expect(r.solution.feasible).toBe(true);
    expect(r.applied).toBe(true);
    expect(r.finalPelvisDisplacementM).not.toBeNull();
    expect(r.finalComDisplacementM).not.toBeNull();
    expect(Number.isFinite(r.finalPelvisDisplacementM)).toBe(true);
    expect(Number.isFinite(r.finalComDisplacementM)).toBe(true);
  },120000);
  it.skipIf(!enabled).each([1,60])("maintains contact-aware closed-loop standing for %s seconds",async duration=>{
    const r=await runWbcStaticProbe(duration),f=r.feedback;
    expect(r.applied).toBe(true);
    expect(f).toBeDefined();
    if(!f)throw new Error("Initial static solve not accepted");
    recordFeedback(f,"standing",duration);
    // Standing endurance gate, on 100 Hz observations. The physical solver
    // still advances at 4000 Hz. Slip is a sampled material-point estimate,
    // not a guarantee of all substep contact behavior.
    expect(f.stopReason).toBe("duration");
    expect(f.executedS).toBeCloseTo(duration,6);
    expect(f.updates).toBe(duration*100);
    expect(f.acceptedUpdates).toBe(duration*100);
    expect(f.maxBilateralLoadLossS).toBe(0);
    expect(f.minFootLoadN.every(v=>v>=40)).toBe(true);
    expect(f.maxTiltDeg).toBeLessThan(5);
    expect(f.minPelvisM).toBeGreaterThan(.85);
    expect(f.maxComDisplacementM).toBeLessThan(.02);
    expect(f.finalComSpeedMps).toBeLessThan(.1);
    expect(f.maxSampledMaterialTravelM).toBeLessThan(.02);
    expect(f.maxGroundPenetrationM).toBeLessThan(.005);
  },300000);
  it.skipIf(!enabled)("rechecks mirrored bilateral transfers before attempting unilateral support",async()=>{
    let bothQualified=true;
    for(const mode of ["shift-left","shift-right"] as const){
    const r=await runWbcStaticProbe(6,mode),f=r.feedback;
    expect(r.applied).toBe(true);
    expect(f).toBeDefined();
    if(!f)throw new Error("Initial static solve not accepted");
    recordFeedback(f,mode,6);
    // Diagnostic: qualify only actual measured support >=75% body weight,
    // near the reference and with small lateral speed for >=.25 s; qualify
    // recenter independently for >=.25 s. A failure is not software success.
    const qualified=f.stopReason==="duration"&&f.longestQualifiedShiftS>=.25&&f.longestRecenterS>=.25;
    bothQualified=bothQualified&&qualified;
    console.log("CLASSICAL_WBC_SHIFT_RESULT "+JSON.stringify({mode,qualified,
      maxSupportWeightRatio:f.maxSupportWeightRatio,maxSignedComShiftM:f.maxSignedComShiftM,
      longestQualifiedShiftS:f.longestQualifiedShiftS,longestRecenterS:f.longestRecenterS}));
    }
    if(!bothQualified){
      console.log("CLASSICAL_WBC_LIFT_SKIPPED both bilateral transfers must qualify");
      return;
    }
    for(const mode of ["lift-left","lift-right"] as const){
      const r=await runWbcStaticProbe(14,mode),f=r.feedback;
      expect(r.applied).toBe(true);
      if(!f)throw new Error("Initial static solve not accepted");
      recordFeedback(f,mode,14);
      // Diagnostic only: do not confuse a successful software run with
      // physical success. Require measured unload, all-volume clearance,
      // real single support >=2s, touchdown and bilateral recovery.
      const qualified=f.stopReason==="duration"&&f.liftAttempted&&f.longestUnloadS>=.25&&
        f.longestUnilateralS>=2&&f.touchdownObserved&&f.longestRecenterS>=.25&&
        f.maxTiltDeg<5&&f.minPelvisM>.85&&f.maxSampledMaterialTravelM<.02&&f.maxGroundPenetrationM<.005;
      console.log("CLASSICAL_WBC_LIFT_RESULT "+JSON.stringify({mode,qualified,stopReason:f.stopReason,
        liftAttempted:f.liftAttempted,longestUnloadS:f.longestUnloadS,longestUnilateralS:f.longestUnilateralS,
        maxSwingClearanceM:f.maxSwingClearanceM,touchdownObserved:f.touchdownObserved,
        longestRecenterS:f.longestRecenterS}));
    }
  },300000);
});
