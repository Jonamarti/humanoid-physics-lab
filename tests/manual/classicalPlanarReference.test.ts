import { describe,it,expect } from "vitest";
import { createHash } from "node:crypto";
import { gzipSync } from "node:zlib";
import { planarModel,referenceParts,referenceJoints,referenceInitialPosition } from "../../src/control/classical/planarReferenceModel";
import { initialPlanarState,planarCommand,type PlanarObservation } from "../../src/control/classical/planarReferenceControl";
import { headingCommand,headingLimitNm,type HeadingObservation } from "../../src/control/classical/headingControl";
import { runPlanarReference } from "../../src/control/classical/planarReferenceRun";

const empty=():PlanarObservation=>({angle:Object.fromEntries(referenceJoints.map(j=>[j.name,0])),
  rate:Object.fromEntries(referenceJoints.map(j=>[j.name,0])),torsoPitch:0,thighPitch:{l:0,r:0},ankleX:{l:0,r:0},
  comX:0,comVelocityX:0,contact:{l:false,r:false}});
describe("Planar MuJoCo reference model and controller",()=>{
  it("preserves source body count, mass and coordinate mapping",()=>{
    expect(referenceParts.length).toBe(17);
    expect(referenceJoints.length).toBe(16);
    expect(referenceParts.reduce((s,p)=>s+p.mass,0)).toBeCloseTo(63.5,10);
    expect(referenceInitialPosition("lFoot")).toEqual([.065,.1,.025]);
    expect(referenceInitialPosition("rFoot")).toEqual([.065,-.1,.025]);
    expect(planarModel()).toContain('timestep="0.00025"');
    expect(planarModel()).not.toContain("freejoint");
    expect(planarModel(.00025,"free3d")).toContain('<freejoint name="root"/>');
    expect(planarModel(.00025,"free3d")).not.toContain('name="root_x"');
    for(const n of ["lHipRoll","rHipRoll","lAnkleRoll","rAnkleRoll"]){
      expect(planarModel(.00025,"free3d-roll")).toContain('name="'+n+'"');
      expect(planarModel(.00025,"free3d")).not.toContain('name="'+n+'"');
    }
  });
  it("uses the timed phase and contact transition without counting time as landing",()=>{
    const o=empty(),first=planarCommand(initialPlanarState(),o,.25);
    expect(first.state.phase).toBe(0);
    expect(first.torques.rHip).toBeCloseTo(-120);
    expect(first.torques.lHip).toBeCloseTo(120);
    expect(first.torques.rKnee).toBeCloseTo(330);
    const second=planarCommand(first.state,o,.10);
    expect(second.state.phase).toBe(1);
    const third=planarCommand(second.state,o,.1);
    expect(third.state.phase).toBe(1);
    o.contact.r=true;
    expect(planarCommand(third.state,o,.01).state.phase).toBe(2);
  });
  it("limits the reference's applied hinge torques",()=>{
    const o=empty();o.rate.rKnee=-100;o.rate.lKnee=100;
    const c=planarCommand(initialPlanarState(),o,.00025);
    expect(c.torques.rKnee).toBe(370);
    expect(c.torques.lKnee).toBe(-370);
    expect(Object.keys(c.torques)).toHaveLength(16);
  });
});
describe("Independent axial heading controller",()=>{
  const base:HeadingObservation={yaw:.2,yawRate:0,footYaw:[0,0],footYawRate:[0,0],jointQ:[0,0],jointRate:[0,0],
    axisZ:[1,1],loadN:[600,0],filteredLoadN:[600,0]};
  it("applies the opposite pelvis reaction through the loaded hip only",()=>{
    const r=headingCommand("heading",base);
    expect(r.hipNm[0]).toBeCloseTo(14);
    expect(r.hipNm[1]).toBeCloseTo(0);
    expect(r.projectedReactionNm).toBeCloseTo(-14);
    expect(r.residualNm).toBeCloseTo(0);
  });
  it("uses the world-axis projection and preserves torque limits",()=>{
    const r=headingCommand("heading",{...base,axisZ:[.5,1]});
    expect(r.hipNm[0]).toBeCloseTo(28);
    expect(r.residualNm).toBeCloseTo(0);
    const limited=headingCommand("heading",{...base,yaw:3,axisZ:[.5,1]});
    expect(limited.hipNm.every(v=>Math.abs(v)<=headingLimitNm)).toBe(true);
    expect(limited.residualNm).toBeLessThan(0);
  });
  it("does not command a net heading correction without loaded contacts",()=>{
    const r=headingCommand("heading",{...base,loadN:[0,0]});
    expect(r.desiredRootNm).toBe(0);
    expect(r.hipNm).toEqual([0,0]);
    expect(headingCommand("neutral",base).hipNm).toEqual([0,0]);
  });
});
const enabled=process.env.HPL_REFERENCE==="1"||
  (process.env.GITHUB_ACTIONS==="true"&&process.env.GITHUB_HEAD_REF==="experiment/classical-c0-simbicon");
describe("Planar reference MuJoCo adaptation (diagnostic, not 3D validation)",()=>{
  it.skipIf(!enabled).each([.00025,.002])("measures physical motion at dt=%s",async dt=>{
    const r=await runPlanarReference(dt);
    console.log("CLASSICAL_PLANAR_MJ "+JSON.stringify({...r,frames:undefined}));
    expect(r.massKg).toBeCloseTo(63.5,8);
    expect(r.maxInitialPositionErrorM).toBeLessThan(1e-8);
    expect(r.maxRootForce).toBe(0);
    expect(r.loadedLandings).toBeLessThanOrEqual(r.qualifiedLandings);
    expect(r.longestLoadedConsecutive).toBeLessThanOrEqual(r.longestConsecutive);
    expect(r.maxLateralM).toBeLessThan(1e-8);
    expect(r.frames.length).toBeGreaterThan(0);
    expect(Object.values(r.maxJointTorqueNm).every(x=>x<=370+1e-8)).toBe(true);
    expect(r.frames.every(f=>f.pose.length===17&&f.pose.every(p=>p.length===7&&p.every(Number.isFinite)))).toBe(true);
    const bytes=Buffer.from(JSON.stringify({partNames:r.partNames,frames:r.frames}),"utf8");
    const packed=gzipSync(bytes).toString("base64"),hash=createHash("sha256").update(bytes).digest("hex");
    const chunk=3000,count=Math.ceil(packed.length/chunk);
    for(let i=0;i<count;i++)console.log("CLASSICAL_PLANAR_MJ_REPLAY "+JSON.stringify({dt,index:i,count,sha256:hash,data:packed.slice(i*chunk,(i+1)*chunk)}));
    // The verified 4000 Hz planar reference is now a regression gate.
    // This is the original geometric-contact criterion, NOT a loaded
    // landing, slip, 3D, stop, or human-naturalness acceptance gate.
    if(dt===.00025){
      expect(r.firstFallS).toBeNull();
      expect(r.nonfinite).toBeNull();
      expect(r.durationS).toBe(12);
      expect(r.longestConsecutive).toBeGreaterThanOrEqual(20);
    }
    // The 500 Hz variant remains a diagnostic.
  },180000);
  it.skipIf(!enabled)("measures the unchanged sagittal controller with a genuinely free 3D root",async()=>{
    const r=await runPlanarReference(.00025,12,false,"free3d");
    console.log("CLASSICAL_FREE3D "+JSON.stringify({...r,frames:undefined}));
    expect(r.space).toBe("free3d");
    expect(r.massKg).toBeCloseTo(63.5,8);
    expect(r.maxInitialPositionErrorM).toBeLessThan(1e-8);
    expect(r.maxRootForce).toBe(0);
    expect(r.frames.length).toBeGreaterThan(0);
    expect(r.frames.every(f=>f.pose.length===17&&f.pose.every(p=>p.length===7&&p.every(Number.isFinite)))).toBe(true);
    const bytes=Buffer.from(JSON.stringify({partNames:r.partNames,frames:r.frames}),"utf8");
    const packed=gzipSync(bytes).toString("base64"),hash=createHash("sha256").update(bytes).digest("hex");
    const chunk=3000,count=Math.ceil(packed.length/chunk);
    for(let i=0;i<count;i++)console.log("CLASSICAL_FREE3D_REPLAY "+JSON.stringify({dt:r.dt,index:i,count,sha256:hash,data:packed.slice(i*chunk,(i+1)*chunk)}));
    // No claim of 3D locomotion: no lateral hip/ankle DOFs yet.
  },180000);
  it.skipIf(!enabled).each(["posture","capture"] as const)("measures free-root coronal policy %s",async policy=>{
    const r=await runPlanarReference(.00025,12,false,"free3d-roll",policy);
    console.log("CLASSICAL_CORONAL "+JSON.stringify({...r,frames:undefined,coronalTrace:undefined}));
    console.log("CLASSICAL_CORONAL_TRACE "+JSON.stringify({policy,trace:r.coronalTrace.filter((_,i)=>i%5===0)}));
    expect(r.space).toBe("free3d-roll");
    expect(r.coronalPolicy).toBe(policy);
    expect(r.massKg).toBeCloseTo(63.5,8);
    expect(r.maxInitialPositionErrorM).toBeLessThan(1e-8);
    expect(r.maxRootForce).toBe(0);
    expect(r.frames.length).toBeGreaterThan(0);
    expect(r.coronalTrace.length).toBeGreaterThan(0);
    expect(r.loadedLandings).toBeLessThanOrEqual(r.qualifiedLandings);
    expect(r.longestLoadedConsecutive).toBeLessThanOrEqual(r.longestConsecutive);
    expect(r.frames.every(f=>f.pose.length===17&&f.pose.every(p=>p.length===7&&p.every(Number.isFinite)))).toBe(true);
    for(const side of ["l","r"]){
      expect(r.maxJointTorqueNm[side+"HipRoll"]).toBeLessThanOrEqual(100+1e-8);
      expect(r.maxJointTorqueNm[side+"AnkleRoll"]).toBeLessThanOrEqual(45+1e-8);
    }
    const bytes=Buffer.from(JSON.stringify({partNames:r.partNames,frames:r.frames}),"utf8");
    const packed=gzipSync(bytes).toString("base64"),hash=createHash("sha256").update(bytes).digest("hex");
    const chunk=3000,count=Math.ceil(packed.length/chunk);
    for(let i=0;i<count;i++)console.log("CLASSICAL_CORONAL_REPLAY "+JSON.stringify({policy,dt:r.dt,index:i,count,sha256:hash,data:packed.slice(i*chunk,(i+1)*chunk)}));
    // A diagnostic ablation, not a 3D locomotion acceptance gate.
  },180000);
  it.skipIf(!enabled).each(["neutral","heading"] as const)("measures free-root axial policy %s",async yawPolicy=>{
    const r=await runPlanarReference(.00025,12,false,"free3d-yaw","capture",yawPolicy);
    console.log("CLASSICAL_HEADING "+JSON.stringify({...r,frames:undefined,headingTrace:undefined}));
    console.log("CLASSICAL_HEADING_TRACE "+JSON.stringify({yawPolicy,trace:r.headingTrace.filter((_,i)=>i%5===0)}));
    expect(r.space).toBe("free3d-yaw");
    expect(r.headingPolicy).toBe(yawPolicy);
    expect(r.massKg).toBeCloseTo(63.5,8);
    expect(r.maxInitialPositionErrorM).toBeLessThan(1e-8);
    expect(r.maxRootForce).toBe(0);
    expect(r.frames.length).toBeGreaterThan(0);
    expect(r.headingTrace.length).toBeGreaterThan(0);
    expect(r.loadedLandings).toBeLessThanOrEqual(r.qualifiedLandings);
    expect(r.frames.every(f=>f.pose.length===17&&f.pose.every(p=>p.length===7&&p.every(Number.isFinite)))).toBe(true);
    for(const side of ["l","r"])expect(r.maxJointTorqueNm[side+"HipYaw"]).toBeLessThanOrEqual(headingLimitNm+1e-8);
    const bytes=Buffer.from(JSON.stringify({partNames:r.partNames,frames:r.frames}),"utf8");
    const packed=gzipSync(bytes).toString("base64"),hash=createHash("sha256").update(bytes).digest("hex");
    const chunk=3000,count=Math.ceil(packed.length/chunk);
    for(let i=0;i<count;i++)console.log("CLASSICAL_HEADING_REPLAY "+JSON.stringify({yawPolicy,dt:r.dt,index:i,count,sha256:hash,data:packed.slice(i*chunk,(i+1)*chunk)}));
    // Isolate yaw effects. No claim that a sagittal+axial rig solves roll.
  },180000);
  it.skipIf(!enabled)("records an unactuated negative control with the same model",async()=>{
    const r=await runPlanarReference(.00025,2,true);
    console.log("CLASSICAL_PLANAR_PASSIVE "+JSON.stringify({...r,frames:undefined}));
    expect(r.maxRootForce).toBe(0);
    expect(Object.keys(r.maxJointTorqueNm)).toHaveLength(0);
    expect(r.qualifiedLandings).toBe(0);
  },60000);
});
