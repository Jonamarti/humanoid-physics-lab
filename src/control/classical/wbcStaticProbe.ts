/** First WBC integration probe: actual compiled MuJoCo model, not locomotion.
 * Audit independent analytic M/J against native bias and unconstrained
 * acceleration. Solve one static contact-consistent inverse dynamics task.
 * Only if the solution meets the numerical constraints, apply its internal
 * torques for 40 ms and observe the actual solver's response. No contact
 * force from the optimizer is injected; no state reset during the trial.
 */
import { getMujoco } from "../../scenario/mujocoModule";
import { planarModel,referenceDt,referenceJoints } from "./planarReferenceModel";
import { coronalJointNames } from "./coronalControl";
import { headingJointNames } from "./headingControl";
import { articulatedMass,pointJacobian,type ArticulatedGeometry,type V3 } from "./articulatedDynamics";
import { solveContactDynamics,type ContactDynamicsProblem,type ContactDynamicsSolution,type LinearRow } from "./contactDynamics";
const vec=(a:ArrayLike<number>,i:number):V3=>[a[i]!,a[i+1]!,a[i+2]!];
const slice9=(a:ArrayLike<number>,i:number):number[]=>Array.from({length:9},(_,j)=>a[i+j]!);
const maxAbs=(a:readonly number[]):number=>a.reduce((s,v)=>Math.max(s,Math.abs(v)),0);
export interface WbcProbe {
  nv:number;massKg:number;gravityError:number;massEquationError:number;comError:number;
  excitedMassEquationError:number;rotatedMassEquationError:number;rotatedGravityError:number;
  supportPoints:number;solution:ContactDynamicsSolution;applied:boolean;
  finalPelvisDisplacementM:number|null;finalComDisplacementM:number|null;maxRootForce:number;
}
export async function runWbcStaticProbe():Promise<WbcProbe> {
  const mj=await getMujoco(),model=mj.MjModel.from_xml_string(planarModel(referenceDt,"free3d-wbc")),data=new mj.MjData(model);
  try{
    const n=model.nv,root=model.body("pelvis").id;
    if(n!==28||model.nq!==29||model.nbody!==18)throw new Error("Unexpected whole-body topology");
    const names=[...referenceJoints.map(x=>x.name),...coronalJointNames,...headingJointNames];
    const hinges=names.map(name=>{
      const id=model.jnt(name).id;
      return {id,name,body:model.jnt_bodyid[id]!,dof:model.jnt_dofadr[id]!};
    });
    if(new Set(hinges.map(h=>h.dof)).size!==22||hinges.some(h=>h.dof<6))throw new Error("Invalid WBC dofs");
    // All generalized velocities initially zero. No initial teleport,
    // free-root force, equality constraint or added ground adhesion.
    mj.mj_forward(model,data);
    const pos=data.xpos as Float64Array;
    function geometryOf(d:typeof data):ArticulatedGeometry{
      const positions=d.xpos as Float64Array,rotations=d.xmat as Float64Array;
      const ipos=d.xipos as Float64Array,irot=d.ximat as Float64Array;
      const anchors=d.xanchor as Float64Array,axes=d.xaxis as Float64Array;
      return {nv:n,rootBody:root,rootPosition:vec(positions,3*root),rootRotation:slice9(rotations,9*root),
        bodies:Array.from({length:model.nbody},(_,b)=>({parent:model.body_parentid[b]!,mass:model.body_mass[b]!,
          inertia:vec(model.body_inertia as Float64Array,3*b),center:vec(ipos,3*b),inertialRotation:slice9(irot,9*b)})),
        hinges:hinges.map(h=>({body:h.body,dof:h.dof,axis:vec(axes,3*h.id),anchor:vec(anchors,3*h.id)})),
        armature:Array.from(model.dof_armature as Float64Array)};
    }
    const geometry=geometryOf(data);
    const dyn=articulatedMass(geometry);
    const bias=Array.from(data.qfrc_bias as Float64Array);
    const passive=Array.from(data.qfrc_passive as Float64Array);
    const smooth=Array.from(data.qfrc_smooth as Float64Array),accSmooth=Array.from(data.qacc_smooth as Float64Array);
    const gravityError=maxAbs(dyn.gravity.map((v,i)=>v-bias[i]!));
    const massEquationError=maxAbs(dyn.mass.map((row,i)=>row.reduce((s,v,j)=>s+v*accSmooth[j]!,0)-smooth[i]!));
    const com0=vec(data.subtree_com as Float64Array,3*root);
    const comError=Math.hypot(...dyn.com.map((v,i)=>v-com0[i]!));
    if(![gravityError,massEquationError,comError].every(Number.isFinite)||gravityError>1e-5||massEquationError>1e-5||comError>1e-8)
      throw new Error("WBC native dynamics audit failed: "+JSON.stringify({gravityError,massEquationError,comError}));
    // The gravity-only equation is insufficient to validate off-diagonal
    // inertia. Use a separate data instance to excite every internal DoF;
    // no mj_step, no root effort, no edits to the real standing state.
    const scratch=new mj.MjData(model);
    let excitedMassEquationError=0,rotatedMassEquationError=0,rotatedGravityError=0;
    try{
      const q=scratch.qpos as Float64Array,tau=scratch.qfrc_applied as Float64Array;
      q.set(data.qpos as Float64Array);
      for(const rotated of [false,true]){
        if(rotated){
          q[3]=Math.cos(.13);q[4]=Math.sin(.13);q[5]=0;q[6]=0;
          for(const h of hinges){
            const adr=model.jnt_qposadr[h.id]!;
            if(h.name.endsWith("Knee"))q[adr]=.2;
            if(h.name.endsWith("HipRoll"))q[adr]=h.name.startsWith("l")?.09:-.07;
            if(h.name.endsWith("HipYaw"))q[adr]=.08;
            if(h.name.endsWith("Hip"))q[adr]=-.12;
          }
        }
        tau.fill(0);mj.mj_forward(model,scratch);
        const d=articulatedMass(geometryOf(scratch));
        if(rotated)rotatedGravityError=maxAbs(d.gravity.map((v,i)=>v-(scratch.qfrc_bias as Float64Array)[i]!));
        for(let j=6;j<n;j++){
          tau.fill(0);tau[j]=3;
          mj.mj_forward(model,scratch);
          const a=scratch.qacc_smooth as Float64Array,s=scratch.qfrc_smooth as Float64Array;
          const error=maxAbs(d.mass.map((row,i)=>row.reduce((v,m,k)=>v+m*a[k]!,0)-s[i]!));
          if(rotated)rotatedMassEquationError=Math.max(rotatedMassEquationError,error);
          else excitedMassEquationError=Math.max(excitedMassEquationError,error);
        }
      }
    }finally{scratch.delete();}
    if(![excitedMassEquationError,rotatedMassEquationError,rotatedGravityError].every(Number.isFinite)||
      Math.max(excitedMassEquationError,rotatedMassEquationError,rotatedGravityError)>1e-5)
      throw new Error("WBC excited native dynamics audit failed: "+JSON.stringify({excitedMassEquationError,rotatedMassEquationError,rotatedGravityError}));
    const contacts:ContactDynamicsProblem["contacts"][number][]=[];
    const gx=data.geom_xpos as Float64Array,gm=data.geom_xmat as Float64Array,sizes=model.geom_size as Float64Array;
    for(const side of ["l","r"]){
      const g=model.geom(side+"Foot_geom").id,b=model.body(side+"Foot").id;
      for(const sx of [-1,1])for(const sy of [-1,1]){
        const local=[sx*sizes[3*g]!,sy*sizes[3*g+1]!,-sizes[3*g+2]!];
        const p:V3=[0,1,2].map(i=>gx[3*g+i]!+local.reduce((s,v,j)=>s+v*gm[9*g+3*i+j]!,0)) as unknown as V3;
        // These are candidate points on the actual sole at the ground
        // surface. Do not invent a contact above or below the floor.
        if(Math.abs(p[2])>.002)throw new Error("Static candidate is not on floor");
        contacts.push({jacobian:pointJacobian(geometry,b,p).linear,mu:.25,maxNormalN:dyn.totalMass*9.81});
      }
    }
    const limits=Array<number>(n-6).fill(0);
    for(const h of hinges)limits[h.dof-6]=h.name.endsWith("Yaw")?40:h.name.endsWith("HipRoll")?100:h.name.endsWith("AnkleRoll")?45:370;
    const tasks:LinearRow[]=Array.from({length:n},(_,i)=>({a:Array.from({length:n},(_,j)=>i===j?1:0),b:0,weight:1}));
    for(const row of dyn.comJacobian)tasks.push({a:row,b:0,weight:10});
    const solution=solveContactDynamics({mass:dyn.mass,bias:bias.map((v,i)=>v-passive[i]!),rootDofs:6,torqueLimits:limits,contacts,tasks});
    const applied=data.qfrc_applied as Float64Array;
    let maxRootForce=0,finalPelvisDisplacementM:number|null=null,finalComDisplacementM:number|null=null;
    const pelvis0=vec(pos,3*root);
    // Numerical feasibility is necessary but not sufficient. Refuse even
    // a feasible solve if the static acceleration/contact tasks are poor.
    const use=solution.feasible&&maxAbs(solution.acceleration)<.25&&solution.contactAccelerationResidual<.05;
    if(use){
      for(let d=6;d<n;d++)applied[d]=solution.torques[d-6]!;
      for(let k=0;k<160;k++){
        for(let d=0;d<6;d++)maxRootForce=Math.max(maxRootForce,Math.abs(applied[d]!));
        mj.mj_step(model,data);
        if(!(data.qpos as Float64Array).every(Number.isFinite))throw new Error("Nonfinite native WBC probe");
      }
      mj.mj_forward(model,data);
      finalPelvisDisplacementM=Math.hypot(...vec(pos,3*root).map((v,i)=>v-pelvis0[i]!));
      finalComDisplacementM=Math.hypot(...vec(data.subtree_com as Float64Array,3*root).map((v,i)=>v-com0[i]!));
    }
    return {nv:n,massKg:dyn.totalMass,gravityError,massEquationError,comError,excitedMassEquationError,rotatedMassEquationError,rotatedGravityError,
      supportPoints:contacts.length,solution,applied:use,
      finalPelvisDisplacementM,finalComDisplacementM,maxRootForce};
  }finally{data.delete();model.delete();}
}
