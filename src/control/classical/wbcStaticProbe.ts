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
import { decodeSolvedContactForce,hasCompleteSolvedContactRows } from "../../physics/contactForces";
import { clamp } from "./simbiconCore";
const vec=(a:ArrayLike<number>,i:number):V3=>[a[i]!,a[i+1]!,a[i+2]!];
const slice9=(a:ArrayLike<number>,i:number):number[]=>Array.from({length:9},(_,j)=>a[i+j]!);
const maxAbs=(a:readonly number[]):number=>a.reduce((s,v)=>Math.max(s,Math.abs(v)),0);
const dot=(a:readonly number[],b:readonly number[]):number=>a.reduce((s,v,i)=>s+v*b[i]!,0);
export interface WbcFeedbackFrame {t:number;com:V3;pelvis:V3;tiltDeg:number;loadsN:number[];predictedNormalN:number;rootResidual:number;contactResidual:number;solverMethod:string;iterations:number;accepted:boolean}
export interface WbcFeedback {
  requestedS:number;executedS:number;updates:number;acceptedUpdates:number;stopReason:string;
  maxComDisplacementM:number;maxTiltDeg:number;minPelvisM:number;maxTorqueRatio:number;
  maxRootForce:number;maxContactResidual:number;maxRootResidual:number;
  maxNormalPredictionErrorN:number;maxBilateralLoadLossS:number;
  trace:WbcFeedbackFrame[];
  frames:Array<{t:number;pose:number[][]}>;
  partNames:string[];
}
export interface WbcProbe {
  nv:number;massKg:number;gravityError:number;massEquationError:number;comError:number;
  excitedMassEquationError:number;rotatedMassEquationError:number;rotatedGravityError:number;
  supportPoints:number;solution:ContactDynamicsSolution;applied:boolean;
  finalPelvisDisplacementM:number|null;finalComDisplacementM:number|null;maxRootForce:number;
  feedback?:WbcFeedback;
}
export async function runWbcStaticProbe(feedbackS=0):Promise<WbcProbe> {
  if(!Number.isFinite(feedbackS)||feedbackS<0||feedbackS>2||Math.abs(feedbackS*100-Math.round(feedbackS*100))>1e-8)throw new RangeError("Invalid feedback duration");
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
    const initialSoles=["l","r"].map(side=>{
      const g=model.geom(side+"Foot_geom").id;
      return {p:vec(gx,3*g),r:slice9(gm,9*g)};
    });
    // Numerical feasibility is necessary but not sufficient. Refuse even
    // a feasible solve if the static acceleration/contact tasks are poor.
    const use=solution.feasible&&maxAbs(solution.acceleration)<.25&&solution.contactAccelerationResidual<.05;
    let feedback:WbcFeedback|undefined;
    if(use){
      const initialQ=Array.from(data.qpos as Float64Array);
      for(let d=6;d<n;d++)applied[d]=solution.torques[d-6]!;
      for(let k=0;k<160;k++){
        for(let d=0;d<6;d++)maxRootForce=Math.max(maxRootForce,Math.abs(applied[d]!));
        mj.mj_step(model,data);
        if(!(data.qpos as Float64Array).every(Number.isFinite))throw new Error("Nonfinite native WBC probe");
      }
      mj.mj_forward(model,data);
      finalPelvisDisplacementM=Math.hypot(...vec(pos,3*root).map((v,i)=>v-pelvis0[i]!));
      finalComDisplacementM=Math.hypot(...vec(data.subtree_com as Float64Array,3*root).map((v,i)=>v-com0[i]!));
      if(feedbackS>0)feedback=runFeedback(initialQ);
    }
    function runFeedback(initialQ:number[]):WbcFeedback {
      // Begins after the same 40ms static open-loop probe. Feedback at
      // 100 Hz, physical dt remains .00025 s. No state rewrites, no support
      // welds, no externally applied force. Stop before using a bad solve.
      const period=.01,substeps=Math.round(period/referenceDt);
      const partNames=Array<string>(17).fill("");
      for(const name of ["pelvis",...referenceJoints.map(j=>j.child)])partNames[model.body(name).id-1]=name;
      const floor=model.geom("floor").id,feet=["l","r"].map(side=>({geom:model.geom(side+"Foot_geom").id,body:model.body(side+"Foot").id}));
      const rootQuat=()=>Array.from((data.qpos as Float64Array).slice(3,7));
      const tilt=()=>Math.acos(clamp((data.xmat as Float64Array)[9*root+8]!,-1,1))*180/Math.PI;
      const pyramid=Boolean(mj.mj_isPyramidal(model));
      const trace:WbcFeedbackFrame[]=[],frames:WbcFeedback["frames"]=[];
      let previous=geometry,previousComJ=dyn.comJacobian,previousInterval=.04;
      let previousSoles=initialSoles;
      const snapshot=()=>feet.map(f=>({p:vec(data.geom_xpos as Float64Array,3*f.geom),r:slice9(data.geom_xmat as Float64Array,9*f.geom)}));
      let updates=0,acceptedUpdates=0,executedS=0,stopReason="duration",maxComDisplacementM=0,maxTiltDeg=0,minPelvisM=Infinity;
      let maxTorqueRatio=0,maxContactResidual=0,maxResidual=0,maxNormalPredictionErrorN=0,loadLossS=0,maxBilateralLoadLossS=0,rootForce=0;
      function frame(t:number){
        const x=data.xpos as Float64Array,q=data.xquat as Float64Array;
        frames.push({t,pose:Array.from({length:17},(_,i)=>[...vec(x,3*(i+1)),...Array.from(q.slice(4*(i+1),4*(i+2)))])});
      }
      frame(0);
      for(let step=0;step<Math.round(feedbackS/period);step++){
        const t=step*period;
        mj.mj_forward(model,data);
        const current=geometryOf(data),d=articulatedMass(current),v=Array.from(data.qvel as Float64Array),q=data.qpos as Float64Array;
        if(![...q,...v].every(Number.isFinite)){stopReason="nonfinite";break;}
        const currentSoles=snapshot();
        const b=Array.from(data.qfrc_bias as Float64Array).map((x,i)=>x-(data.qfrc_passive as Float64Array)[i]!);
        const contacts:ContactDynamicsProblem["contacts"][number][]=[],loads=[0,0];
        const list=data.contact,efc=data.efc_force as Float64Array;
        try{
          for(let k=0;k<list.size();k++){
            const ct=list.get(k);if(!ct)throw new Error("Missing WBC contact");
            try{
              const other=ct.geom1===floor?ct.geom2:ct.geom2===floor?ct.geom1:-1;
              const fi=feet.findIndex(f=>f.geom===other);
              if(fi<0||ct.dist>.002)continue;
              const point=vec(ct.pos as Float64Array,0);
              const now=currentSoles[fi]!,old=previousSoles[fi]!;
              const local=[0,1,2].map(j=>[0,1,2].reduce((s,i)=>s+(point[i]!-now.p[i]!)*now.r[3*i+j]!,0));
              const oldPoint:V3=[0,1,2].map(i=>old.p[i]!+local.reduce((s,x,j)=>s+x*old.r[3*i+j]!,0)) as unknown as V3;
              const J=pointJacobian(current,feet[fi]!.body,point).linear;
              const oldJ=pointJacobian(previous,feet[fi]!.body,oldPoint).linear;
              const jdot=J.map((row,i)=>dot(row.map((a,j)=>(a-oldJ[i]![j]!)/previousInterval),v));
              const target=J.map((row,i)=>clamp(-12*dot(row,v)+(i===2?-100*(point[2]+.001):0),-10,10));
              contacts.push({jacobian:J,mu:.25,maxNormalN:d.totalMass*9.81,accelerationBias:jdot,targetAcceleration:target});
              if(hasCompleteSolvedContactRows(ct.efc_address,ct.dim,pyramid,data.nefc,efc.length)){
                const force=decodeSolvedContactForce({solverRows:efc,efcAddress:ct.efc_address,dim:ct.dim,pyramidal:pyramid,friction:ct.friction as Float64Array,adhesionN:ct.adhesion});
                const a=ct.frame as Float64Array;
                const fz=force.normalN*a[2]!+force.tangentN[0]*a[5]!+force.tangentN[1]*a[8]!;
                loads[fi]!+=Math.max(0,ct.geom1===floor?fz:-fz);
              }
            }finally{ct.delete();}
          }
        }finally{list.delete();}
        if(!contacts.length){stopReason="no foot-ground contact";break;}
        const task:LinearRow[]=[];
        const unit=(i:number)=>Array.from({length:n},(_,j)=>i===j?1:0);
        for(const h of hinges){
          const adr=model.jnt_qposadr[h.id]!,rate=v[h.dof]!;
          task.push({a:unit(h.dof),b:clamp(50*(initialQ[adr]!-q[adr]!)-10*rate,-80,80),weight:1});
        }
        // Root orientation target is identity, expressed in root-local
        // angular coordinates. The quaternion's imaginary part provides
        // a shortest-arc small-angle error, with no root actuator.
        const quat=rootQuat(),sign=quat[0]!>=0?1:-1;
        for(let i=0;i<3;i++)task.push({a:unit(3+i),b:clamp(-60*sign*quat[i+1]!-10*v[3+i]!,-40,40),weight:8});
        for(let i=0;i<3;i++){
          const row=d.comJacobian[i]!;
          const convective=dot(row.map((a,j)=>(a-previousComJ[i]![j]!)/previousInterval),v);
          task.push({a:row,b:clamp(30*(com0[i]!-d.com[i]!)-10*dot(row,v),-5,5)-convective,weight:25});
        }
        const s=solveContactDynamics({mass:d.mass,bias:b,rootDofs:6,torqueLimits:limits,contacts,tasks:task});
        updates++;
        const predicted=s.forces.reduce((a,f)=>a+f[2]!,0);
        const accepted=s.feasible&&s.maxTorqueViolation<1e-8&&s.contactAccelerationResidual<=2&&maxAbs(s.acceleration)<=100;
        maxContactResidual=Math.max(maxContactResidual,s.contactAccelerationResidual);
        maxResidual=Math.max(maxResidual,s.maxRootResidual);
        // Current measured load vs next predicted load is a diagnostic,
        // not an exact same-time force error or a hard acceptance gate.
        maxNormalPredictionErrorN=Math.max(maxNormalPredictionErrorN,Math.abs(predicted-loads[0]!-loads[1]!));
        loadLossS=loads.some(l=>l<40)?loadLossS+period:0;
        maxBilateralLoadLossS=Math.max(maxBilateralLoadLossS,loadLossS);
        trace.push({t,com:d.com,pelvis:current.rootPosition,tiltDeg:tilt(),loadsN:loads,predictedNormalN:predicted,
          rootResidual:s.maxRootResidual,contactResidual:s.contactAccelerationResidual,solverMethod:s.solver.method,iterations:s.solver.iterations,accepted});
        if(!accepted){stopReason=!s.feasible?"solver infeasible/unconverged":s.maxTorqueViolation>=1e-8?"torque bound":s.contactAccelerationResidual>2?"contact acceleration task":"acceleration bound";break;}
        applied.fill(0);
        for(let j=6;j<n;j++){
          applied[j]=s.torques[j-6]!;
          maxTorqueRatio=Math.max(maxTorqueRatio,Math.abs(applied[j]!)/limits[j-6]!);
        }
        for(let j=0;j<6;j++)rootForce=Math.max(rootForce,Math.abs(applied[j]!));
        acceptedUpdates++;
        previous=current;previousComJ=d.comJacobian;previousSoles=currentSoles;previousInterval=period;
        for(let sub=0;sub<substeps;sub++){
          mj.mj_step(model,data);
          executedS+=referenceDt;
          if(!(data.qpos as Float64Array).every(Number.isFinite)){stopReason="nonfinite";break;}
        }
        mj.mj_forward(model,data);
        const com=vec(data.subtree_com as Float64Array,3*root);
        maxComDisplacementM=Math.max(maxComDisplacementM,Math.hypot(...com.map((x,i)=>x-com0[i]!)));
        maxTiltDeg=Math.max(maxTiltDeg,tilt());
        minPelvisM=Math.min(minPelvisM,(data.xpos as Float64Array)[3*root+2]!);
        if((step+1)%4===0)frame((step+1)*period);
        if(stopReason!=="duration")break;
        if(minPelvisM<.45||maxTiltDeg>60){stopReason="fall";break;}
      }
      if(frames.at(-1)!.t<executedS-1e-8)frame(executedS);
      return {requestedS:feedbackS,executedS,updates,acceptedUpdates,stopReason,maxComDisplacementM,maxTiltDeg,
        minPelvisM:Number.isFinite(minPelvisM)?minPelvisM:pelvis0[2],maxTorqueRatio,maxRootForce:rootForce,
        maxContactResidual,maxRootResidual:maxResidual,maxNormalPredictionErrorN,maxBilateralLoadLossS,trace,frames,partNames};
    }
    return {nv:n,massKg:dyn.totalMass,gravityError,massEquationError,comError,excitedMassEquationError,rotatedMassEquationError,rotatedGravityError,
      supportPoints:contacts.length,solution,applied:use,
      finalPelvisDisplacementM,finalComDisplacementM,maxRootForce,...(feedback?{feedback}:{})};
  }finally{data.delete();model.delete();}
}
