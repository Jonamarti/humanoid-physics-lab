/** Run the planar reference adaptation with native MuJoCo dynamics.
 * Read-only observations and internal hinge torques; no root assistance.
 * Planar reference has three root DoFs. The free3d diagnostic removes
 * that constraint; free3d-roll adds four coronal actuators.
 * Contact/inertia differences from Bullet are explicit in planarModel.
 */
import { getMujoco } from "../../scenario/mujocoModule";
import { planarModel,referenceDt,referenceInitialPosition,referenceJoints,referenceParts,referenceTorqueLimit,type ReferenceSpace } from "./planarReferenceModel";
import { initialPlanarState,planarCommand,swingSide,type Leg,type PlanarPhase } from "./planarReferenceControl";
import { LandingQualityAudit,materialPointTravel,type LandingEvent,type SoleTransform,type FootIndex } from "./contactQuality";
import { coronalCommand,coronalJointNames,hipLimitNm,ankleLimitNm,type CoronalPolicy,type CoronalCommand } from "./coronalControl";
import { decodeSolvedContactForce,hasCompleteSolvedContactRows } from "../../physics/contactForces";
type V3=[number,number,number];
const vec=(a:ArrayLike<number>,i:number):V3=>[a[i]!,a[i+1]!,a[i+2]!];
const round=(v:number):number=>Math.round(v*1e6)/1e6;
const delta=(a:number,b:number):number=>Math.atan2(Math.sin(a-b),Math.cos(a-b));
export interface PlanarRun {
  dt:number;durationS:number;massKg:number;firstFallS:number|null;nonfinite:string|null;
  space:ReferenceSpace;maxLateralM:number;maxRollDeg:number;coronalPolicy?:CoronalPolicy;
  coronalTrace:Array<{t:number;comY:number;velocityY:number;roll:number;loadsN:number[];actualCopY:number|null;command:CoronalCommand}>;
  loadedLandings:number;longestLoadedConsecutive:number;landingAudit:LandingEvent[];
  maxLoadedMaterialTravelM:number;
  forwardM:number;qualifiedLandings:number;longestConsecutive:number;maxTiltDeg:number;maxRootForce:number;
  maxJointTorqueNm:Record<string,number>;maxInitialPositionErrorM:number;
  maxLoadedMarkerDriftM:number;maxGroundPenetrationM:number;twentyLandings:boolean;
  transitions:Array<{t:number;from:PlanarPhase;to:PlanarPhase;side:Leg;clearanceM:number;loadN:number;qualified:boolean}>;
  partNames:string[];frames:Array<{t:number;phase:PlanarPhase;pose:number[][];clearance:number[];loadsN:number[]}>;
}
export async function runPlanarReference(dt=referenceDt,durationS=12,passive=false,space:ReferenceSpace="planar",policy:CoronalPolicy="capture"):Promise<PlanarRun> {
  const hz=25,every=Math.round(1/(dt*hz)),total=Math.round(durationS/dt);
  if(!(dt>0)||!(durationS>0)||durationS>12||!Number.isFinite(dt+durationS)||
    Math.abs(every*dt*hz-1)>1e-8||Math.abs(total*dt-durationS)>1e-8)throw new RangeError("Invalid run settings");
  const mj=await getMujoco(),model=mj.MjModel.from_xml_string(planarModel(dt,space)),data=new mj.MjData(model);
  try{
    const rootDofs=space==="planar"?3:6;
    const coronal=space==="free3d-roll",extra=coronal?4:0;
    if(model.nv!==rootDofs+16+extra||model.nq!==(space==="planar"?19:23)+extra||model.nbody!==18||model.njnt!==(space==="planar"?19:17)+extra)throw new Error("Unexpected reference model topology");
    const qpos=data.qpos as Float64Array,qvel=data.qvel as Float64Array,applied=data.qfrc_applied as Float64Array;
    const ids=referenceJoints.map(j=>{const id=model.jnt(j.name).id;return {name:j.name,id,
      q:model.jnt_qposadr[id]!,dof:model.jnt_dofadr[id]!};});
    const lateralIds=coronal?coronalJointNames.map(name=>{const id=model.jnt(name).id;return {name,id,q:model.jnt_qposadr[id]!,dof:model.jnt_dofadr[id]!};}):[];
    const internal=[...ids,...lateralIds];
    if(internal.some(j=>j.dof<rootDofs)||new Set(internal.map(j=>j.dof)).size!==16+extra)throw new Error("Invalid internal hinge mapping");
    const parts=referenceParts.map(p=>({name:p.name,id:model.body(p.name).id}));
    const bodyId=(name:string):number=>parts.find(p=>p.name===name)!.id;
    const footIds=[model.geom("lFoot_geom").id,model.geom("rFoot_geom").id];
    const floorId=model.geom("floor").id,ankleIds=["lAnkle","rAnkle"].map(n=>ids.find(j=>j.name===n)!.id);
    const sizes=model.geom_size as Float64Array;
    const massKg=Array.from(model.body_mass as Float64Array).reduce((a,b)=>a+b,0);
    mj.mj_forward(model,data);
    const pos=data.xpos as Float64Array,quat=data.xquat as Float64Array,mat=data.xmat as Float64Array;
    let maxInitialPositionErrorM=0;
    for(const p of parts){
      const initial=referenceInitialPosition(p.name);
      const actual=vec(pos,3*p.id);
      maxInitialPositionErrorM=Math.max(maxInitialPositionErrorM,Math.hypot(...actual.map((x,i)=>x-initial[i]!)));
    }
    const rootId=bodyId("pelvis"),torsoId=bodyId("uTorso");
    const pelvisX0=pos[3*rootId]!,pelvisY0=pos[3*rootId+1]!;
    const pitch=(id:number):number=>Math.atan2(mat[9*id+2]!,mat[9*id+8]!);
    let state=initialPlanarState(),previousAngles=Object.fromEntries(ids.map(j=>[j.name,0]));
    let previousComX=(data.subtree_com as Float64Array)[3*rootId]!;
    let previousComY=(data.subtree_com as Float64Array)[3*rootId+1]!,previousRoll=0;
    const filteredLoads=[massKg*9.81/2,massKg*9.81/2];
    const coronalTrace:PlanarRun["coronalTrace"]=[];
    let firstFallS:number|null=null,nonfinite:string|null=null,qualifiedLandings=0,longestConsecutive=0,streak=0,flight=false;
    let maxTiltDeg=0,maxRootForce=0,maxLoadedMarkerDriftM=0,maxGroundPenetrationM=0,maxLateralM=0,maxRollDeg=0,maxLoadedMaterialTravelM=0;
    const quality=new LandingQualityAudit(dt);
    const previousSole:Array<SoleTransform|null>=[null,null],previousLoad=[0,0],materialTravel=[0,0];
    const peak:Record<string,number>={},marker:Array<V3|null>=[null,null];
    const frames:PlanarRun["frames"]=[],transitions:PlanarRun["transitions"]=[];
    const pyramid=Boolean(mj.mj_isPyramidal(model));
    for(let k=0;k<=total;k++){
      const t=k*dt;
      if(!qpos.every(Number.isFinite)||!qvel.every(Number.isFinite)){nonfinite="Nonfinite MuJoCo state at "+t;break;}
      const gx=data.geom_xpos as Float64Array,gm=data.geom_xmat as Float64Array;
      const clearance=footIds.map(g=>gx[3*g+2]!-[0,1,2].reduce((z,j)=>z+Math.abs(gm[9*g+6+j]!)*sizes[3*g+j]!,0));
      const loads=[0,0],touch=[false,false],travelWeighted=[0,0];
      let pressureY=0;
      const soles=footIds.map(g=>({position:vec(gx,3*g),rotation:Array.from(gm.slice(9*g,9*g+9))}));
      const list=data.contact,efc=data.efc_force as Float64Array;
      try{
        for(let c=0;c<list.size();c++){
          const ct=list.get(c);if(!ct)throw new Error("Missing contact");
          try{
            const other=ct.geom1===floorId?ct.geom2:ct.geom2===floorId?ct.geom1:-1;
            if(other<0)continue;
            maxGroundPenetrationM=Math.max(maxGroundPenetrationM,-ct.dist);
            const i=footIds.indexOf(other);
            if(i<0)continue;
            if(ct.dist<=.001)touch[i]=true;
            if(!hasCompleteSolvedContactRows(ct.efc_address,ct.dim,pyramid,data.nefc,efc.length))continue;
            const solved=decodeSolvedContactForce({solverRows:efc,efcAddress:ct.efc_address,dim:ct.dim,pyramidal:pyramid,friction:ct.friction as Float64Array,adhesionN:ct.adhesion});
            const f=ct.frame as Float64Array,[a,b]=solved.tangentN;
            const vertical=solved.normalN*f[2]!+a*f[5]!+b*f[8]!;
            const load=Math.max(0,ct.geom1===floorId?vertical:-vertical);
            loads[i]!+=load;
            if(coronal)pressureY+=load*ct.pos[1]!;
            const prev=previousSole[i];
            if(load>0&&prev&&previousLoad[i]!>=40){
              const p=ct.pos as Float64Array;
              travelWeighted[i]!+=load*materialPointTravel(prev,soles[i]!,vec(p,0));
            }
          }finally{ct.delete();}
        }
      }finally{list.delete();}
      for(let i=0;i<2;i++){
        const point=vec(gx,3*footIds[i]!);
        if(loads[i]!>40){const p=marker[i]??point;marker[i]=p;maxLoadedMarkerDriftM=Math.max(maxLoadedMarkerDriftM,Math.hypot(point[0]-p[0],point[1]-p[1]));}
        else marker[i]=null;
        if(loads[i]!>=40&&previousLoad[i]!>=40){
          materialTravel[i]!+=travelWeighted[i]!/loads[i]!;
          maxLoadedMaterialTravelM=Math.max(maxLoadedMaterialTravelM,materialTravel[i]!);
        }else materialTravel[i]=0;
        previousSole[i]=soles[i]!;previousLoad[i]=loads[i]!;
      }
      const torsoPitch=pitch(torsoId);
      const tilt=space==="planar"?Math.abs(torsoPitch):Math.acos(Math.max(-1,Math.min(1,mat[9*torsoId+8]!)));
      const roll=space==="planar"?0:Math.atan2(-mat[9*torsoId+5]!,mat[9*torsoId+8]!);
      maxTiltDeg=Math.max(maxTiltDeg,tilt*180/Math.PI);
      maxRollDeg=Math.max(maxRollDeg,Math.abs(roll)*180/Math.PI);
      maxLateralM=Math.max(maxLateralM,Math.abs(pos[3*rootId+1]!-pelvisY0));
      if(firstFallS===null&&(pos[3*rootId+2]!<.45||tilt>Math.PI/3))firstFallS=t;
      if(k>0&&!passive){
        const angle=Object.fromEntries(ids.map(j=>[j.name,qpos[j.q]!]));
        const rate=Object.fromEntries(ids.map(j=>[j.name,delta(angle[j.name]!,previousAngles[j.name]!)/dt]));
        const comX=(data.subtree_com as Float64Array)[3*rootId]!;
        const anchors=data.xanchor as Float64Array;
        const old=state.phase,side=swingSide(old),i:FootIndex=side==="l"?0:1;
        quality.observe(t,[loads[0]!,loads[1]!],[clearance[0]!,clearance[1]!],i,firstFallS!==null);
        if(clearance[i]!>.01)flight=true;
        const command=planarCommand(state,{angle,rate,torsoPitch,
          thighPitch:{l:pitch(bodyId("lThigh")),r:pitch(bodyId("rThigh"))},
          ankleX:{l:anchors[3*ankleIds[0]!]!,r:anchors[3*ankleIds[1]!]!},
          comX,comVelocityX:(comX-previousComX)/dt,contact:{l:touch[0]!,r:touch[1]!}},dt);
        state=command.state;
        if(old!==state.phase){
          const landing=old%2===1;
          const accepted=landing&&flight&&clearance[i]!<.02&&firstFallS===null;
          if(landing){quality.touchdown(t,i,accepted);if(accepted){qualifiedLandings++;streak++;longestConsecutive=Math.max(streak,longestConsecutive);}else streak=0;flight=false;}
          transitions.push({t:round(t),from:old,to:state.phase,side,clearanceM:round(clearance[i]!),loadN:round(loads[i]!),qualified:accepted});
        }
        // Prepare the next step exactly once. Initial step is passive, as
        // in the upstream simulator's callback-after-step scheduling.
        applied.fill(0);
        for(const j of ids){const tau=command.torques[j.name]!;applied[j.dof]=tau;peak[j.name]=Math.max(peak[j.name]??0,Math.abs(tau));}
        if(coronal){
          const com=data.subtree_com as Float64Array;
          const comY=com[3*rootId+1]!,comZ=com[3*rootId+2]!;
          const vy=(comY-previousComY)/dt;
          for(let l=0;l<2;l++)filteredLoads[l]!+=(loads[l]!-filteredLoads[l]!)*dt/(.025+dt);
          const ax=data.xaxis as Float64Array;
          const jac=(l:number):number=>{
            const hip=lateralIds[l]!.id,ank=ankleIds[l]!;
            const dx=anchors[3*ank]!-anchors[3*hip]!,dz=anchors[3*ank+2]!-anchors[3*hip+2]!;
            return ax[3*hip+2]!*dx-ax[3*hip]!*dz;
          };
          const footRoll=(l:number):number=>{
            const g=footIds[l]!;
            return Math.atan2(-gm[9*g+5]!,gm[9*g+8]!);
          };
          const q=(j:number):number=>qpos[lateralIds[j]!.q]!;
          const v=(j:number):number=>qvel[lateralIds[j]!.dof]!;
          const sw:FootIndex=swingSide(state.phase)==="l"?0:1;
          const lateral=coronalCommand(policy,{swing:sw,phaseElapsedS:state.elapsed,lifting:state.phase%2===0,
            comY,comZ,velocityY:vy,torsoRoll:roll,torsoRollRate:delta(roll,previousRoll)/dt,
            hipQ:[q(0),q(1)],hipRate:[v(0),v(1)],ankleQ:[q(2),q(3)],ankleRate:[v(2),v(3)],
            ankleY:[anchors[3*ankleIds[0]!+1]!,anchors[3*ankleIds[1]!+1]!],hipToAnkleJacobianY:[jac(0),jac(1)],
            footRoll:[footRoll(0),footRoll(1)],soleY:[gx[3*footIds[0]!+1]!,gx[3*footIds[1]!+1]!],
            loadN:[loads[0]!,loads[1]!],filteredLoadN:[filteredLoads[0]!,filteredLoads[1]!]});
          const commands=[...lateral.hipNm,...lateral.ankleNm];
          for(let j=0;j<4;j++){
            const joint=lateralIds[j]!,tau=commands[j]!,limit=j<2?hipLimitNm:ankleLimitNm;
            if(!Number.isFinite(tau)||Math.abs(tau)>limit+1e-8)throw new Error("Coronal torque invariant");
            applied[joint.dof]=tau;peak[joint.name]=Math.max(peak[joint.name]??0,Math.abs(tau));
          }
          if(k%every===0)coronalTrace.push({t:round(t),comY,velocityY:vy,roll,loadsN:[...loads],
            actualCopY:loads[0]!+loads[1]!>1?pressureY/(loads[0]!+loads[1]!):null,command:lateral});
          previousComY=comY;previousRoll=roll;
        }
        previousAngles=angle;previousComX=comX;
      }
      if(k%every===0||k===total){
        frames.push({t:round(t),phase:state.phase,pose:parts.map(p=>[...vec(pos,3*p.id),...Array.from(quat.slice(4*p.id,4*p.id+4))]),clearance,loadsN:loads});
      }
      for(let d=0;d<rootDofs;d++)maxRootForce=Math.max(maxRootForce,Math.abs(applied[d]!));
      if(maxRootForce!==0||Object.values(peak).some(v=>v>referenceTorqueLimit+1e-8))throw new Error("Internal torque invariant");
      if(k===total||firstFallS!==null&&t>=firstFallS+.8)break;
      mj.mj_step(model,data);
      mj.mj_forward(model,data);
    }
    quality.finish();
    const last=frames.at(-1);
    return {dt,durationS:last?.t??0,massKg,firstFallS,nonfinite,space,maxLateralM,maxRollDeg,
      ...(coronal?{coronalPolicy:policy}:{}),coronalTrace,
      loadedLandings:quality.accepted,longestLoadedConsecutive:quality.longestConsecutive,landingAudit:quality.events,maxLoadedMaterialTravelM,forwardM:last?last.pose[parts.findIndex(p=>p.name==="pelvis")]![0]!-pelvisX0:0,
      qualifiedLandings,longestConsecutive,maxTiltDeg,maxRootForce,maxJointTorqueNm:peak,maxInitialPositionErrorM,
      maxLoadedMarkerDriftM,maxGroundPenetrationM,twentyLandings:longestConsecutive>=20&&firstFallS===null&&nonfinite===null,
      transitions,partNames:parts.map(p=>p.name),frames};
  }finally{data.delete();model.delete();}
}
