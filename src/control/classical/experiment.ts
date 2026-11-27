/** Reduced free-root experiment; deliberately separate from production Uke.
 * No ML, world anchor, root torque or position writes after initialization.
 * SIMBICON-inspired sagittal and coronal virtual torso/swing-hip control.
 * Initial gains are hypotheses, not a reproduction of the published gait.
 */
import { advanceGait, initialGait, hipPair, pd, clamp, swingPitchTarget, type Side } from "./simbiconCore";
import { advanceUnilateral, initialUnilateral, unilateralTargets, lateralLoadTarget, kneeLoadCorrection, type UnilateralPhase } from "./unilateral";
import { getMujoco } from "../../scenario/mujocoModule";
import { decodeSolvedContactForce, hasCompleteSolvedContactRows } from "../../physics/contactForces";
import { capturePoint, supportPolygon, signedMargin } from "../../metrics/balance";
import { sha256Hex } from "../../scenario/sha256";
import { RECORDING_SCHEMA_ID, type ContactSample, type Recording, type RecordingSample } from "../../recording/schema";
import type { Vec3 } from "../../scenario/schema";

export type ClassicalMode = "passive" | "stand" | "walk" | "balance" | "shift-left" | "shift-right" | "lift-left" | "lift-right" | "shift2-left" | "shift2-right" | "lift2-left" | "lift2-right";
const DT = 0.002;
const HZ = 500;
const RECORD_HZ = 100;
const SIDES = ["left", "right"] as const;
const AXES = ["hip_pitch", "hip_roll", "knee", "ankle_pitch", "ankle_roll"] as const;
const LIMITS = [130, 100, 110, 75, 65] as const;
const BODY_NAMES = ["world", "pelvis", "left_thigh", "left_shin", "left_foot", "right_thigh", "right_shin", "right_foot"];
const JOINT_NAMES = ["root", ...SIDES.flatMap(s => AXES.map(a => s + "_" + a))];
const leg = (s: Side, y: number): string => `
  <body name="${s}_thigh" pos="0 ${y} 0">
    <joint name="${s}_hip_pitch" axis="0 1 0" range="-1.4 1"/>
    <joint name="${s}_hip_roll" axis="1 0 0" range="-.6 .6"/>
    <geom type="capsule" fromto="0 0 -.04 0 0 -.38" size=".065" mass="8"/>
    <body name="${s}_shin" pos="0 0 -.42">
      <joint name="${s}_knee" axis="0 1 0" range="0 2.1"/>
      <geom type="capsule" fromto="0 0 -.04 0 0 -.38" size=".045" mass="4.5"/>
      <body name="${s}_foot" pos="0 0 -.42">
        <joint name="${s}_ankle_pitch" axis="0 1 0" range="-.7 .7"/>
        <joint name="${s}_ankle_roll" axis="1 0 0" range="-.4 .4"/>
        <geom name="${s}_sole" type="box" pos=".05 0 -.04" size=".13 .055 .035" mass="1"/>
      </body>
    </body>
  </body>`;
export const CLASSICAL_MJCF = `<mujoco model="classical_c0">
  <compiler angle="radian"/>
  <option timestep="${DT}" gravity="0 0 -9.81" cone="elliptic" integrator="implicitfast" iterations="50"/>
  <default><joint damping="1.5" armature=".03"/><geom friction="1 .005 .0001" solref=".015 1" solimp=".9 .95 .001"/></default>
  <worldbody>
    <geom name="floor" type="plane" size="8 8 .1"/>
    <body name="pelvis" pos="0 0 .918">
      <freejoint name="root"/>
      <geom name="trunk" type="capsule" fromto="0 0 .02 0 0 .50" size=".12" mass="48"/>
      ${leg("left", .105)}${leg("right", -.105)}
    </body>
  </worldbody>
</mujoco>`;

export interface LateralFrame {t:number;comY:number;velocityY:number;refY:number;desiredCopY:number;measuredCopY:number|null;loadsN:number[];desiredLoadsN:number[];kneeCorrectionNm:number[]}
export interface ClassicalMetrics {
  mode: ClassicalMode; steps: number; phase: string; firstFallS: number | null;
  finalForwardM: number; minPelvisM: number; maxTiltDeg: number;
  maxRootTorque: number; maxTorqueRatio: number; finalComSpeedMps: number;
  maxHipResidualNm: number; maxLoadedMarkerDriftM: number;
  maxGroundPenetrationM: number; diagnosticsOnly: true;
  unilateral?: { support: Side; phase: UnilateralPhase; reachedShift: boolean; reachedHold: boolean; completed: boolean;
    longestSupportS: number; maxSupportWeightRatio: number; maxSwingClearanceM: number;
    maxComShiftM: number; finalLoadsN: number[]; trace: LateralFrame[]; transitions: Array<{t:number;phase:UnilateralPhase;loadsN:number[];comY:number;clearanceM:number[]}> };
}
export interface ClassicalResult { recording: Recording; metrics: ClassicalMetrics }
const vec = (a: ArrayLike<number>, off: number): Vec3 => [a[off]!, a[off+1]!, a[off+2]!];
const angularDiff = (a: number, b: number): number => Math.atan2(Math.sin(a-b), Math.cos(a-b));

export async function runClassical(mode: ClassicalMode, durationS = 3): Promise<ClassicalResult> {
  if (!["passive", "stand", "walk", "balance", "shift-left", "shift-right", "lift-left", "lift-right", "shift2-left", "shift2-right", "lift2-left", "lift2-right"].includes(mode) || !Number.isFinite(durationS) || durationS <= 0 ||
      durationS > 60 || Math.abs(durationS * RECORD_HZ - Math.round(durationS * RECORD_HZ)) > 1e-8) {
    throw new RangeError("Invalid classical experiment");
  }
  const mj = await getMujoco();
  const model = mj.MjModel.from_xml_string(CLASSICAL_MJCF);
  const data = new mj.MjData(model);
  try {
    if (model.nbody !== BODY_NAMES.length || model.njnt !== JOINT_NAMES.length || model.nv !== 16) {
      throw new Error("Unexpected reduced model topology");
    }
    const qpos = data.qpos as Float64Array, qvel = data.qvel as Float64Array;
    const applied = data.qfrc_applied as Float64Array;
    const ix = SIDES.map(s => AXES.map(a => {
      const id = model.jnt(s+"_"+a).id;
      const dof = model.jnt_dofadr[id]!, q = model.jnt_qposadr[id]!;
      if (dof < 6 || model.jnt_type[id] !== 3) throw new Error("Non-internal hinge");
      return { dof, q };
    }));
    const feet = SIDES.map(s => model.geom(s+"_sole").id);
    const sizes = model.geom_size as Float64Array;
    const bodyOf = model.geom_bodyid as Int32Array;
    const pyramidal = Boolean(mj.mj_isPyramidal(model));
    // Identical initial configuration for all modes; only initialization may set qpos.
    for (const joints of ix) {
      qpos[joints[0]!.q] = -.05; qpos[joints[2]!.q] = .10; qpos[joints[3]!.q] = -.05;
    }
    mj.mj_forward(model, data);
    const referenceCom = vec(data.subtree_com as Float64Array, 3);
    const mass = Array.from(model.body_mass as Float64Array).reduce((a,b)=>a+b,0);
    const unilateralMode=mode.startsWith("shift")||mode.startsWith("lift");
    const allocatedMode=mode.startsWith("shift2-")||mode.startsWith("lift2-");
    const support:Side=mode.endsWith("left")?"left":"right";
    const supportIndex=support==="left"?0:1, swingIndex=1-supportIndex;
    const supportY=(data.geom_xpos as Float64Array)[3*feet[supportIndex]!+1]!;
    let unilateral=initialUnilateral();
    const transitions: NonNullable<ClassicalMetrics["unilateral"]>["transitions"]=[];
    let maxSupportWeightRatio=0,maxSwingClearanceM=0,maxComShiftM=0;
    let finalLoadsN:number[]=[0,0];
    const lateralTrace:LateralFrame[]=[];
    const filteredLoads=[mass*9.81/2,mass*9.81/2];
    let gait = initialGait();
    let firstFallS: number | null = null, minPelvisM = Infinity, maxTiltDeg = 0, maxRootTorque = 0;
    let maxTorqueRatio = 0, maxHipResidualNm = 0, maxLoadedMarkerDriftM = 0, maxGroundPenetrationM = 0;
    let finalComSpeedMps = 0;
    const markerStart: Array<Vec3 | null> = [null, null];
    const samples: RecordingSample[] = [];
    let priorAngles: number[] | null = null;
    const x0 = qpos[0]!;
    const total = Math.round(durationS*HZ);
    for (let k=0; k<=total; k++) {
      const t = k*DT;
      mj.mj_subtreeVel(model, data);
      const com = vec(data.subtree_com as Float64Array, 3);
      const vel = vec(data.subtree_linvel as Float64Array, 3);
      const xmat = data.xmat as Float64Array;
      const gmat = data.geom_xmat as Float64Array;
      const gx = data.geom_xpos as Float64Array;
      const pelvis = vec(data.xpos as Float64Array, 3);
      const angles = [1,2,5].flatMap(b => [Math.atan2(xmat[9*b+2]!, xmat[9*b+8]!),
        Math.atan2(-xmat[9*b+5]!, xmat[9*b+8]!)]);
      const rates = angles.map((a,i)=>priorAngles ? angularDiff(a,priorAngles[i]!)/DT : 0);
      priorAngles = angles;
      const tilt = Math.acos(Math.max(-1,Math.min(1,xmat[17]!)))*180/Math.PI;
      minPelvisM = Math.min(minPelvisM, pelvis[2]); maxTiltDeg = Math.max(maxTiltDeg, tilt);
      finalComSpeedMps = Math.hypot(...vel);
      if (firstFallS === null && (pelvis[2] < .45 || tilt > 60)) firstFallS = t;
      const clear = feet.map(g => gx[3*g+2]! - [0,1,2].reduce((z,j)=>z+Math.abs(gmat[9*g+6+j]!)*sizes[3*g+j]!,0));
      const loads = [0,0];
      const contacts: ContactSample[] = [];
      const list = data.contact, efc = data.efc_force as Float64Array;
      try {
        for (let c=0;c<list.size();c++) {
          const ct = list.get(c);
          if (!ct) throw new Error("Missing contact");
          try {
            const b1 = bodyOf[ct.geom1]!, b2 = bodyOf[ct.geom2]!;
            if (b1===0 || b2===0) {
              maxGroundPenetrationM = Math.max(maxGroundPenetrationM, -ct.dist);
              if (firstFallS === null && (b1===1 || b2===1)) firstFallS = t;
            }
            if (!hasCompleteSolvedContactRows(ct.efc_address,ct.dim,pyramidal,data.nefc,efc.length)) continue;
            const solved = decodeSolvedContactForce({solverRows:efc,efcAddress:ct.efc_address,dim:ct.dim,pyramidal,
              friction:ct.friction as Float64Array,adhesionN:ct.adhesion});
            if (solved.normalN<=0) continue;
            const f = ct.frame as Float64Array, [a,b] = solved.tangentN;
            const force: Vec3 = [solved.normalN*f[0]!+a*f[3]!+b*f[6]!, solved.normalN*f[1]!+a*f[4]!+b*f[7]!, solved.normalN*f[2]!+a*f[5]!+b*f[8]!];
            contacts.push({body_a:BODY_NAMES[b1]!,body_b:BODY_NAMES[b2]!,point_m:vec(ct.pos as Float64Array,0),force_n:force});
            for(let s=0;s<2;s++) if ((b1===0 && ct.geom2===feet[s]) || (b2===0 && ct.geom1===feet[s])) {
              loads[s]! += Math.max(0,b1===0?force[2]:-force[2]);
            }
          } finally { ct.delete(); }
        }
      } finally { list.delete(); }
      if(unilateralMode) {
        finalLoadsN=loads.slice();
        for(let i=0;i<2;i++) filteredLoads[i]! += (loads[i]!-filteredLoads[i]!)*DT/(.04+DT);
        maxComShiftM=Math.max(maxComShiftM,(com[1]-referenceCom[1])*(support==="left"?1:-1));
        if(t>=.5) maxSupportWeightRatio=Math.max(maxSupportWeightRatio,loads[supportIndex]!/(mass*9.81));
        if(unilateral.reachedShift) maxSwingClearanceM=Math.max(maxSwingClearanceM,clear[swingIndex]!);
        if(k>0) {
          const old=unilateral.phase;
          unilateral=advanceUnilateral(unilateral,{comY:com[1],velocityY:vel[1],loadN:[loads[0]!,loads[1]!],
            clearanceM:[clear[0]!,clear[1]!],supportY,bodyWeightN:mass*9.81,fallen:firstFallS!==null},support,mode.startsWith("lift"),DT);
          if(old!==unilateral.phase) transitions.push({t,phase:unilateral.phase,loadsN:loads.slice(),comY:com[1],clearanceM:clear.slice()});
        }
      }
      for(let i=0;i<2;i++) {
        const pos=vec(gx,3*feet[i]!);
        if(loads[i]!>40) {
          const start=markerStart[i]??pos;
          markerStart[i]=start;
          maxLoadedMarkerDriftM=Math.max(maxLoadedMarkerDriftM,Math.hypot(pos[0]-start[0],pos[1]-start[1]));
        } else markerStart[i]=null;
      }
      if (k%(HZ/RECORD_HZ)===0) {
        const cp=capturePoint(com,vel);
        const polygon=supportPolygon(contacts,b=>b==="left_foot"||b==="right_foot");
        samples.push({t,qpos:Array.from(qpos),qvel:Array.from(qvel),contacts,effectors:[],com,
          xcom:[...cp],support_polygon:polygon.map(p=>[...p]),capture_margin_m:signedMargin(cp,polygon)});
      }
      if (k===total) break;
      applied.fill(0);
      if (mode!=="passive" && firstFallS===null) {
        const moving=mode==="walk" && t>=.5 && gait.phase!=="timeout";
        if(moving) gait=advanceGait(gait,{loadN:[loads[0]!,loads[1]!],clearanceM:[clear[0]!,clear[1]!]},DT);
        const walking=moving && gait.phase!=="timeout";
        const s=gait.swing==="left"?0:1, st=1-s;
        const write=(i:number,j:number,value:number):void=>{
          applied[ix[i]![j]!.dof]=value;
          maxTorqueRatio=Math.max(maxTorqueRatio,Math.abs(value)/LIMITS[j]!);
        };
        if (mode==="balance" || unilateralMode) {
          // C0b hypothesis: independent hip posture is not enough. Regulate
          // centroidal acceleration through the feet, not the floating base.
          // LIPM: xddot = g/h (x - p). Desired acceleration = -4 e - 4 v.
          // Approximate desired CoP per loaded foot, respecting its sole bounds.
          // Fixed initial COM reference; no chasing an already drifting COM.
          const h = Math.max(.3, com[2]);
          const pX = com[0] + h/9.81*(4*(com[0]-referenceCom[0])+4*vel[0]);
          const target=unilateralMode?unilateralTargets(unilateral,referenceCom[1],supportY):{refY:referenceCom[1],flexion:0};
          const pY = com[1] + h/9.81*(4*(com[1]-target.refY)+4*vel[1]);
          const loaded = loads.map(n=>n>20);
          const nLoaded=loaded.filter(Boolean).length;
          for(let axis=0;axis<2;axis++) {
            const torso = pd(0,angles[axis]!,rates[axis]!,axis===0?260:240,35,axis===0?180:150);
            // Equal torso reaction in bilateral stance; opposite differential
            // correction keeps thighs symmetric without changing the total.
            const differential = pd(0,angularDiff(angles[2+axis]!,angles[4+axis]!),
              rates[2+axis]!-rates[4+axis]!,100,12,LIMITS[axis]!);
            const swingActive=unilateralMode && target.flexion>0;
            const side=swingActive?swingIndex:0;
            const angleIndex=2+2*side+axis;
            const desired=swingActive?pd(axis===0?-.05-.30*target.flexion:(side===0?.08:-.08)*target.flexion,
              angles[angleIndex]!,rates[angleIndex]!,180,18,LIMITS[axis]!):-torso/2+differential;
            const pair=hipPair(torso,desired,LIMITS[axis]!);
            write(side,axis,pair.swingNm); write(1-side,axis,pair.stanceNm);
            maxHipResidualNm=Math.max(maxHipResidualNm,Math.abs(pair.residualNm));
          }
          // Distribution uses observed support state, nominal weight to avoid
          // feeding high-frequency impact-force noise back into the actuators.
          // For this symmetric reduced model only. Not a general WBC.
          const weights=loaded.map((on,i)=>on?(unilateralMode?Math.max(1,filteredLoads[i]!):1):0);
          const weightSum=weights[0]!+weights[1]!;
          const shares=weights.map(w=>weightSum>0?w/weightSum:0);
          const centerY = nLoaded>0 ? shares[0]!*gx[3*feet[0]!+1]!+shares[1]!*gx[3*feet[1]!+1]! : com[1];
          const centers=[gx[3*feet[0]!+1]!,gx[3*feet[1]!+1]!] as const;
          const desiredShares=lateralLoadTarget(pY,centers,[loaded[0]!,loaded[1]!]);
          const desiredCenterY=nLoaded>0?desiredShares[0]*centers[0]+desiredShares[1]*centers[1]:com[1];
          const correctionNm=[0,0];
          for(let i=0;i<2;i++) {
            const knee=ix[i]![2]!;
            const swingActive=unilateralMode && i===swingIndex && target.flexion>0;
            const kneeTarget=.10+(swingActive?.65*target.flexion:0);
            // Desired normal load is derived from the requested global CoP.
            // Apply the Jacobian-transpose correction at the knee, using the
            // actual world knee axis and lever to the requested pressure point.
            // Compare this policy against the unmodified C2a policy.
            if(allocatedMode && nLoaded===2 && !swingActive) {
              const thighId=i===0?2:5, shinId=thighId+1;
              const anchor=vec(data.xpos as Float64Array,3*shinId);
              const copX=clamp(pX,gx[3*feet[i]!]!-.11,gx[3*feet[i]!]!+.11);
              const copY=centers[i]!+clamp(pY-desiredCenterY,-.045,.045);
              const lever=xmat[9*thighId+1]!*(copY-anchor[1])-xmat[9*thighId+4]!*(copX-anchor[0]);
              correctionNm[i]=kneeLoadCorrection(mass*9.81*desiredShares[i]!,mass*9.81*shares[i]!,lever);
            }
            write(i,2,clamp(pd(kneeTarget,qpos[knee.q]!,qvel[knee.dof]!,260,24,LIMITS[2])+correctionNm[i]!,-LIMITS[2],LIMITS[2]));
            const ankle=vec(data.xpos as Float64Array,3*(i===0?4:7));
            const fx=gx[3*feet[i]!]!, fy=gx[3*feet[i]!+1]!;
            const normal=mass*9.81*shares[i]!;
            const tx=clamp(pX,fx-.11,fx+.11);
            const ty=fy+clamp(pY-(allocatedMode?desiredCenterY:centerY),-.045,.045);
            for(let axis=0;axis<2;axis++) {
              const j=axis+3, joint=ix[i]![j]!;
              const ankleTarget=swingActive?(axis===0?clamp(-angles[2+2*i]!-qpos[knee.q]!,-.65,.65):clamp(-angles[3+2*i]!,-.35,.35)):(axis===0?-.05:0);
              const tau=loaded[i]&&!swingActive ? (axis===0?normal*(tx-ankle[0]):-normal*(ty-ankle[1]))-5*qvel[joint.dof]!
                : pd(ankleTarget,qpos[joint.q]!,qvel[joint.dof]!,180,18,LIMITS[j]!);
              write(i,j,clamp(tau,-LIMITS[j]!,LIMITS[j]!));
            }
          }
          if(unilateralMode && k%100===0) {
            let fz=0, weightedY=0;
            for(const ct of contacts) {
              const footGround=(ct.body_a==="world"&&(ct.body_b==="left_foot"||ct.body_b==="right_foot"))||
                (ct.body_b==="world"&&(ct.body_a==="left_foot"||ct.body_a==="right_foot"));
              if(!footGround) continue;
              const normal=Math.max(0,ct.body_a==="world"?ct.force_n[2]:-ct.force_n[2]);
              fz+=normal; weightedY+=normal*ct.point_m[1];
            }
            lateralTrace.push({t,comY:com[1],velocityY:vel[1],refY:target.refY,desiredCopY:pY,
              measuredCopY:fz>1?weightedY/fz:null,loadsN:loads.slice(),
              desiredLoadsN:desiredShares.map(w=>w*mass*9.81),kneeCorrectionNm:correctionNm});
          }
        } else {
        // One desired torso reaction, distributed over the two hips.
        const torsoPitch=pd(0,angles[0]!,rates[0]!,260,35,180);
        const torsoRoll=pd(walking?(s===0?.08:-.08):0,angles[1]!,rates[1]!,240,30,150);
        const swingBase=walking?(gait.phase==="lift"?-.22:-.30):-.05;
        const swingTarget=walking?swingPitchTarget(swingBase,com[0]-gx[3*feet[st]!]!,vel[0]):swingBase;
        for(let axis=0;axis<2;axis++) {
          const target=axis===0?swingTarget:walking?(s===0?.05:-.05):0;
          const i=2+2*s+axis;
          const desired=pd(target,angles[i]!,rates[i]!,180,18,LIMITS[axis]!);
          const pair=hipPair(axis===0?torsoPitch:torsoRoll,desired,LIMITS[axis]!);
          write(s,axis,pair.swingNm); write(st,axis,pair.stanceNm);
          maxHipResidualNm=Math.max(maxHipResidualNm,Math.abs(pair.residualNm));
        }
        for(let i=0;i<2;i++) {
          const knee=walking && i===s ? (gait.phase==="lift"?.70:.15) : .10;
          const targets=[knee,-knee/2,0];
          for(let j=2;j<5;j++) {
            const joint=ix[i]![j]!;
            write(i,j,pd(targets[j-2]!,qpos[joint.q]!,qvel[joint.dof]!,j===2?260:180,j===2?24:18,LIMITS[j]!));
          }
        }
        }
      }
      for(let j=0;j<6;j++) maxRootTorque=Math.max(maxRootTorque,Math.abs(applied[j]!));
      if(maxRootTorque!==0) throw new Error("Root force invariant");
      mj.mj_step(model,data);
      mj.mj_forward(model,data);
      if(!qpos.every(Number.isFinite)||!qvel.every(Number.isFinite)) throw new Error("Non-finite physics");
    }
    const metrics: ClassicalMetrics={mode,steps:gait.steps,phase:mode==="walk"?gait.phase:mode,firstFallS,
      finalForwardM:qpos[0]!-x0,minPelvisM,maxTiltDeg,maxRootTorque,maxTorqueRatio,finalComSpeedMps,
      maxHipResidualNm,maxLoadedMarkerDriftM,maxGroundPenetrationM,diagnosticsOnly:true};
    if(unilateralMode) metrics.unilateral={support,phase:unilateral.phase,reachedShift:unilateral.reachedShift,
      reachedHold:unilateral.reachedHold,completed:unilateral.completed,longestSupportS:unilateral.longestSupportS,
      maxSupportWeightRatio,maxSwingClearanceM,maxComShiftM,finalLoadsN,transitions,trace:lateralTrace};
    const last=samples[samples.length-1]!;
    // No claim of standing or stepping validation from this diagnostic.
    const recording: Recording={schema:RECORDING_SCHEMA_ID,meta:{scenario_id:"classical_c0_"+mode,
      scenario_sha256:sha256Hex(JSON.stringify({mode,durationS,mjcf:CLASSICAL_MJCF})),engine_version:"classical_c0",
      mujoco_version:mj.mj_versionString(),physics_hz:HZ,record_hz:RECORD_HZ,frame:"x_forward_y_left_z_up",quat_order:"wxyz"},
      model:{mjcf:CLASSICAL_MJCF,bodies:BODY_NAMES,joints:JOINT_NAMES},samples,peaks:{effectors:{}},
      summary:{outcome:firstFallS===null?"unresolved":"fall",direction_deg:null,t_outcome_s:firstFallS??durationS,
        final_tilt_deg:Math.acos(Math.max(-1,Math.min(1,xmatFinal(data))))*180/Math.PI,max_tilt_deg:maxTiltDeg,
        final_com_height_m:last.com![2],slide_m:Math.hypot(last.qpos[0]!-samples[0]!.qpos[0]!,last.qpos[1]!-samples[0]!.qpos[1]!),
        expect:"none",expect_diffs:[]}};
    return {recording,metrics};
  } finally { data.delete(); model.delete(); }
}
function xmatFinal(data: {xmat: unknown}):number { return (data.xmat as Float64Array)[17]!; }
