/**
 * Independent planar SIMBICON reference controller, based on the published
 * FSM/PD/COM feedback and numerical settings of Michael Firmin's MIT demo
 * (SimbiconJS commit ddcb94844d2722e117aa2f07569d0fa356a72776).
 * No Coach implementation copied. This is a MuJoCo adaptation, not the
 * unchanged upstream demo. Planar constraint is intentional.
 */
import { clamp } from "./simbiconCore";
import { referenceJoints,referenceTorqueLimit } from "./planarReferenceModel";
export type Leg = "l"|"r";
export type PlanarPhase=0|1|2|3;
export interface PlanarState {
  phase:PlanarPhase; elapsed:number;
  torsoPrevious:Partial<Record<Leg,number>>;
  swingPrevious:Partial<Record<Leg,number>>;
}
export interface PlanarObservation {
  angle:Record<string,number>;rate:Record<string,number>;
  torsoPitch:number; thighPitch:Record<Leg,number>; ankleX:Record<Leg,number>;
  comX:number;comVelocityX:number;contact:Record<Leg,boolean>;
}
export const initialPlanarState=():PlanarState=>({phase:0,elapsed:0,torsoPrevious:{},swingPrevious:{}});
export const swingSide=(phase:PlanarPhase):Leg=>phase<2?"r":"l";
export const opposite=(side:Leg):Leg=>side==="l"?"r":"l";
const pd=(goal:number,current:number,rate:number,kp=300,kd=30):number=>kp*(goal-current)-kd*rate;
const wrap=(angle:number):number=>Math.atan2(Math.sin(angle),Math.cos(angle));
/** Commands calculated after a physical step for the NEXT step, preserving
 * the reference simulator's one-step actuation delay. The upstream controller
 * updates four states by timed swing and contact; no extra gait gate here.
 * Independent evaluation records genuine clearance/contact transitions.
 */
export function planarCommand(prev:PlanarState,o:PlanarObservation,dt:number):{state:PlanarState;torques:Record<string,number>} {
  if(!(dt>0)||!Number.isFinite(dt))throw new RangeError("Invalid control dt");
  const swing=swingSide(prev.phase),stance=opposite(swing),lifting=prev.phase%2===0;
  let elapsed=prev.elapsed+dt;
  const swingGoal=lifting?-.4-.2*o.comVelocityX:.7-2.2*(o.comX-o.ankleX[swing]);
  // The world-frame torso and thigh feedback is expressed in HPL +Y pitch.
  // Stored previous angles are per VPD channel, as in the reference; this
  // intentionally retains the reactivation transient instead of retuning it.
  const torsoRate=prev.torsoPrevious[stance]===undefined?0:wrap(o.torsoPitch-prev.torsoPrevious[stance]!)/dt;
  const swingRate=prev.swingPrevious[swing]===undefined?0:wrap(o.thighPitch[swing]-prev.swingPrevious[swing]!)/dt;
  const torsoReaction=300*o.torsoPitch+30*torsoRate;
  const swingTorque=pd(swingGoal,o.thighPitch[swing],swingRate);
  const target:Record<string,number>={
    [swing+"Knee"]:lifting?1.1:.05,
    [stance+"Knee"]:lifting?.05:.10,
    lAnkle:-.2,rAnkle:-.2,
    lShoulder:swing==="r"?-.3:.3,rShoulder:swing==="r"?.3:-.3,
    lElbow:0,rElbow:-.4,
  };
  const torque:Record<string,number>={};
  for(const joint of referenceJoints){
    const n=joint.name;
    if(n==="lHip"||n==="rHip")continue;
    const trunk=n==="waist"||n==="lTorso2uTorso";
    const wrist=n.endsWith("Wrist");
    const arm=n.endsWith("Shoulder")||n.endsWith("Elbow");
    // Upstream sets the left elbow's gains only once it becomes the swing
    // arm's counterpart in phase 2; retain the initial default for phases 0/1
    // before that event. (Its target remains zero.)
    const kp=trunk?600:wrist?5:arm?100:300;
    const kd=trunk?60:wrist?3:arm?30:30;
    torque[n]=pd(target[n]??0,o.angle[n]!,o.rate[n]!,kp,kd);
  }
  torque[swing+"Hip"]=swingTorque;
  torque[stance+"Hip"]=torsoReaction-swingTorque;
  for(const n of Object.keys(torque)){
    if(!Number.isFinite(torque[n]))throw new Error("Nonfinite planar command "+n);
    torque[n]=clamp(torque[n]!,-referenceTorqueLimit,referenceTorqueLimit);
  }
  let phase=prev.phase;
  if((lifting&&elapsed>.3)||(!lifting&&o.contact[swing])){
    phase=((phase+1)%4) as PlanarPhase;elapsed=0;
  }
  return {state:{phase,elapsed,torsoPrevious:{...prev.torsoPrevious,[stance]:o.torsoPitch},
    swingPrevious:{...prev.swingPrevious,[swing]:o.thighPitch[swing]}},torques:torque};
}
