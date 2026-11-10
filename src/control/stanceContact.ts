import type { XY, XYZ } from "./balanceSensors";
import { hingeSiteJacobianColumn, type CartesianJointData } from "./cartesianFootImpedance";
import type { JointPDSettings } from "./jointPD";
import type { NamedHingeModel } from "./standingBaseline";
import type { StepPhase } from "./experimentalStep";
import type { FootSupport } from "./footContacts";

/** An opt-in research controller. The requested virtual foot force is NOT
 * the measured ground reaction. Only the MuJoCo contact solver determines
 * whether the rear foot actually propels the body or slips.
 */
export interface StanceContactCommand {
  readonly virtualForceWorldN: XYZ;
  readonly intendedPushOffN: number;
  readonly reactionForwardN: number;
  readonly materialSlipSpeedMps: number;
  /** Conservative proxy, NOT a guaranteed dynamic friction-cone margin. */
  readonly forceHeadroomN: number;
}

/** Use a contact material-point velocity: a rigid sole marker is allowed to
 * move as the foot rolls about a stationary heel/toe contact.
 *
 * The virtual force tries to move the supporting foot BACKWARD during the
 * latter part of forward swing, so the physical floor reaction may be forward.
 * During RECOVER, both loaded feet receive only slip damping (no push-off).
 * Feedback opposes material slip in either tangent direction. Both effects
 * are bounded by measured normal load / available friction and later by
 * actual joint torque limits. Do not call this a verified push-off unless
 * the measured ground reaction is positive along the gait direction.
 */
export function planStanceContactCommand(input: {
  readonly phase: StepPhase;
  readonly requestedStepXY: XY;
  readonly comWorldM: XYZ;
  readonly contactWorldM: XYZ;
  readonly foot: FootSupport;
  readonly materialSlipXYMps: XY;
  /** 0 preserves the old controller exactly; opt-in experimental strength. */
  readonly strength: number;
}): StanceContactCommand | null {
  const {phase,requestedStepXY,comWorldM,contactWorldM,foot,
    materialSlipXYMps:slip,strength}=input;
  const friction=foot.effectiveSlideFrictionMin;
  const utilization=foot.frictionUtilizationPeak;
  const raw=[...requestedStepXY,...comWorldM,...contactWorldM,...slip,
    foot.verticalLoadN,...foot.tangentialReactionXYN,strength];
  if (raw.some(x=>!Number.isFinite(x)) || strength<0 || strength>1 ||
      (friction!==null && (!Number.isFinite(friction) || friction<0)) ||
      (utilization!==null && (!Number.isFinite(utilization) || utilization<0))) {
    throw new RangeError("Medida de apoyo o intensidad de contacto inválida");
  }
  if (strength===0 || !["LIFT","SWING","LAND","RECOVER"].includes(phase) ||
      foot.verticalLoadN<40 || !foot.verticalLoadCentroidWorldM ||
      friction===null || utilization===null || friction<=0) return null;

  const frictionCapacity=friction*foot.verticalLoadN;
  // The cone is evaluated per contact in MuJoCo: this aggregate reserve is
  // only a conservative gating heuristic. Never override contact dynamics.
  const headroom=Math.max(0,Math.min(35,
    (0.8-utilization)*frictionCapacity,
    0.85*frictionCapacity-Math.hypot(...foot.tangentialReactionXYN)));
  if (headroom<0.1) return null;

  // +X is forward. The rear support foot provides propulsion only if the
  // trunk COM has already progressed ahead of that measured contact and
  // the opposite leg is in swing. Backward / lateral steps do not request
  // forward propulsion; the contact damping can still suppress slip.
  const forward=requestedStepXY[0]>0.03 && Math.abs(requestedStepXY[1])<0.20;
  const comLeadX=comWorldM[0]-contactWorldM[0];
  const pushRamp=forward && (phase==="SWING" || phase==="LAND") ?
    Math.max(0,Math.min(1,(comLeadX-0.015)/0.045)) : 0;
  const pushOffN=12*strength*pushRamp;
  const damping=55*strength;
  const desiredX=-pushOffN-damping*slip[0];
  const desiredY=-damping*slip[1];
  const requestedMagnitude=Math.hypot(desiredX,desiredY);
  if(requestedMagnitude<1e-9) return null;
  const scale=Math.min(1,headroom/requestedMagnitude);
  const normalizedStepMagnitude=Math.hypot(...requestedStepXY);
  const reactionForwardN=normalizedStepMagnitude>0 ?
    (foot.tangentialReactionXYN[0]*requestedStepXY[0]+
      foot.tangentialReactionXYN[1]*requestedStepXY[1])/normalizedStepMagnitude : 0;
  return {
    virtualForceWorldN:[desiredX*scale,desiredY*scale,0],
    intendedPushOffN:pushOffN*scale,
    reactionForwardN,
    materialSlipSpeedMps:Math.hypot(...slip),
    forceHeadroomN:headroom,
  };
}

/** Map the experimental contact-point force into *internal* hip/knee/ankle
 * torques using virtual work (J^T F). No welded support, external body
 * push, free-root force, or collision manipulation.
 */
export function applyStanceContactCommand(
  model: NamedHingeModel,
  data: CartesianJointData,
  gains: ReadonlyMap<number, JointPDSettings>,
  side: "left"|"right",
  contactWorldM: XYZ,
  virtualForceWorldN: XYZ,
): void {
  if (![...contactWorldM,...virtualForceWorldN].every(Number.isFinite)) {
    throw new RangeError("Fuerza o contacto de apoyo no finitos");
  }
  for (const part of ["hip_pitch","hip_roll","knee","ankle_pitch","ankle_roll"]) {
    const joint=model.jnt(side+"_"+part).id;
    if(joint<=0 || joint>=model.njnt || model.jnt_type[joint]!==3) {
      throw new TypeError("Control de apoyo sobre articulación no interna");
    }
    const dof=model.jnt_dofadr[joint]!;
    const gain=gains.get(joint);
    if(!gain || dof<6) throw new TypeError("Apoyo sin límite de par interno");
    const i=joint*3;
    const j=hingeSiteJacobianColumn(
      [data.xaxis[i]!,data.xaxis[i+1]!,data.xaxis[i+2]!],
      [data.xanchor[i]!,data.xanchor[i+1]!,data.xanchor[i+2]!],
      contactWorldM,
    );
    const torque=j[0]*virtualForceWorldN[0]+
      j[1]*virtualForceWorldN[1]+j[2]*virtualForceWorldN[2];
    const candidate=data.qfrc_applied[dof]!+torque;
    data.qfrc_applied[dof]=Math.max(-gain.maxTorqueNm,
      Math.min(gain.maxTorqueNm,candidate));
  }
  if (Array.from(data.qfrc_applied).slice(0,6).some(n=>n!==0)) {
    throw new Error("Control de apoyo aplicó fuerza al root libre");
  }
}
