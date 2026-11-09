import type { XYZ } from "./balanceSensors";
import type { NamedHingeModel } from "./standingBaseline";
import type { JointPDSettings, MuJoCoJointStateView } from "./jointPD";

export interface CartesianJointData extends MuJoCoJointStateView {
  /** Joint axes and anchors transformed into world coordinates by mj_forward/mj_step. */
  readonly xaxis: ArrayLike<number>;
  readonly xanchor: ArrayLike<number>;
}

export interface CartesianFootParams {
  readonly targetWorldM: XYZ;
  readonly currentWorldM: XYZ;
  readonly measuredVelocityWorldMps: XYZ;
  readonly gainsNPerM: XYZ;
  readonly dampingNsPerM: XYZ;
  readonly maxFootForceN: number;
  readonly blend: number;
}

const clamp = (value: number, limit: number): number =>
  Math.max(-limit, Math.min(limit, value));

/** Linear geometric Jacobian of the world position of a foot site with
 * respect to one MuJoCo hinge DOF: d(p_site)/dq = axis_world ×
 * (p_site - anchor_world). No WASM output-buffer copying is needed.
 */
export function hingeSiteJacobianColumn(
  axisWorld: XYZ, anchorWorldM: XYZ, siteWorldM: XYZ,
): XYZ {
  if (![...axisWorld,...anchorWorldM,...siteWorldM].every(Number.isFinite)) {
    throw new RangeError("Jacobiano: coordenadas no finitas");
  }
  const [ax,ay,az] = axisWorld;
  const dx = siteWorldM[0]-anchorWorldM[0];
  const dy = siteWorldM[1]-anchorWorldM[1];
  const dz = siteWorldM[2]-anchorWorldM[2];
  return [ay*dz-az*dy,az*dx-ax*dz,ax*dy-ay*dx];
}

export function desiredFootCartesianForce(params: CartesianFootParams): XYZ {
  const {targetWorldM:goal,currentWorldM:current,
    measuredVelocityWorldMps:velocity,gainsNPerM:kp,
    dampingNsPerM:kd,maxFootForceN:limit,blend} = params;
  if (![...goal,...current,...velocity,...kp,...kd,limit,blend].every(Number.isFinite) ||
    limit <= 0 || blend < 0 || blend > 1 ||
    kp.some(n=>n<0) || kd.some(n=>n<0)) {
    throw new RangeError("Control cartesiano: parámetros no físicos");
  }
  return [0,1,2].map(i =>
    clamp(blend * (kp[i]!*(goal[i]!-current[i]!) -
      kd[i]!*velocity[i]!), limit)) as [number,number,number];
}

/** Impedance in world XYZ, mapped into actual bounded internal-joint
 * torques by virtual work tau=J^T*F. Keep baseline standing PD intact.
 * Physical foot motion still comes ONLY from MuJoCo integration.
 */
export function applyCartesianFootImpedance(
  model: NamedHingeModel, data: CartesianJointData,
  gains: ReadonlyMap<number,JointPDSettings>,
  side: "left"|"right",
  params: CartesianFootParams,
): Readonly<{ forceWorldN: XYZ; maxAbsTorqueNm: number }> {
  const force = desiredFootCartesianForce(params);
  const dofs = [
    "hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll",
  ];
  let maxAbs = 0;
  for (const part of dofs) {
    const name = side+"_"+part;
    const jid = model.jnt(name).id;
    if (jid <= 0 || jid >= model.njnt || model.jnt_type[jid] !== 3) {
      throw new TypeError("Jacobiano aplicado a articulación no interna: "+name);
    }
    const v = model.jnt_dofadr[jid]!;
    const gain = gains.get(jid);
    if (!gain || v < 6) throw new TypeError("Par interno no acotado: "+name);
    const a = jid*3;
    const column = hingeSiteJacobianColumn(
      [data.xaxis[a]!,data.xaxis[a+1]!,data.xaxis[a+2]!],
      [data.xanchor[a]!,data.xanchor[a+1]!,data.xanchor[a+2]!],
      params.currentWorldM,
    );
    const torque = column[0]*force[0]+column[1]*force[1]+column[2]*force[2];
    const clipped = clamp(data.qfrc_applied[v]!+torque,gain.maxTorqueNm);
    if (!Number.isFinite(clipped)) throw new RangeError("Par de pie no finito");
    data.qfrc_applied[v]=clipped;
    maxAbs=Math.max(maxAbs,Math.abs(clipped));
  }
  if (Array.from(data.qfrc_applied).slice(0,6).some(v=>v!==0)) {
    throw new Error("El control cartesiano intentó actuar la pelvis libre");
  }
  return {forceWorldN:force,maxAbsTorqueNm:maxAbs};
}
