/** Classical scalar joint PD: returned torque is in N*m.
 * No learned model, no kinematic teleportation and no world/root force.
 */
export interface JointPDSettings {
  readonly targetRad: number;
  readonly stiffnessNmPerRad: number;
  readonly dampingNmsPerRad: number;
  readonly maxTorqueNm: number;
}

export function limitedJointTorque(
  jointAngleRad: number,
  jointSpeedRadPerSec: number,
  settings: JointPDSettings,
): number {
  const { targetRad, stiffnessNmPerRad, dampingNmsPerRad, maxTorqueNm } = settings;
  if (![jointAngleRad, jointSpeedRadPerSec, targetRad, stiffnessNmPerRad,
    dampingNmsPerRad, maxTorqueNm].every(Number.isFinite)) {
    throw new RangeError("Los parámetros del controlador deben ser finitos");
  }
  if (stiffnessNmPerRad < 0 || dampingNmsPerRad < 0 || maxTorqueNm < 0) {
    throw new RangeError("Las ganancias y límites de par no pueden ser negativos");
  }
  const requested = stiffnessNmPerRad * (targetRad - jointAngleRad)
    - dampingNmsPerRad * jointSpeedRadPerSec;
  if (!Number.isFinite(requested)) throw new RangeError("Par calculado no finito");
  return Math.max(-maxTorqueNm, Math.min(maxTorqueNm, requested));
}

export interface MuJoCoJointIndexView {
  readonly njnt: number;
  readonly jnt_type: ArrayLike<number>;
  readonly jnt_qposadr: ArrayLike<number>;
  readonly jnt_dofadr: ArrayLike<number>;
}

export interface MuJoCoJointStateView {
  readonly qpos: ArrayLike<number>;
  readonly qvel: ArrayLike<number>;
  /** MuJoCo's generalized force vector, in N*m on hinge DoFs. */
  readonly qfrc_applied: Float64Array;
}

/** Set torque only for explicitly selected HINGE joints (mjtJoint type 3).
 * A free pelvis has type 0 and can NEVER be driven through this API.
 * Root forces and external contact forces are intentionally untouched.
 * Call this again every physics tick; a higher-level controller will supply
 * time-varying targets after the sensor loop (#8–#9).
 */
export function applyHingePD(
  model: MuJoCoJointIndexView,
  data: MuJoCoJointStateView,
  settingsByJointIndex: ReadonlyMap<number, JointPDSettings>,
): number {
  let applied = 0;
  for (const [jointIndex, gains] of settingsByJointIndex) {
    if (!Number.isSafeInteger(jointIndex) || jointIndex < 0 || jointIndex >= model.njnt) {
      throw new RangeError("Índice de articulación fuera del modelo");
    }
    if (model.jnt_type[jointIndex] !== 3) {
      throw new TypeError("Solo se permiten bisagras internas; root/freejoint no admite motor PD");
    }
    const q = model.jnt_qposadr[jointIndex]!;
    const v = model.jnt_dofadr[jointIndex]!;
    if (!Number.isSafeInteger(q) || !Number.isSafeInteger(v) ||
        q < 0 || q >= data.qpos.length ||
        v < 0 || v >= data.qvel.length || v >= data.qfrc_applied.length) {
      throw new RangeError("Dirección de estado de articulación inválida");
    }
    data.qfrc_applied[v] = limitedJointTorque(data.qpos[q]!, data.qvel[v]!, gains);
    applied++;
  }
  return applied;
}
