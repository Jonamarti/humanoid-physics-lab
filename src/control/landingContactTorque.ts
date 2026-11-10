/** Experimental contact-aware torque relief. It does not add ground forces,
 * change MuJoCo friction, modify qpos, or control the free pelvis root.
 * Only reduces swing hip torques when *measured* foot-ground contact is
 * loaded AND near the solver's friction budget, AND the sole is sliding.
 */
export interface ContactHipReliefInput {
  readonly phase: string;
  readonly normalForceN: number;
  readonly frictionUtilization: number | null;
  readonly soleTangentialSpeedMps: number;
  /** 0 = disabled; 1 = allow up to 75% hip-torque reduction. */
  readonly relief: number;
}

/** Computed at 50 Hz using the previously solved contact impulse.
 * This is a heuristic actuator policy, not a friction-cone constraint.
 */
export function contactHipTorqueScale(input: ContactHipReliefInput): number {
  const {phase,normalForceN,frictionUtilization,soleTangentialSpeedMps,relief}=input;
  if (![normalForceN,soleTangentialSpeedMps,relief].every(Number.isFinite) ||
      normalForceN < 0 || soleTangentialSpeedMps < 0 || relief < 0 || relief > 1 ||
      (frictionUtilization !== null &&
      (!Number.isFinite(frictionUtilization) || frictionUtilization < 0))) {
    throw new RangeError("Contacto/fricción o alivio de pares inválidos");
  }
  if (!["LAND","RECOVER","COMPLETE"].includes(phase) ||
      normalForceN < 40 || frictionUtilization === null || relief === 0 ||
      soleTangentialSpeedMps <= 0.035) return 1;
  const frictionRisk = Math.max(0,Math.min(1,
    (frictionUtilization-0.45)/0.45));
  const speedRisk = Math.max(0,Math.min(1,
    (soleTangentialSpeedMps-0.035)/0.18));
  return 1-0.75*relief*frictionRisk*speedRisk;
}

/** Saturated current actuator torques have already been computed by the
 * nominal controller. Scale ONLY the moving leg's internal 3 hip hinges;
 * leave the knee/ankle's load-bearing control and root DOFs untouched.
 */
export function attenuateSwingHipTorques(
  model: {
    readonly njnt: number;
    readonly jnt_dofadr: ArrayLike<number>;
    readonly jnt_type: ArrayLike<number>;
    jnt(name:string): { readonly id:number };
  },
  appliedForces: Float64Array,
  side:"left"|"right",
  scale:number,
): void {
  if (!Number.isFinite(scale) || scale < 0.25 || scale > 1) {
    throw new RangeError("Reducción de par fuera del rango admitido");
  }
  for(const name of ["hip_pitch","hip_roll","hip_yaw"]) {
    const id=model.jnt(side+"_"+name).id;
    if(id<=0 || id>=model.njnt || model.jnt_type[id]!==3) {
      throw new RangeError("Articulación interna inválida: "+name);
    }
    const dof=model.jnt_dofadr[id]!;
    if(dof < 6 || dof >= appliedForces.length) {
      throw new RangeError("Prohibido modificar root con alivio de fricción");
    }
    appliedForces[dof] *= scale;
  }
  if(Array.from(appliedForces.slice(0,6)).some(n=>n!==0)) {
    throw new Error("Root no actuado: pares aplicados deben ser internos");
  }
}
