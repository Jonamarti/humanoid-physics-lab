/** Public, independently authored pre-seoi rigid-body benchmark.
 * The experimental parameters reflect historical AnimSpec tests, but this
 * module does not copy implementation from that private repository.
 */
export const CYLINDER = Object.freeze({
  massKg: 75,
  heightM: 1.75,
  radiusM: 0.18,
  horizontalGapM: 0.02,
  obstacleDepthM: 0.12,
  gripHeightM: 1.40,
  friction: 0.6,
  gravity: 9.81,
  startS: 0.30,
  timestepS: 0.001,
});

export interface CylinderTrial {
  readonly obstacleHeightM: number;
  /** Horizontal +X force, not magnitude of the full force vector. */
  readonly horizontalForceN: number;
  /** Ratio Fz/Fx; can be negative for a downward pull. */
  readonly upwardRatio: number;
  readonly durationS: number;
}

export const DEFAULT_CYLINDER_TRIAL: CylinderTrial = Object.freeze({
  obstacleHeightM: 0.8,
  horizontalForceN: 600,
  upwardRatio: 0.3,
  durationS: 0.15,
});

export function validateCylinderTrial(trial: CylinderTrial): void {
  const { obstacleHeightM, horizontalForceN, upwardRatio, durationS } = trial;
  if (![obstacleHeightM, horizontalForceN, upwardRatio, durationS].every(Number.isFinite) ||
      obstacleHeightM < 0 || obstacleHeightM > 1.5 ||
      horizontalForceN < 0 || horizontalForceN > 3000 ||
      upwardRatio < -1 || upwardRatio > 2 ||
      durationS < 0 || durationS > 1) {
    throw new RangeError("Parámetros del cilindro fuera del intervalo de ensayo");
  }
}

export function cylinderMJCF(obstacleHeightM: number): string {
  validateCylinderTrial({ ...DEFAULT_CYLINDER_TRIAL, obstacleHeightM });
  const { massKg, heightM, radiusM, horizontalGapM, obstacleDepthM, friction, timestepS } = CYLINDER;
  const halfDepth = obstacleDepthM / 2;
  const obstacle = obstacleHeightM > 0
    ? `<geom name="obstacle" type="box"
          pos="${radiusM + horizontalGapM + halfDepth} 0 ${obstacleHeightM / 2}"
          size="${halfDepth} 0.40 ${obstacleHeightM / 2}"
          rgba="0.68 0.43 0.24 1" friction="${friction} 0.005 0.0001"/>`
    : "";
  return `<mujoco model="cylinder_obstacle">
  <compiler angle="radian"/>
  <option timestep="${timestepS}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"
          rgba="0.23 0.28 0.32 1" friction="${friction} 0.005 0.0001"/>
    ${obstacle}
    <body name="uke_cylinder" pos="0 0 ${heightM / 2 + 0.001}">
      <freejoint name="free_cylinder"/>
      <geom name="cylinder" type="cylinder" size="${radiusM} ${heightM / 2}"
            mass="${massKg}" rgba="0.74 0.83 0.88 1"
            friction="${friction} 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>`;
}

/** Point of application relative to the rigid cylinder centre of mass,
 * expressed in the cylinder's local axes; MuJoCo transforms it each step.
 */
export function localGripPoint(): readonly [number, number, number] {
  return [CYLINDER.radiusM, 0, CYLINDER.gripHeightM - CYLINDER.heightM / 2];
}

/** Constant applied force during the selected interval, zero outside. */
export function cylinderForceAt(timeS: number, trial: CylinderTrial): readonly [number, number, number] {
  validateCylinderTrial(trial);
  if (!Number.isFinite(timeS)) throw new RangeError("Tiempo no finito");
  const active = timeS >= CYLINDER.startS - 1e-12 &&
    timeS < CYLINDER.startS + trial.durationS - 1e-12;
  return active
    ? [trial.horizontalForceN, 0, trial.horizontalForceN * trial.upwardRatio]
    : [0, 0, 0];
}

/** Rigid-pivot energetic approximation, NOT a prediction guaranteed by MuJoCo.
 * Assumes an instantaneous impulse about the obstacle's fixed top edge,
 * neglecting contact slip, collision compliance and active stepping.
 */
export function idealPivotPrediction(trial: CylinderTrial): {
  readonly tips: boolean;
  readonly initialAngularMomentumKgM2S: number;
  readonly initialKineticEnergyJ: number;
  readonly potentialBarrierJ: number;
} {
  validateCylinderTrial(trial);
  const { massKg: m, heightM: h, radiusM: r, horizontalGapM: gap, gripHeightM: grip, gravity } = CYLINDER;
  const centerOfMassZ = h / 2;
  const pivotX = r + gap;
  const pivotZ = trial.obstacleHeightM;
  const rx = -pivotX;
  const rz = centerOfMassZ - pivotZ;
  const ax = r - pivotX;
  const az = grip - pivotZ;
  const jx = trial.horizontalForceN * trial.durationS;
  const jz = jx * trial.upwardRatio;
  const angularMomentum = az * jx - ax * jz;
  const icm = m * (3 * r * r + h * h) / 12;
  const ipivot = icm + m * (rx * rx + rz * rz);
  const kinetic = angularMomentum > 0 ? (angularMomentum ** 2) / (2 * ipivot) : 0;
  const barrier = m * gravity * Math.max(0, Math.hypot(rx, rz) - rz);
  return {
    tips: angularMomentum > 0 && kinetic >= barrier,
    initialAngularMomentumKgM2S: angularMomentum,
    initialKineticEnergyJ: kinetic,
    potentialBarrierJ: barrier,
  };
}

/** Root qpos is MuJoCo [x,y,z,w,qx,qy,qz]; local cylinder Z up
 * projected on XZ gives a signed forward rotation (+X).
 */
export function cylinderForwardTiltDeg(rootQpos: readonly number[]): number {
  if (rootQpos.length < 7 || !rootQpos.slice(3, 7).every(Number.isFinite)) {
    throw new RangeError("Cuaternión de cuerpo libre inválido");
  }
  const [w, x, y, z] = rootQpos.slice(3, 7);
  const forwardX = 2 * (x! * z! + w! * y!);
  const upZ = 1 - 2 * (x! * x! + y! * y!);
  return Math.atan2(forwardX, upZ) * 180 / Math.PI;
}
