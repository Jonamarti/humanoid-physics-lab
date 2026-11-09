/** Reproducible external push at sternum; physics remains fully actuated
 * by MuJoCo contacts + internal joints, without any root stabilisation.
 *
 * The force is a half-sine pulse: J = 2 * Fpeak * duration / pi.
 * This is a test protocol, not a human-safe force prescription.
 */
/** +X forward, +Y left in world coordinates; diagonal vectors are unit length. */
export const PUSH_DIRECTIONS = [
  "forward", "backward", "left", "right",
  "forward-left", "forward-right", "backward-left", "backward-right",
] as const;
export type PushDirection = typeof PUSH_DIRECTIONS[number];

const INV_SQRT_2 = Math.SQRT1_2;
export const PUSH_DIRECTION_VECTORS: Readonly<Record<PushDirection, readonly [number, number, number]>> = {
  forward: [1, 0, 0], backward: [-1, 0, 0],
  left: [0, 1, 0], right: [0, -1, 0],
  "forward-left": [INV_SQRT_2, INV_SQRT_2, 0],
  "forward-right": [INV_SQRT_2, -INV_SQRT_2, 0],
  "backward-left": [-INV_SQRT_2, INV_SQRT_2, 0],
  "backward-right": [-INV_SQRT_2, -INV_SQRT_2, 0],
};

export interface PushTrial {
  readonly direction: PushDirection;
  readonly peakForceN: number;
  readonly startS: number;
  readonly durationS: number;
}

export const DEFAULT_PUSH_TRIAL: PushTrial = Object.freeze({
  direction: "forward",
  peakForceN: 50,
  startS: 1,
  durationS: 0.12,
});

export function validatePushTrial(trial: PushTrial): void {
  if (!PUSH_DIRECTIONS.includes(trial.direction) ||
      ![trial.peakForceN, trial.startS, trial.durationS].every(Number.isFinite) ||
      trial.peakForceN < 0 || trial.peakForceN > 10000 ||
      trial.startS < 0 || trial.startS > 120 ||
      trial.durationS <= 0 || trial.durationS > 1.0) {
    throw new RangeError("Parámetros de empujón fuera de rango");
  }
}

/** +X anterior, -X posterior, +Y lateral izquierda, -Y derecha.
 * World-frame axes are fixed so results can be compared and reproduced.
 */
export function pushForceAtTime(
  timeS: number,
  trial: PushTrial,
): readonly [number, number, number] {
  validatePushTrial(trial);
  if (!Number.isFinite(timeS)) throw new RangeError("Tiempo no finito");
  const phase = (timeS - trial.startS) / trial.durationS;
  // Floating-point division may produce 0.999999999999999 at the endpoint.
  // Snap endpoint residuals to zero to avoid phantom external impulses.
  if (phase <= 1e-12 || phase >= 1 - 1e-12) return [0, 0, 0];
  const force = trial.peakForceN * Math.sin(Math.PI * phase);
  const direction = PUSH_DIRECTION_VECTORS[trial.direction];
  // Direction vectors have unit magnitude; diagonal impulses carry the
  // same total N·s as cardinal impulses, not sqrt(2) times as much.
  return [direction[0] * force, direction[1] * force, 0];
}

export function expectedPushImpulseNs(trial: PushTrial): number {
  validatePushTrial(trial);
  return 2 * trial.peakForceN * trial.durationS / Math.PI;
}

export function rootTiltDegrees(freeRootQpos: readonly number[]): number {
  if (freeRootQpos.length < 7 || !freeRootQpos.slice(3, 7).every(Number.isFinite)) {
    throw new RangeError("Cuaternión raíz inválido");
  }
  const qx = freeRootQpos[4]!;
  const qy = freeRootQpos[5]!;
  const cosine = Math.max(-1, Math.min(1, 1 - 2 * (qx*qx + qy*qy)));
  return Math.acos(cosine) * 180 / Math.PI;
}
