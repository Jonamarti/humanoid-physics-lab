/** Reproducible external push at sternum; physics remains fully actuated
 * by MuJoCo contacts + internal joints, without any root stabilisation.
 *
 * The force is a half-sine pulse: J = 2 * Fpeak * duration / pi.
 * This is a test protocol, not a human-safe force prescription.
 */
export type PushDirection = "forward" | "backward" | "left" | "right";

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
  if (!["forward", "backward", "left", "right"].includes(trial.direction) ||
      ![trial.peakForceN, trial.startS, trial.durationS].every(Number.isFinite) ||
      trial.peakForceN < 0 || trial.peakForceN > 2000 ||
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
  if (phase <= 0 || phase >= 1) return [0, 0, 0];
  const force = trial.peakForceN * Math.sin(Math.PI * phase);
  switch (trial.direction) {
    case "forward": return [force, 0, 0];
    case "backward": return [-force, 0, 0];
    case "left": return [0, force, 0];
    case "right": return [0, -force, 0];
  }
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
