import {
  pushForceAtTime, validatePushTrial, type PushTrial,
} from "../physics/standingPush";

export type ForcePhase = "pending" | "active" | "applied";

export interface ForceDisplay {
  readonly phase: ForcePhase;
  /** Direction/magnitude at the theoretical pulse peak for persistent preview. */
  readonly referenceForceN: readonly [number, number, number];
  /** Maximum actual force sampled during the just-simulated frame, if any. */
  readonly displayForceN: readonly [number, number, number];
  readonly activeMagnitudeN: number;
  readonly activeFraction: number;
}

/** A force is ACTIVE only when it was really applied in a MuJoCo step.
 * A render at t > pulse end can still highlight the pulse if that render
 * includes the last force-bearing physics step (no missed short pulses).
 * There is no invented post-pulse force or extra physical impulse.
 */
export function forceDisplayForFrame(
  simulationTimeS: number,
  trial: PushTrial,
  appliedForceThisFrameN: readonly [number, number, number] = [0, 0, 0],
): ForceDisplay {
  validatePushTrial(trial);
  if (!Number.isFinite(simulationTimeS) ||
      !appliedForceThisFrameN.every(Number.isFinite)) {
    throw new RangeError("Tiempo o vector de fuerza no finito");
  }
  const referenceForceN = pushForceAtTime(trial.startS + trial.durationS / 2, trial);
  const activeMagnitudeN = Math.hypot(...appliedForceThisFrameN);
  const active = activeMagnitudeN > 1e-12;
  const phase: ForcePhase = active ? "active" :
    simulationTimeS >= trial.startS + trial.durationS ? "applied" : "pending";
  return {
    phase,
    referenceForceN,
    displayForceN: active ? appliedForceThisFrameN : referenceForceN,
    activeMagnitudeN: active ? activeMagnitudeN : 0,
    activeFraction: trial.peakForceN > 0
      ? Math.min(1, activeMagnitudeN / trial.peakForceN) : 0,
  };
}
