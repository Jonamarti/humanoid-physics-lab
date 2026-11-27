/** Classical C0: independent SIMBICON-inspired primitives, not a calibrated gait.
 * Yin, Loken, van de Panne (2007), sections 3.1–3.3.
 * Coordinates: X forward, Y left, Z up. Positive hip pitch about +Y points backwards.
 */
export type Side = "left" | "right";
export type Pair = readonly [number, number];
export type Phase = "lift" | "land" | "timeout";
export interface GaitState {
  swing: Side; phase: Phase; elapsedS: number; airborne: boolean; loadedS: number; steps: number;
}
export interface GaitObservation { loadN: Pair; clearanceM: Pair }
export const clamp = (x: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, x));
export function initialGait(): GaitState {
  return { swing: "left", phase: "lift", elapsedS: 0, airborne: false, loadedS: 0, steps: 0 };
}
/** A timed lift followed by verified contact; a timeout never counts as a step. */
export function advanceGait(state: GaitState, obs: GaitObservation, dt: number): GaitState {
  if (!(dt > 0) || !Number.isFinite(dt) || ![...obs.loadN, ...obs.clearanceM].every(Number.isFinite)
      || obs.loadN.some(v => v < 0)) throw new RangeError("Invalid gait observation");
  if (state.phase === "timeout") return state;
  const i = state.swing === "left" ? 0 : 1;
  const elapsedS = state.elapsedS + dt;
  const airborne = state.airborne || (obs.loadN[i]! < 5 && obs.clearanceM[i]! > 0.01 && obs.loadN[1-i]! > 20);
  if (elapsedS > 1.2) return { ...state, elapsedS, airborne, phase: "timeout" };
  if (state.phase === "lift") return { ...state, elapsedS, airborne, phase: elapsedS >= 0.32 ? "land" : "lift" };
  const loadedS = airborne && obs.loadN[i]! > 40 && obs.clearanceM[i]! < 0.015 ? state.loadedS + dt : 0;
  if (loadedS >= 0.06) return { ...initialGait(), swing: state.swing === "left" ? "right" : "left", steps: state.steps + 1 };
  return { ...state, elapsedS, airborne, loadedS };
}
export function pd(target: number, angle: number, rate: number, kp: number, kd: number, limit: number): number {
  if (![target, angle, rate, kp, kd, limit].every(Number.isFinite) || kp < 0 || kd < 0 || limit <= 0) throw new RangeError("Invalid PD");
  return clamp(kp * (target - angle) - kd * rate, -limit, limit);
}
/** The reaction on the common pelvis is -(stanceHip + swingHip).
 * Preserve that identity while respecting both motor limits. */
export function hipPair(torsoNm: number, desiredSwingNm: number, limitNm: number): {
  stanceNm: number; swingNm: number; residualNm: number;
} {
  if (![torsoNm, desiredSwingNm, limitNm].every(Number.isFinite) || limitNm <= 0) throw new RangeError("Invalid hip torque");
  const sum = clamp(-torsoNm, -2 * limitNm, 2 * limitNm);
  const swingNm = clamp(desiredSwingNm, Math.max(-limitNm, sum - limitNm), Math.min(limitNm, sum + limitNm));
  const stanceNm = sum - swingNm;
  return { stanceNm, swingNm, residualNm: torsoNm + stanceNm + swingNm };
}
/** Sagittal swing reference feedback from displacement/velocity of the COM
 * relative to the stance ankle. Coefficients are initial hypotheses. */
export function swingPitchTarget(baseRad: number, displacementM: number, velocityMps: number): number {
  if (![baseRad, displacementM, velocityMps].every(Number.isFinite)) throw new RangeError("Invalid COM feedback");
  return clamp(baseRad - 0.8 * displacementM - 0.22 * velocityMps, -0.75, 0.45);
}
