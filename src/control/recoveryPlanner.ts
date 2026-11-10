import { capturePointXY, convexHullXY, insideConvexHullXY, type XY } from "./balanceSensors";
import type { FootSupport, GroundSupportObservation } from "./footContacts";

export type RecoveryDecision = "NONE" | "SHIFT_WEIGHT" | "STEP" | "FALL";
export type FootSide = "left" | "right";

export interface RecoveryInput {
  readonly comXY: XY;
  readonly comVelocityXY: XY;
  readonly comHeightM: number;
  readonly ground: GroundSupportObservation;
  /** True when large trunk rotation makes the LIPM approximation unreliable. */
  readonly excessiveTrunkTilt: boolean;
  /** Delay from detection before deciding to step. */
  readonly unstableDurationS: number;
  readonly maxStepReachM?: number;
  readonly stepReactionDelayS?: number;
  readonly captureMarginM?: number;
}

export interface RecoveryPlan {
  readonly decision: RecoveryDecision;
  readonly capturePointXY: XY;
  readonly supportFoot: FootSide | null;
  readonly swingFoot: FootSide | null;
  /** Proposed landing location; NOT a command to move the foot. */
  readonly targetFootXY: XY | null;
  readonly reason: string;
}

const distance = (a: XY, b: XY): number => Math.hypot(a[0] - b[0], a[1] - b[1]);
const finite = (p: XY): boolean => p.every(Number.isFinite);
const clamp = (value: number, limit: number): number =>
  Math.max(-limit, Math.min(limit, value));

function footprintTranslated(foot: FootSupport, target: XY): XY[] {
  return foot.footprintXY.map(p =>
    [p[0] + target[0] - foot.centerXY[0], p[1] + target[1] - foot.centerXY[1]]);
}

/** This is a pure, conservative proposal heuristic. It never moves a body
 * and never reports that a physical step has been executed.
 */
export function planRecovery(input: RecoveryInput): RecoveryPlan {
  const {
    comXY, comVelocityXY, comHeightM, ground, excessiveTrunkTilt,
    unstableDurationS,
  } = input;
  const reach = input.maxStepReachM ?? 0.38;
  const delay = input.stepReactionDelayS ?? 0.13;
  const margin = input.captureMarginM ?? 0.012;
  if (![...comXY, ...comVelocityXY, comHeightM, unstableDurationS,
    reach, delay, margin, ground.totalVerticalLoadN,
    ground.left.verticalLoadN, ground.right.verticalLoadN].every(Number.isFinite) ||
    comHeightM <= 0 || unstableDurationS < 0 || reach <= 0 ||
    delay < 0 || margin < 0 || ground.totalVerticalLoadN < 0 ||
    ground.left.verticalLoadN < 0 || ground.right.verticalLoadN < 0 ||
    !finite(ground.left.centerXY) || !finite(ground.right.centerXY)) {
    throw new RangeError("Observaciones o parámetros de recuperación inválidos");
  }
  const capture = capturePointXY(comXY, comVelocityXY, comHeightM);
  const result = (decision: RecoveryDecision, reason: string,
    supportFoot: FootSide | null = null, swingFoot: FootSide | null = null,
    targetFootXY: XY | null = null): RecoveryPlan =>
    ({ decision, capturePointXY: capture, reason, supportFoot, swingFoot, targetFootXY });

  if (excessiveTrunkTilt) return result("FALL", "Tronco fuera del régimen LIPM");
  if (!ground.left.isLoaded && !ground.right.isLoaded) {
    return result("FALL", "Sin fuerzas de apoyo verificadas");
  }
  if (insideConvexHullXY(capture, ground.loadedSupportPolygonXY, margin)) {
    return result("NONE", "Punto de captura dentro de la base cargada");
  }
  if (unstableDurationS < delay) {
    return result("SHIFT_WEIGHT", "Esperando la latencia del planificador");
  }
  if (!ground.left.isLoaded || !ground.right.isLoaded) {
    return result("FALL", "Falta soporte bilateral para transferir peso de forma segura");
  }

  // An initially centered COM is BETWEEN feet, not inside either sole.
  // Do not declare inevitable FALL: plan a weight-transfer phase first,
  // and only allow a real foot to lift once support is verified physically.
  const candidates: { support: FootSide; swing: FootSide; target: XY;
    score: number; needsTransfer: boolean }[] = [];
  for (const [supportSide, swingSide] of
    [["left", "right"], ["right", "left"]] as const) {
    const support = ground[supportSide];
    const swing = ground[swingSide];
    const supportHull = convexHullXY(support.footprintXY);
    const needsTransfer = !insideConvexHullXY(comXY, supportHull, margin);
    // Reject implausibly large shifts even if a distant step target exists.
    if (needsTransfer && distance(comXY, support.centerXY) > 0.3) continue;
    // Predict velocity growth during a simple reaction delay.
    const desired: XY = [
      capture[0] + comVelocityXY[0] * delay,
      capture[1] + comVelocityXY[1] * delay,
    ];
    const dx = desired[0] - swing.centerXY[0];
    const dy = desired[1] - swing.centerXY[1];
    const len = Math.hypot(dx, dy);
    const scale = len > reach ? reach / len : 1;
    const target: [number, number] = [
      swing.centerXY[0] + dx * scale,
      swing.centerXY[1] + dy * scale,
    ];
    // Keep left/right on their anatomical sides to avoid leg crossing.
    if (swingSide === "left" && target[1] < support.centerXY[1] + 0.04) {
      target[1] = support.centerXY[1] + 0.04;
    } else if (swingSide === "right" && target[1] > support.centerXY[1] - 0.04) {
      target[1] = support.centerXY[1] - 0.04;
    }
    if (distance(swing.centerXY, target) > reach + 1e-8) continue;
    const future = convexHullXY([
      ...support.footprintXY, ...footprintTranslated(swing, target),
    ]);
    if (!insideConvHullWithMargin(capture, future, margin)) continue;
    const score = distance(target, desired) +
      swing.verticalLoadN / Math.max(ground.totalVerticalLoadN, 1) +
      (needsTransfer ? 0.12 : 0);
    candidates.push({ support: supportSide, swing: swingSide, target, score, needsTransfer });
  }
  candidates.sort((a, b) => a.score - b.score);
  const best = candidates[0];
  if (!best) return result("FALL", "No hay paso alcanzable con apoyo único viable");
  if (best.needsTransfer) {
    return result("SHIFT_WEIGHT", "Debe transferir carga al pie de soporte antes de levantar",
      best.support, best.swing, best.target);
  }
  return result("STEP", "Objetivo de paso candidato; exigir despegue y aterrizaje físicos",
    best.support, best.swing, best.target);
}

function insideConvHullWithMargin(p: XY, vertices: readonly XY[], margin: number): boolean {
  // Small conservative guard against numerical boundary triggering.
  // We use the same convex-hull test as the observation; epsilon extends
  // outward by a configurable tolerance, it is not a safety certificate.
  return insideConvexHullXY(p, vertices, clamp(margin, 0.03));
}
