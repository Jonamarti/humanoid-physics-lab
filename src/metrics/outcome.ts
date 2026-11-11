/**
 * Veredicto mínimo para un cuerpo rígido vertical (el cilindro): inclinación del eje y dirección de caída.
 * Entrada: la matriz de rotación `xmat` de MuJoCo (9 valores, por filas); el eje local Z es la tercera columna.
 */
import type { Outcome } from "../scenario/schema";

/** Por debajo de este ángulo (grados) el actor sigue de pie. */
export const STANDING_MAX_TILT_DEG = 15;
/** Por encima de este ángulo (grados) el actor ha caído. */
export const FALL_MIN_TILT_DEG = 60;
/** Por debajo de este ángulo (grados) la dirección de caída no está definida. */
export const DIRECTION_MIN_TILT_DEG = 5;

const RAD_TO_DEG = 180 / Math.PI;

/** Ángulo en grados entre el eje local Z del cuerpo y la vertical del mundo. */
export function tiltDeg(xmat: ArrayLike<number>, offset = 0): number {
  const xz = xmat[offset + 2]!;
  const yz = xmat[offset + 5]!;
  const zz = xmat[offset + 8]!;
  return Math.atan2(Math.hypot(xz, yz), zz) * RAD_TO_DEG;
}

/** `standing` si < 15 grados, `fall` si > 60 grados y `tilted` en otro caso. */
export function classifyTilt(tilt: number): Outcome {
  if (tilt < STANDING_MAX_TILT_DEG) return "standing";
  if (tilt > FALL_MIN_TILT_DEG) return "fall";
  return "tilted";
}

/**
 * Dirección hacia la que se inclina el eje, en grados (0 = +X, 90 = +Y), en (-180, 180].
 * `null` si la inclinación es menor que `DIRECTION_MIN_TILT_DEG`.
 */
export function fallDirectionDeg(xmat: ArrayLike<number>, offset = 0): number | null {
  if (tiltDeg(xmat, offset) < DIRECTION_MIN_TILT_DEG) return null;
  return Math.atan2(xmat[offset + 5]!, xmat[offset + 2]!) * RAD_TO_DEG;
}

/** Diferencia angular absoluta entre dos direcciones, en [0, 180] grados. */
export function angularDifferenceDeg(a: number, b: number): number {
  const d = Math.abs(a - b) % 360;
  return d > 180 ? 360 - d : d;
}

/** Veredicto mínimo del humanoide (el completo es de otra PR): de pie si el tronco se inclina menos de este ángulo (grados)... */
export const HUMANOID_STANDING_MAX_TRUNK_TILT_DEG = 3;
/** ...y la pelvis sigue por encima de esta fracción de su altura inicial. */
export const HUMANOID_STANDING_MIN_PELVIS_RATIO = 0.9;
/** Caído si la pelvis baja de esta fracción de su altura inicial. */
export const HUMANOID_FALL_MAX_PELVIS_RATIO = 0.5;

/** `fall` si la pelvis < 50 % de su altura inicial; `standing` si el tronco < 3° y la pelvis > 90 %; si no, `tilted`. */
export function classifyHumanoid(trunkTiltDegrees: number, pelvisHeightRatio: number): Outcome {
  if (pelvisHeightRatio < HUMANOID_FALL_MAX_PELVIS_RATIO) return "fall";
  if (trunkTiltDegrees < HUMANOID_STANDING_MAX_TRUNK_TILT_DEG && pelvisHeightRatio > HUMANOID_STANDING_MIN_PELVIS_RATIO) {
    return "standing";
  }
  return "tilted";
}
