/**
 * Veredictos. Cilindro (cuerpo rígido vertical): inclinación del eje y dirección de caída; entrada, la matriz de rotación
 * `xmat` de MuJoCo (9 valores, por filas; el eje local Z es la tercera columna). Humanoide: `classifyHumanoidTrajectory`
 * (al final), una función pura sobre una trayectoria muestreada (sin MuJoCo), issue #75 PR 4.
 */
import type { Outcome } from "../scenario/schema";
import { octantOf, signedMargin, unbalanceDirection, wrapDeg, type Octant, type XY, type XYZ } from "./balance";

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

// ---------------------------------------------------------------------------------------------------------------------
// Humanoide
// ---------------------------------------------------------------------------------------------------------------------

/** Un pie «ha cambiado de posición» si su suela se desplaza en el plano horizontal más de esto (m) respecto a la inicial. */
export const STEP_MIN_FOOT_DISPLACEMENT_M = 0.05;
/** Tiempo (s) que debe mantenerse la estabilidad (punto de captura en el apoyo y velocidad baja) para dar el veredicto. */
export const STABLE_HOLD_S = 2;
/** Velocidad máxima del centro de masas (m/s) de un estado «estable». */
export const STABLE_MAX_COM_SPEED_MPS = 0.1;
/** Caída: la pelvis baja de esta fracción de su altura inicial. */
export const FALL_MAX_PELVIS_RATIO = 0.5;
/** Caída: si alguno de estos cuerpos toca el suelo (cabeza, pecho, abdomen o pelvis). */
export const FALL_CONTACT_BODIES: readonly string[] = ["head", "chest", "abdomen", "pelvis"];
/** Los pies no cuentan como «parte que toca el suelo». */
export const FOOT_BODIES: readonly string[] = ["left_foot", "right_foot"];
/** Cuerpos cuya velocidad de impacto se informa (el tronco superior, `chest`, hace de espalda). */
export const IMPACT_BODIES = ["head", "chest", "pelvis"] as const;
/** Tolerancia al comparar instantes (s). */
const TIME_EPS_S = 1e-6;

/** Estado de equilibrio de una muestra de la trayectoria de un humanoide (a `record_hz`). Suelo en z = 0. */
export interface BalanceFrame {
  /** Tiempo del escenario, en s. */
  t: number;
  /** Centro de masas, en m. */
  com: XYZ;
  /** Velocidad del centro de masas, en m/s. */
  com_velocity: XYZ;
  /** Punto de captura `c + v / w0` en el plano horizontal, en m. */
  xcom: XY;
  /** Polígono de apoyo (envolvente convexa antihoraria de los contactos de los pies con carga > 20 N); vacío si ninguno. */
  support_polygon: readonly XY[];
  /** Altura de la pelvis, en m. */
  pelvis_z: number;
  /** Posición horizontal de la suela de cada pie, en m. */
  feet: { left: XY; right: XY };
  /** Dirección del eje del tronco (cuerpo `chest`) en el mundo (0 = +X, 90 = +Y); `null` si se inclina menos de 5°. */
  trunk_dir_deg: number | null;
}

/** Primer contacto con el suelo de una parte del cuerpo (los pies, aunque se pasen, se ignoran). */
export interface GroundTouch {
  body: string;
  t_s: number;
  /** Velocidad del punto de contacto justo antes del contacto, en m/s (mundo). */
  velocity_mps: XYZ;
}

export interface HumanoidTrajectory {
  /** Muestras en orden de tiempo creciente (al menos una). */
  frames: readonly BalanceFrame[];
  /** Primeros contactos con el suelo de cada cuerpo del actor, en cualquier orden. */
  touches: readonly GroundTouch[];
  /** Altura de la pelvis tras el asentamiento, en m. */
  initial_pelvis_z: number;
  /** Azimut del frente inicial de Uke (0 = +X), en radianes. */
  heading_rad: number;
}

export interface ImpactInfo {
  t_s: number;
  /** Módulo de la velocidad del punto de contacto justo antes del impacto, en m/s. */
  speed_mps: number;
  /** Componente hacia el suelo de esa velocidad (>= 0), en m/s. */
  normal_speed_mps: number;
}

export interface FallInfo {
  /** Instante en que se cumple por primera vez el criterio de caída, en s. */
  t_s: number;
  /** `contact:<cuerpo>` (cabeza, pecho, abdomen o pelvis tocan el suelo) o `pelvis_height` (pelvis < 50 % de la inicial). */
  trigger: string;
  /** Dirección de la caída en grados respecto al frente inicial de Uke (0 = delante, 90 = izquierda, 180 = atrás); `null` si el tronco nunca se inclina 5°. */
  direction_deg: number | null;
  /** La misma dirección en el mundo (0 = +X, 90 = +Y). */
  direction_world_deg: number | null;
  /** La dirección anterior cuantizada a una de 8. */
  direction_name: Octant | null;
  /** Primera parte (no pie) que toca el suelo, y su instante; `null` si ninguna toca. */
  first_part: string | null;
  first_part_t_s: number | null;
  /** Orden de los primeros contactos de las partes (no pies) con el suelo. */
  contact_order: Array<{ body: string; t_s: number }>;
  /** Velocidad de impacto de cabeza, tronco superior (`chest`) y pelvis; `null` si esa parte no toca. */
  impact: Record<(typeof IMPACT_BODIES)[number], ImpactInfo | null>;
}

export interface StepInfo {
  /** Primer instante en que un pie se desplaza más de 5 cm, en s. */
  t_s: number;
  /** Pie que lo hace primero. */
  foot: "left" | "right";
}

export interface UnbalanceInfo {
  /** Primer instante en que el punto de captura queda fuera del apoyo, en s. */
  t_s: number;
  /** Dirección, relativa al frente inicial, en que sale: múltiplo de 45° en (-180, 180]. */
  direction_deg: number;
  direction: Octant;
  /** Margen más negativo alcanzado (m, <= 0): cuánto se aleja el punto de captura del apoyo. */
  min_margin_m: number;
  /** `true` si en la última muestra el punto de captura está de nuevo dentro del apoyo. */
  recovered: boolean;
}

export interface BalanceSummary {
  /** Margen con signo (m) del punto de captura al polígono de apoyo: mínimo y último. `null` si ninguna muestra tiene apoyo. */
  min_capture_margin_m: number | null;
  final_capture_margin_m: number | null;
  final_com_speed_mps: number;
  /** Desplazamiento horizontal máximo de la suela de cada pie respecto a su posición inicial, en m. */
  max_foot_displacement_m: { left: number; right: number };
  /** Instante desde el que (y hasta el final) el punto de captura está en el apoyo y la velocidad del centro de masas < 0,1 m/s; `null` si no lo está al final. */
  stable_since_s: number | null;
}

export type HumanoidOutcome = "fall" | "stepped" | "standing" | "unresolved";

export interface HumanoidVerdict {
  outcome: HumanoidOutcome;
  /** Instante en que el veredicto adoptó su valor final (caída: la caída; de pie o paso: cuando la estabilidad ya no se pierde; `unresolved`: la última muestra), en s. */
  t_outcome_s: number;
  fall: FallInfo | null;
  step: StepInfo | null;
  unbalance: UnbalanceInfo | null;
  balance: BalanceSummary;
}

function norm3(v: XYZ): number {
  return Math.hypot(v[0], v[1], v[2]);
}

function marginOf(frame: BalanceFrame): number | null {
  return signedMargin(frame.xcom, frame.support_polygon);
}

function isStable(frame: BalanceFrame): boolean {
  const margin = marginOf(frame);
  return margin !== null && margin >= 0 && norm3(frame.com_velocity) < STABLE_MAX_COM_SPEED_MPS;
}

function buildFall(traj: HumanoidTrajectory, tPelvis: number | null): FallInfo | null {
  const touches = traj.touches.filter((c) => !FOOT_BODIES.includes(c.body)).sort((a, b) => a.t_s - b.t_s);
  const fallTouch = touches.find((c) => FALL_CONTACT_BODIES.includes(c.body));
  if (fallTouch === undefined && tPelvis === null) return null;
  // Gana el contacto si coincide con la altura de la pelvis.
  const byContact = fallTouch !== undefined && (tPelvis === null || fallTouch.t_s <= tPelvis);
  const t = byContact ? fallTouch!.t_s : tPelvis!;
  // Dirección del eje del tronco en la primera muestra, desde el instante de caída, en que está definida.
  const dirFrame = traj.frames.find((f) => f.t >= t - TIME_EPS_S && f.trunk_dir_deg !== null);
  const world = dirFrame ? dirFrame.trunk_dir_deg : null;
  const relative = world === null ? null : wrapDeg(world - (traj.heading_rad * 180) / Math.PI);
  const impact = Object.fromEntries(IMPACT_BODIES.map((body): [string, ImpactInfo | null] => {
    const c = touches.find((x) => x.body === body);
    return [body, c ? { t_s: c.t_s, speed_mps: norm3(c.velocity_mps), normal_speed_mps: Math.max(0, -c.velocity_mps[2]) } : null];
  })) as FallInfo["impact"];
  return {
    t_s: t,
    trigger: byContact ? `contact:${fallTouch!.body}` : "pelvis_height",
    direction_deg: relative,
    direction_world_deg: world,
    direction_name: relative === null ? null : octantOf(relative).name,
    first_part: touches[0]?.body ?? null,
    first_part_t_s: touches[0]?.t_s ?? null,
    contact_order: touches.map((c) => ({ body: c.body, t_s: c.t_s })),
    impact,
  };
}

/**
 * Veredicto del humanoide sobre una trayectoria muestreada (función pura: cada rama se comprueba sin simulación).
 *
 * - `fall`: cabeza, pecho, abdomen o pelvis tocan el suelo, o la pelvis baja del 50 % de su altura inicial (en cualquier instante).
 * - Estable: punto de captura dentro del apoyo (margen >= 0) y velocidad del centro de masas < 0,1 m/s. «Se mantiene estable» =
 *   lo es en todas las muestras de los últimos 2 s de la grabación.
 * - `stepped`: algún pie se desplazó más de 5 cm (en cualquier instante), han pasado al menos 2 s desde ese instante y se mantiene
 *   estable al final. `standing`: lo mismo sin mover los pies (la grabación dura al menos 2 s).
 * - `unresolved`: nada de lo anterior al acabar el tiempo.
 */
export function classifyHumanoidTrajectory(traj: HumanoidTrajectory): HumanoidVerdict {
  const { frames } = traj;
  if (frames.length === 0) throw new RangeError("La trayectoria no tiene muestras");
  const first = frames[0]!;
  const last = frames[frames.length - 1]!;

  // Desplazamiento de los pies y primer paso.
  const maxDisp = { left: 0, right: 0 };
  let step: StepInfo | null = null;
  for (const f of frames) {
    for (const side of ["left", "right"] as const) {
      const d = Math.hypot(f.feet[side][0] - first.feet[side][0], f.feet[side][1] - first.feet[side][1]);
      if (d > maxDisp[side]) maxDisp[side] = d;
      if (step === null && d > STEP_MIN_FOOT_DISPLACEMENT_M) step = { t_s: f.t, foot: side };
    }
  }

  // Margen del punto de captura, primera salida del apoyo y altura de la pelvis.
  let minMargin: number | null = null;
  let unbalance: UnbalanceInfo | null = null;
  let tPelvis: number | null = null;
  // Último polígono en cuyo interior estuvo el punto de captura: origen de la dirección de salida.
  let insidePolygon: readonly XY[] = first.support_polygon;
  for (const f of frames) {
    const margin = marginOf(f);
    if (margin !== null && (minMargin === null || margin < minMargin)) minMargin = margin;
    if (unbalance === null && (margin === null || margin < 0)) {
      const dir = unbalanceDirection(f.xcom, insidePolygon, f.com, traj.heading_rad);
      unbalance = { t_s: f.t, direction_deg: dir.direction_deg, direction: dir.name, min_margin_m: 0, recovered: false };
    } else if (unbalance === null) {
      insidePolygon = f.support_polygon;
    }
    if (tPelvis === null && f.pelvis_z < FALL_MAX_PELVIS_RATIO * traj.initial_pelvis_z) tPelvis = f.t;
  }
  const lastMargin = marginOf(last);
  if (unbalance) {
    unbalance.min_margin_m = Math.min(0, minMargin ?? 0);
    unbalance.recovered = lastMargin !== null && lastMargin >= 0;
  }

  // Tramo estable final.
  let stableSince: number | null = null;
  for (let i = frames.length - 1; i >= 0 && isStable(frames[i]!); i--) stableSince = frames[i]!.t;
  const held = stableSince !== null && last.t - stableSince >= STABLE_HOLD_S - TIME_EPS_S;

  const balance: BalanceSummary = {
    min_capture_margin_m: minMargin,
    final_capture_margin_m: lastMargin,
    final_com_speed_mps: norm3(last.com_velocity),
    max_foot_displacement_m: maxDisp,
    stable_since_s: stableSince,
  };

  const fall = buildFall(traj, tPelvis);
  if (fall) return { outcome: "fall", t_outcome_s: fall.t_s, fall, step, unbalance, balance };
  if (step === null && held) {
    return { outcome: "standing", t_outcome_s: stableSince!, fall: null, step, unbalance, balance };
  }
  if (step !== null && held && last.t - step.t_s >= STABLE_HOLD_S - TIME_EPS_S) {
    return { outcome: "stepped", t_outcome_s: Math.max(stableSince!, step.t_s), fall: null, step, unbalance, balance };
  }
  return { outcome: "unresolved", t_outcome_s: last.t, fall: null, step, unbalance, balance };
}
