/**
 * Tipos de `hpl.recording.v1`: salida de `runScenario`. Nombres de campo en inglés y `snake_case`.
 * La reproducción de una grabación nunca recalcula física.
 */
import type { BalanceSummary, FallInfo, StepInfo, UnbalanceInfo } from "../metrics/outcome";
import type { Outcome, Vec3 } from "../scenario/schema";

export const RECORDING_SCHEMA_ID = "hpl.recording.v1";

export interface RecordingMeta {
  scenario_id: string;
  /** SHA-256 (hex) del escenario validado, serializado con el orden de claves del validador. */
  scenario_sha256: string;
  /** Versión de `package.json` en el momento de ejecutar. */
  engine_version: string;
  mujoco_version: string;
  physics_hz: number;
  record_hz: number;
  /** Ejes del mundo: X adelante, Y izquierda, Z arriba. */
  frame: "x_forward_y_left_z_up";
  /** Orden de los cuaterniones de `qpos`: w, x, y, z (como MuJoCo). */
  quat_order: "wxyz";
}

export interface RecordingModel {
  /** MJCF completo usado en la simulación (el vídeo y la reproducción no dependen del código). */
  mjcf: string;
  /** Nombres de los cuerpos, en el orden de los índices de MuJoCo (el primero es `world`). */
  bodies: string[];
  /** Nombres de las articulaciones, en el orden de MuJoCo. */
  joints: string[];
}

export interface EffectorSample {
  id: string;
  /** Punto de aplicación en el mundo, en m. */
  point_m: Vec3;
  /** Fuerza aplicada (en el mundo) durante el paso físico que empieza en este instante, en N. */
  force_n: Vec3;
}

export interface ContactSample {
  /** Cuerpo de la primera geometría del contacto (`world` para el suelo). */
  body_a: string;
  /** Cuerpo de la segunda geometría del contacto. */
  body_b: string;
  point_m: Vec3;
  /** Fuerza que `body_a` ejerce sobre `body_b`, en el mundo, en N. Solo contactos con fuerza normal > 0. */
  force_n: Vec3;
}

export interface RecordingSample {
  /** Tiempo del escenario en s (0 = fin del asentamiento). */
  t: number;
  qpos: number[];
  qvel: number[];
  effectors: EffectorSample[];
  contacts: ContactSample[];
  /** Centro de masas del actor, en m. Solo con actor humanoide: las grabaciones anteriores no traen estos cuatro campos. */
  com?: Vec3;
  /** Punto de captura `c + v / w0` en el plano horizontal `[x, y]`, en m. */
  xcom?: [number, number];
  /** Polígono de apoyo: envolvente convexa antihoraria de los contactos de los pies con carga > 20 N, `[x, y]` en m; vacío sin apoyo. */
  support_polygon?: Array<[number, number]>;
  /** Margen con signo del punto de captura al polígono de apoyo (m, positivo dentro); `null` sin apoyo. */
  capture_margin_m?: number | null;
}

export interface EffectorPeaks {
  /** Máximo del módulo de la fuerza, muestreado a 500 Hz, en N. */
  max_force_n: number;
  /** Impulso `∫ F dt` en el mundo, en N·s. */
  impulse_ns: Vec3;
  /** Trabajo `∫ F · v_punto dt`, en J. */
  work_j: number;
}

export interface RecordingPeaks {
  effectors: Record<string, EffectorPeaks>;
}

export type ExpectResult = "pass" | "fail" | "none";

/** Medidas del humanoide (solo con actor `humanoid`). */
export interface UkeSummary {
  /** Inclinación máxima del tronco (cuerpo `chest`) durante el escenario, en grados. */
  max_trunk_tilt_deg: number;
  /** Altura inicial de la pelvis (tras el asentamiento), en m. */
  initial_pelvis_height_m: number;
  /** Altura mínima de la pelvis durante el escenario, en m. */
  min_pelvis_height_m: number;
  /** Velocidad del centro de masas al terminar el asentamiento de 0,5 s, en m/s. */
  settle_com_speed_mps: number;
  /** Altura del centro de masas de pie tras el asentamiento, en m. */
  initial_com_height_m: number;
  /** Mayor |fuerza generalizada| escrita en los 6 grados de la raíz (debe ser 0). */
  max_root_generalized_force: number;
  /** Estado del controlador (`UkeController.status()`). */
  controller: Record<string, unknown>;
  /** Equilibrio: margen del punto de captura, desplazamiento de los pies y estabilidad final (ausente en grabaciones anteriores). */
  balance?: BalanceSummary;
  /** Datos de la caída; `null` si no hay caída. */
  fall?: FallInfo | null;
  /** Primer paso (pie e instante); `null` si ningún pie se movió más de 5 cm. */
  step?: StepInfo | null;
  /** Instante y dirección (una de 8) en que el punto de captura sale del apoyo; `null` si nunca sale. */
  unbalance?: UnbalanceInfo | null;
}

export interface RecordingSummary {
  outcome: Outcome;
  /**
   * Dirección de caída en grados (0 = +X, 90 = +Y): la del eje del actor cuando su inclinación supera 60 grados
   * por primera vez o, si nunca la supera, en su inclinación máxima; `null` si esa inclinación es < 5 grados.
   * Humanoide: la del eje del tronco al caer (`uke.fall.direction_world_deg`) y, sin caída, la de su inclinación máxima.
   */
  direction_deg: number | null;
  /** Instante en que el veredicto adoptó su valor final y ya no cambió, en s. */
  t_outcome_s: number;
  final_tilt_deg: number;
  max_tilt_deg: number;
  /** Altura final del centro de masas del actor, en m. */
  final_com_height_m: number;
  /** Desplazamiento horizontal del actor entre el inicio y el final, en m. */
  slide_m: number;
  /** `none` si el escenario no declara `expect`. */
  expect: ExpectResult;
  /** Diferencias con `expect` (vacía si pasa o no hay `expect`). */
  expect_diffs: string[];
  /** Solo con actor humanoide. */
  uke?: UkeSummary;
}

export interface Recording {
  schema: typeof RECORDING_SCHEMA_ID;
  meta: RecordingMeta;
  model: RecordingModel;
  samples: RecordingSample[];
  peaks: RecordingPeaks;
  summary: RecordingSummary;
}
