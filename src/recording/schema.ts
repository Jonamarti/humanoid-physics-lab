/**
 * Tipos de `hpl.recording.v1`: salida de `runScenario`. Nombres de campo en inglés y `snake_case`.
 * La reproducción de una grabación nunca recalcula física.
 */
import type { ToriBudgetSummary } from "../effectors/toriBudget";
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

/**
 * Estado de una mano virtual en una muestra (`src/effectors/hand.ts`, `docs/VIRTUAL_HAND.md`). El punto de agarre y la fuerza
 * van en `point_m` y `force_n` del `EffectorSample` que lo contiene; aquí, lo propio de la mano.
 */
export interface HandSampleData {
  /** `true` si la mano está cerrada y no soltada (`false` antes de `close_s` y tras una suelta o un resbalón). */
  held: boolean;
  /** Objetivo de la mano tras recortarlo al alcance, en el mundo, en m; `null` si no sujeta. */
  target_m: Vec3 | null;
  /** La demanda del muelle llegó al tope de fuerza (la fuerza se recortó a `max_force_n`). */
  force_limited: boolean;
  /** El tope de potencia redujo la fuerza. */
  power_limited: boolean;
  /** El objetivo se recortó a la esfera de alcance desde el hombro. */
  reach_limited: boolean;
}

export interface EffectorSample {
  id: string;
  /** Punto de aplicación en el mundo, en m (en una mano, el punto de agarre). */
  point_m: Vec3;
  /** Fuerza aplicada (en el mundo) durante el paso físico que empieza en este instante, en N. */
  force_n: Vec3;
  /** Solo en manos (`kind: "hand"` del escenario); ausente en la fuerza prescrita y en grabaciones anteriores. */
  kind?: "hand";
  /** Solo con `kind: "hand"`. */
  hand?: HandSampleData;
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

/** Resumen de una mano virtual (a 500 Hz); ver `docs/VIRTUAL_HAND.md`. */
export interface HandPeaks {
  /** Máximo de `F · v_punto` (W), con la velocidad al comienzo del paso; 0 si nunca fue positivo. */
  max_power_w: number;
  /** Tiempo total con la demanda del muelle en el tope de fuerza, en s. */
  time_at_cap_s: number;
  /** Tiempo total en que el tope de potencia redujo la fuerza, en s. */
  time_power_limited_s: number;
  /** Tiempo total con el objetivo recortado a la esfera de alcance, en s. */
  time_reach_limited_s: number;
  /** Cierre, suelta programada y resbalón, con el instante del paso en que ocurren. */
  events: Array<{ t_s: number; kind: "close" | "release" | "slip" }>;
}

export interface EffectorPeaks {
  /** Solo en tirones guiados (`kind: "pull"`): máximo de `F · v` (W) y tiempo con el tope de potencia actuando (s). */
  pull?: { max_power_w: number; time_power_limited_s: number };
  /** Máximo del módulo de la fuerza, muestreado a 500 Hz, en N. */
  max_force_n: number;
  /** Impulso `∫ F dt` en el mundo, en N·s. */
  impulse_ns: Vec3;
  /** Trabajo `∫ F · v_punto dt`, en J. */
  work_j: number;
  /** Tope de fuerza declarado por el escenario (`max_force_n`), en N; ausente si el efector no declara ninguno. */
  limit_n?: number;
  /** Solo en manos virtuales. */
  hand?: HandPeaks;
}

/** Resumen de un obstáculo con fuerza limitada (a 500 Hz, desde t = 0); ver `docs/LIMITED_OBSTACLE.md`. */
export interface ObstaclePeaks {
  /** Máximos de la orden del actuador (incluye la compensación del peso propio), en N. */
  max_force_n: number;
  max_horizontal_force_n: number;
  max_vertical_force_n: number;
  /** Topes declarados (horizontal y vertical), en N. */
  limit_horizontal_n: number;
  limit_vertical_n: number;
  /** Máximo del módulo de la fuerza de contacto del obstáculo sobre el actor, en N. */
  max_contact_force_on_actor_n: number;
  /** Impulso de la fuerza de contacto sobre el actor, en N·s. */
  contact_impulse_on_actor_ns: Vec3;
  /** Máxima separación entre el obstáculo y su objetivo (cuánto cede), en m. */
  max_deviation_m: number;
  max_speed_mps: number;
  /** Tiempo con algún tope recortando la orden, en s. */
  time_saturated_s: number;
  /** Trabajo de la orden del actuador, en J. */
  work_j: number;
}

/** Resumen de un segmento del cuerpo de Tori (`src/effectors/toriBody.ts`). */
export interface ToriSegmentPeaks {
  /** Máximo del módulo de su fuerza de contacto sobre el actor, en N. */
  max_contact_force_on_actor_n: number;
  /** Tiempo en contacto con el actor, en s. */
  contact_time_s: number;
  /** Máxima separación de su objetivo (cuánto cede), en m. */
  max_deviation_m: number;
  /** Tiempo con algún tope recortando la orden, en s. */
  time_saturated_s: number;
}

export interface ToriBodyPeaks {
  /** Máximo del módulo de la fuerza de contacto total del cuerpo de Tori sobre el actor, en N. */
  max_contact_force_on_actor_n: number;
  contact_impulse_on_actor_ns: Vec3;
  /** Por segmento (`pelvis`, `trunk`, ...). */
  segments: Record<string, ToriSegmentPeaks>;
}

export interface RecordingPeaks {
  /** Solo si el escenario tiene obstáculos. */
  obstacles?: Record<string, ObstaclePeaks>;
  /** Solo si el escenario tiene cuerpo de Tori activo. */
  tori_body?: ToriBodyPeaks;
  effectors: Record<string, EffectorPeaks>;
}

/** Tipos del detector de artefactos (`src/metrics/artifacts.ts`, `docs/ARTIFACTS.md`). */
export const ARTIFACT_TYPES = [
  "ground_penetration", "segment_penetration", "joint_speed", "contact_force", "foot_slip", "tremor",
  "energy_gain", "effector_over_limit", "root_force",
] as const;
export type ArtifactType = (typeof ARTIFACT_TYPES)[number];

/** Un artefacto detectado: tipo, instante, elemento implicado, valor medido y umbral superado. */
export interface ArtifactFinding {
  type: ArtifactType;
  /** Instante del escenario en que se midió el valor, en s. */
  t_s: number;
  /** Cuerpo, par de cuerpos, articulación o efector implicado (vacío si no aplica). */
  where: string;
  /** Valor medido (unidades en `docs/ARTIFACTS.md`). */
  value: number;
  /** Umbral superado. */
  threshold: number;
  detail: string;
}

/** Un detector que no pudo evaluarse (nunca equivale a «limpio»). */
export interface ArtifactNotEvaluated {
  type: ArtifactType;
  reason: string;
}

/** Máximo de una magnitud medida en cada paso físico (500 Hz), con su instante y el elemento implicado. */
export interface PeakAt {
  value: number;
  t_s: number;
  where: string;
}

/**
 * Medidas del ejecutor que el detector necesita y que NO se pueden sacar de las muestras a `record_hz` (se acumulan en cada
 * paso físico; solo cuentan los pasos con `t >= 0`, no el asentamiento). Un campo ausente significa «no medido» (el
 * detector lo marca como no evaluado): por ejemplo, sin articulaciones internas no hay `joint_speed`.
 */
export interface RecordingDiagnostics {
  /** Masa del actor, en kg. */
  actor_mass_kg: number;
  /** Penetración máxima de una geometría del actor en el suelo, en m (`-dist` del contacto). */
  ground_penetration: PeakAt;
  /** Penetración máxima entre dos cuerpos distintos del actor, en m. Solo humanoide. */
  segment_penetration?: PeakAt;
  /** Mayor |velocidad| de una articulación interna (bisagra), en rad/s. */
  joint_speed?: PeakAt;
  /** Mayor fuerza normal de un contacto del actor (suelo, props o él mismo), en N. */
  contact_force: PeakAt;
  /** Mayor deslizamiento neto de un pie cargado y sin rozamiento saturado en un tramo continuo, en m. Solo humanoide. */
  foot_slip?: PeakAt;
  /** Mayor número de cambios de sentido (de al menos `amplitude_rad`) de una articulación en una ventana de 1 s. */
  tremor?: PeakAt & { amplitude_rad: number };
  /** Mayor exceso de energía mecánica sobre la aportada (pares internos y efectores), en J, y la escala de referencia. */
  energy?: { max_excess_j: number; t_s: number; scale_j: number };
  /** Mayor |fuerza generalizada| escrita en los 6 grados de la raíz (debe ser 0), en cualquier paso. */
  root_generalized_force?: number;
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
  /** Artefactos detectados (`detectArtifacts`); lista vacía = limpio. Ausente en grabaciones anteriores a la PR 5 de #75. */
  artifacts?: ArtifactFinding[];
  /** Presupuesto de Tori (`src/effectors/toriBudget.ts`); solo si el escenario declara `tori`. */
  tori_budget?: ToriBudgetSummary;
  /** Detectores que no pudieron evaluarse y por qué. Ausente en grabaciones anteriores. */
  artifacts_not_evaluated?: ArtifactNotEvaluated[];
}

export interface Recording {
  schema: typeof RECORDING_SCHEMA_ID;
  meta: RecordingMeta;
  model: RecordingModel;
  samples: RecordingSample[];
  peaks: RecordingPeaks;
  summary: RecordingSummary;
  /** Medidas por paso físico para el detector de artefactos. Ausente en grabaciones anteriores. */
  diagnostics?: RecordingDiagnostics;
}
