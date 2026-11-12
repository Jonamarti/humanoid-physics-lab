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
  /** Tope de fuerza declarado por el escenario (`max_force_n`), en N; ausente si el efector no declara ninguno. */
  limit_n?: number;
}

export interface RecordingPeaks {
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

/** Medidas de un ariete (`summary.rams.<id>`, solo con efectores `ram`; ver `docs/GOLPES.md`). */
export interface RamSummary {
  /** Masa efectiva declarada, en kg. */
  mass_kg: number;
  /** Velocidad inicial declarada, en m/s. */
  speed_mps: number;
  /** Inicio del primer paso físico con contacto con otro cuerpo (Uke), en s; `null` si nunca lo tocó. */
  t_first_contact_s: number | null;
  /** Del primer contacto al final del último paso con contacto, en s (incluye separaciones intermedias); `null` sin contacto. */
  contact_duration_s: number | null;
  /** Pasos físicos (500 Hz) con contacto. */
  contact_steps: number;
  /** Mayor módulo de la fuerza neta del ariete sobre Uke en un paso, en N. */
  peak_force_n: number;
  /** Impulso `∫ F dt` del ariete sobre Uke, en N·s (el de `peaks.effectors.<id>.impulse_ns`). */
  transmitted_impulse_ns: Vec3;
  /** Velocidad del ariete al inicio del primer paso con contacto (la de llegada), en m/s. */
  v_before_mps: Vec3 | null;
  /** Velocidad del ariete al final del último paso con contacto, en m/s. */
  v_after_mps: Vec3 | null;
  /** Mayor penetración del contacto del ariete (`-dist`), en m. */
  max_penetration_m: number;
  /** Instante en que se retiró (se desactivó su colisión), en s; `null` si no llegó a retirarse. */
  retired_t_s: number | null;
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
  /** Medidas de cada efector `ram` por `id`; ausente sin arietes. */
  rams?: Record<string, RamSummary>;
  /** Artefactos detectados (`detectArtifacts`); lista vacía = limpio. Ausente en grabaciones anteriores a la PR 5 de #75. */
  artifacts?: ArtifactFinding[];
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
