/**
 * Tipos de `hpl.scenario.v1`. Los nombres de campo van en inglés y `snake_case`.
 * Referencia de cada campo, con unidades: `docs/SCENARIO_V1.md`.
 */

/** Identificador de versión del esquema. */
export const SCENARIO_SCHEMA_ID = "hpl.scenario.v1";

/** Frecuencia de la física en Hz; `record_hz` debe dividirla exactamente. */
export const SCENARIO_PHYSICS_HZ = 500;

/** Duración máxima admitida (escenario y efectores), en segundos. */
export const SCENARIO_MAX_DURATION_S = 30;

/** Rango admitido de `record_hz`. */
export const SCENARIO_RECORD_HZ_MIN = 30;
export const SCENARIO_RECORD_HZ_MAX = 500;

/** Terna de números `[x, y, z]`. Ejes: X adelante, Y izquierda, Z arriba. */
export type Vec3 = readonly [number, number, number];

export type ActorKind = "rigid_cylinder" | "humanoid";
export type PropKind = "fixed_box";
export type EffectorKind = "prescribed_force" | "hand" | "pull";

/**
 * Cuerpos referenciables de cada tipo de actor. El resto de fases ampliarán la tabla
 * (por ejemplo, los segmentos del humanoide).
 */
export const ACTOR_BODIES: Readonly<Record<ActorKind, readonly string[]>> = {
  rigid_cylinder: ["cylinder"],
  // La raíz (pelvis) va primero: es el cuerpo principal del actor. Nombres del modelo `src/uke/generate.ts`.
  humanoid: [
    "pelvis", "abdomen", "chest", "head", "left_upperarm", "left_forearm", "right_upperarm", "right_forearm",
    "left_thigh", "left_shin", "left_foot", "right_thigh", "right_shin", "right_foot",
  ],
};

/**
 * Actitudes de Uke: `tense` (postura objetivo y equilibrio activo) y `relaxed` (sin objetivos: solo amortiguación
 * articular pasiva y los topes de rango del MJCF).
 */
export const UKE_ATTITUDES = ["tense", "relaxed"] as const;
export type UkeAttitude = (typeof UKE_ATTITUDES)[number];

/**
 * Sitios con nombre del humanoide y el cuerpo al que pertenecen (`src/uke/generate.ts`). Un efector puede apuntar a uno
 * con `site` en lugar de `body` + `local_point_m`.
 */
export const HUMANOID_SITE_BODY: Readonly<Record<string, string>> = {
  pelvis_origin: "pelvis",
  belt_front: "abdomen", belt_back: "abdomen", abdomen_front: "abdomen",
  sternum: "chest", sternum_impact: "chest",
  nape: "head", head_front: "head", head_side_left: "head", head_side_right: "head",
  ...Object.fromEntries(["left", "right"].flatMap((s) => [
    [`${s}_shoulder_impact`, `${s}_upperarm`], [`${s}_triceps`, `${s}_upperarm`], [`${s}_armpit`, `${s}_upperarm`],
    [`${s}_forearm_impact`, `${s}_forearm`], [`${s}_sleeve_elbow`, `${s}_forearm`], [`${s}_wrist`, `${s}_forearm`],
    [`${s}_knee_back`, `${s}_shin`],
    [`${s}_sole`, `${s}_foot`], [`${s}_ankle`, `${s}_foot`],
  ])),
};

/** Posturas iniciales de Uke (tabla en `src/uke/stances.ts`). */
export const UKE_STANCES = ["neutral", "natural_grip", "defensive", "staggered_left", "staggered_right"] as const;
export type UkeStance = (typeof UKE_STANCES)[number];

/** Claves de `segment_lengths_m` del humanoide (en m; definiciones en `docs/UKE_MODEL.md`). */
export const SEGMENT_LENGTH_KEYS = ["thigh", "shin", "foot", "upper_arm", "forearm", "trunk", "neck"] as const;
export type SegmentLengthKey = (typeof SEGMENT_LENGTH_KEYS)[number];

/** Marcos en los que se expresa `force_n`. */
export const EFFECTOR_FRAMES = ["world", "actor_initial", "actor_current"] as const;
export type EffectorFrame = (typeof EFFECTOR_FRAMES)[number];

/** Tipos de unión de la mano virtual (`src/effectors/hand.ts`): solo tracción, tracción y empuje, o solo empuje. */
export const HAND_LINKS = ["rope", "clamp", "push"] as const;
export type HandLinkKind = (typeof HAND_LINKS)[number];

/** Formas temporales de la fuerza prescrita. */
export const EFFECTOR_WAVEFORMS = ["half_sine", "constant"] as const;
export type EffectorWaveform = (typeof EFFECTOR_WAVEFORMS)[number];

/**
 * Resultados posibles del veredicto. Cilindro: `standing`, `tilted` o `fall`. Humanoide: `standing`, `stepped`, `fall` o
 * `unresolved` (definiciones en `src/metrics/outcome.ts`).
 */
export const OUTCOMES = ["standing", "tilted", "fall", "stepped", "unresolved"] as const;
export type Outcome = (typeof OUTCOMES)[number];

export interface GroundSpec {
  /** Coeficiente de rozamiento deslizante del suelo (adimensional, >= 0). */
  friction: number;
}

export interface RigidCylinderActor {
  id: string;
  kind: "rigid_cylinder";
  /** Masa en kg (> 0). */
  mass_kg: number;
  /** Altura en m (> 0). */
  height_m: number;
  /** Radio en m (> 0). */
  radius_m: number;
}

export interface HumanoidActor {
  id: string;
  kind: "humanoid";
  /** Masa en kg (50 a 150). */
  mass_kg: number;
  /** Altura en m (1,50 a 2,05). */
  height_m: number;
  /** Factor de fuerza (0,5 a 1,5). Por defecto 1. Escala el par máximo. */
  strength: number;
  /** Fracción (0 a 1) del exceso de masa del tronco para IMC > 27. Por defecto, la de `src/uke/generate.ts` (0,6). */
  build?: number;
  /** Longitudes explícitas de segmentos, en m. */
  segment_lengths_m?: Partial<Record<SegmentLengthKey, number>>;
  /** Por defecto `tense`. */
  attitude: UkeAttitude;
  /** Por defecto `neutral`. */
  stance: UkeStance;
}

export type ActorSpec = RigidCylinderActor | HumanoidActor;

export interface FixedBoxProp {
  id: string;
  kind: "fixed_box";
  /** Aristas completas `[x, y, z]` en m (> 0). */
  size_m: Vec3;
  /** Posición del centro en el mundo, en m. */
  pos_m: Vec3;
}

export type PropSpec = FixedBoxProp;

export interface PrescribedForceEffector {
  id: string;
  kind: "prescribed_force";
  /** `id` de un actor existente. */
  actor: string;
  /**
   * Cuerpo del actor al que se aplica (ver `ACTOR_BODIES`). Con `site`, el validador lo rellena con el cuerpo
   * que contiene el sitio.
   */
  body: string;
  /** Alternativa a `body` + `local_point_m` (solo humanoide): sitio con nombre (ver `HUMANOID_SITE_BODY`). */
  site?: string;
  /**
   * Punto de aplicación en el sistema local del cuerpo, en m. Con `site` es un desplazamiento opcional (por defecto
   * `[0, 0, 0]`) respecto al sitio, en los ejes del cuerpo.
   */
  local_point_m: Vec3;
  frame: EffectorFrame;
  /** Fuerza máxima en N, expresada en `frame`. */
  force_n: Vec3;
  /** Instante de inicio en s, medido desde el comienzo del escenario (>= 0). */
  start_s: number;
  /** Duración en s (> 0 y <= 30). */
  duration_s: number;
  waveform: EffectorWaveform;
}

/** Tramo del objetivo de una mano: desplazamiento acumulado respecto al agarre al cerrarse la mano (mínimo tirón). */
export interface HandPhaseSpec {
  /** Instante de inicio en s, desde el comienzo del escenario (no antes de `close_s`). */
  start_s: number;
  /** Duración en s (> 0). */
  duration_s: number;
  /** Desplazamiento acumulado en m, en el marco `actor_initial` de Uke, alcanzado al final del tramo. */
  offset_m: Vec3;
}

/**
 * Mano virtual de Tori (`src/effectors/hand.ts`, `docs/VIRTUAL_HAND.md`): un punto objetivo unido por muelle y amortiguador
 * al agarre, con tope de fuerza, de potencia y de alcance desde un hombro. Tori no tiene cuerpo: el hombro, el objetivo y los
 * ejes se dan en el marco `actor_initial` del actor (ejes y origen del cuerpo principal al terminar el asentamiento, t = 0).
 * Los campos opcionales ausentes toman los valores por defecto de `HAND_DEFAULTS` (no se rellenan al validar).
 */
export interface HandEffector {
  id: string;
  kind: "hand";
  actor: string;
  /** Cuerpo del actor que se agarra; con `site`, el validador lo rellena con el cuerpo que contiene el sitio. */
  body: string;
  /** Alternativa a `body` + `local_point_m` (solo humanoide): sitio con nombre (ver `HUMANOID_SITE_BODY`). */
  site?: string;
  /** Punto de agarre en el sistema local del cuerpo (con `site`: desplazamiento opcional respecto al sitio). */
  local_point_m: Vec3;
  link: HandLinkKind;
  /** Hombro de Tori (centro de la esfera de alcance), en m, en el marco `actor_initial`. */
  shoulder_m: Vec3;
  /** Instante en que la mano se cierra sobre el agarre y empieza a actuar, en s. */
  close_s: number;
  /** Instante de suelta programada, en s (>= `close_s`). */
  release_s?: number;
  phases: HandPhaseSpec[];
  /** Solo `push`: dirección del empuje en `actor_initial` (se normaliza). */
  axis?: Vec3;
  k_n_per_m?: number;
  c_ns_per_m?: number;
  slack_m?: number;
  max_force_n?: number;
  max_power_w?: number;
  reach_m?: number;
  slip_after_s?: number;
}

/** Punto de la curva de un tirón guiado: en `t_s` la fuerza pedida es `force_n` (en el `frame` del efector). */
export interface PullKeySpec {
  t_s: number;
  force_n: Vec3;
}

/**
 * Tirón guiado (`src/effectors/pull.ts`): mano de Tori cuya FUERZA sigue una curva en el tiempo (interpolación lineal entre
 * puntos; nula antes del primero y desde el último), con tope de fuerza y de potencia. Sirve para tirones en arco, donde lo
 * que se quiere fijar es la dirección de la fuerza y no la posición de la mano.
 */
export interface PullEffector {
  id: string;
  kind: "pull";
  actor: string;
  body: string;
  site?: string;
  local_point_m: Vec3;
  frame: EffectorFrame;
  keys: PullKeySpec[];
  /** Tope de fuerza en N (por defecto 500): ningún punto puede superarlo. */
  max_force_n?: number;
  /** Tope de potencia en W (por defecto 600): si `F · v` lo supera, la fuerza se reduce. */
  max_power_w?: number;
}

export type EffectorSpec = PrescribedForceEffector | HandEffector | PullEffector;

/** Tori abstracto para el presupuesto (`src/effectors/toriBudget.ts`): una masa con dos pies, sin cuerpo. */
export interface ToriSpec {
  mass_kg: number;
  /** Rozamiento de sus pies con el suelo. Por defecto, el de `ground.friction`. */
  friction?: number;
  /** Tope del tiro total de las manos, en N (por defecto 1100). */
  total_pull_max_n?: number;
  /** Tope de la potencia total de las manos, en W (por defecto 1500). */
  total_power_max_w?: number;
  /** Opcional: cuerpo de Tori que ejecuta una secuencia de posturas y hace de obstáculo (`docs/TORI_BODY.md`). */
  body?: ToriBodySpec;
}

/** Postura de Tori en un instante (`src/effectors/toriBody.ts`). Coordenadas en el MUNDO; ángulos en grados. */
export interface ToriPoseKeySpec {
  t_s: number;
  /** Punto medio entre las caderas. */
  pelvis_m: Vec3;
  /** Hacia dónde mira la pelvis (0 = +X, 90 = +Y); no se reduce a ±180. */
  yaw_deg: number;
  /** Inclinación del tronco hacia delante. */
  lean_deg?: number;
  side_lean_deg?: number;
  /** Giro de los hombros respecto a la pelvis. */
  twist_deg?: number;
  /** Apoyo de cada pie; z = altura de la planta. */
  left_foot_m: Vec3;
  right_foot_m: Vec3;
}

/**
 * Cuerpo de Tori: segmentos sueltos con fuerza limitada que siguen `keys` y chocan con Uke. Su masa es `tori.mass_kg`.
 * Con `enabled: false` el escenario se simula sin él (sirve para comparar con y sin cuerpo sin borrar las posturas).
 */
export interface ToriBodySpec {
  height_m: number;
  /** Por defecto `true`. */
  enabled?: boolean;
  /** Rozamiento de su tela contra Uke (por defecto 0,8). */
  friction?: number;
  /** Efectores (por `id`) cuyo punto de agarre siguen las muñecas. */
  hands?: { left?: string; right?: string };
  keys: ToriPoseKeySpec[];
}

/** Formas y superficies del obstáculo (`src/effectors/obstacle.ts`). */
export const OBSTACLE_SHAPES = ["bar", "post", "pad", "foot"] as const;
export type ObstacleShapeKind = (typeof OBSTACLE_SHAPES)[number];
export const OBSTACLE_SURFACES = ["cloth", "skin"] as const;
export type ObstacleSurfaceKind = (typeof OBSTACLE_SURFACES)[number];

/** Tramo de la trayectoria de un obstáculo: desplazamiento acumulado respecto a `start_m` (mínimo tirón). */
export interface ObstaclePhaseSpec {
  start_s: number;
  duration_s: number;
  /** Desplazamiento acumulado en m, en el MUNDO, alcanzado al final del tramo. */
  offset_m: Vec3;
}

/**
 * Obstáculo con masa y fuerza limitada (`src/effectors/obstacle.ts`, `docs/LIMITED_OBSTACLE.md`): la cadera, la pierna o
 * el pie de Tori en abstracto. Es un cuerpo del modelo (se llama como su `id`) que sigue su trayectoria con topes de fuerza;
 * si Uke empuja más, cede. A diferencia de la mano, `start_m` y los desplazamientos están en el MUNDO: el cuerpo se crea
 * antes del asentamiento, cuando aún no existe el marco `actor_initial`. Durante el asentamiento se mantiene en `start_m`.
 * Los campos opcionales ausentes toman los valores por defecto de `OBSTACLE_DEFAULTS` (no se rellenan al validar).
 */
export interface ObstacleSpec {
  /** `[a-z][a-z0-9_]*`; nombre del cuerpo en el modelo y en la grabación. */
  id: string;
  shape: ObstacleShapeKind;
  /** Masa de Tori en kg: fija la masa del obstáculo (la mitad) y los topes por defecto (0,8 y 2,0 pesos). */
  tori_mass_kg: number;
  /** Posición inicial del centro de la geometría en el mundo, en m. */
  start_m: Vec3;
  phases: ObstaclePhaseSpec[];
  mass_kg?: number;
  /** `bar` y `post`: tramo recto y radio. */
  length_m?: number;
  radius_m?: number;
  /** `pad` y `foot`: aristas completas. */
  size_m?: Vec3;
  surface?: ObstacleSurfaceKind;
  friction?: number;
  kp_n_per_m?: number;
  damping_ratio?: number;
  /** Tope horizontal (módulo en XY), en N. */
  max_force_n?: number;
  /** Tope vertical, en N. */
  max_vertical_force_n?: number;
  max_speed_mps?: number;
}

export interface ExpectSpec {
  outcome: Outcome;
  /** Dirección de caída en grados (0 = +X, 90 = +Y). Solo con `outcome: "fall"`. */
  direction_deg?: number;
  /** Tolerancia de la dirección en grados (> 0 y <= 180). */
  tolerance_deg?: number;
}

export interface Scenario {
  schema: typeof SCENARIO_SCHEMA_ID;
  /** Solo `[a-z0-9_]`; igual al nombre del fichero sin extensión. */
  id: string;
  /** Duración del escenario en s (> 0 y <= 30). */
  duration_s: number;
  /** Frecuencia de muestreo de la grabación en Hz (30–500, divisor exacto de 500). */
  record_hz: number;
  ground: GroundSpec;
  actors: ActorSpec[];
  props: PropSpec[];
  effectors: EffectorSpec[];
  /** Obstáculos con fuerza limitada. Opcional: ausente equivale a ninguno (y no cambia el hash de los escenarios anteriores). */
  obstacles?: ObstacleSpec[];
  /** Opcional: con él la grabación lleva `summary.tori_budget`. */
  tori?: ToriSpec;
  expect?: ExpectSpec;
}
