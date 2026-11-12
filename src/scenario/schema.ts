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
export type EffectorKind = "prescribed_force";

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

export type EffectorSpec = PrescribedForceEffector;

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
  expect?: ExpectSpec;
}
