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

export type ActorKind = "rigid_cylinder";
export type PropKind = "fixed_box";
export type EffectorKind = "prescribed_force";

/**
 * Cuerpos referenciables de cada tipo de actor. El resto de fases ampliarán la tabla
 * (por ejemplo, los segmentos del humanoide).
 */
export const ACTOR_BODIES: Readonly<Record<ActorKind, readonly string[]>> = {
  rigid_cylinder: ["cylinder"],
};

/** Marcos en los que se expresa `force_n`. */
export const EFFECTOR_FRAMES = ["world", "actor_initial", "actor_current"] as const;
export type EffectorFrame = (typeof EFFECTOR_FRAMES)[number];

/** Formas temporales de la fuerza prescrita. */
export const EFFECTOR_WAVEFORMS = ["half_sine", "constant"] as const;
export type EffectorWaveform = (typeof EFFECTOR_WAVEFORMS)[number];

/** Resultados posibles del veredicto. */
export const OUTCOMES = ["standing", "tilted", "fall"] as const;
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

export type ActorSpec = RigidCylinderActor;

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
  /** Cuerpo del actor al que se aplica (ver `ACTOR_BODIES`). */
  body: string;
  /** Punto de aplicación en el sistema local del cuerpo, en m. */
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
