/**
 * Obstáculo con masa y fuerza limitada (issue #76, PR 3). Módulo autónomo: no depende de `src/control/`, `src/uke/`,
 * `src/scenario/` ni del ejecutor; solo importa `minimumJerk` de `hand.ts` y la decodificación de contactos de
 * `src/physics/contactForces.ts`. Documentación: `docs/LIMITED_OBSTACLE.md`.
 *
 * Un obstáculo es un cuerpo con masa (cadera, hombro, pierna o pie de Tori en abstracto) montado sobre tres
 * deslizaderas (X, Y, Z) y, si gira, una bisagra. Sigue una trayectoria objetivo con fuerza LIMITADA: nunca es un
 * cuerpo fijo ni `mocap`. Si Uke empuja más que el tope, el obstáculo cede y se mide cuánto.
 *
 * Uso previsto desde un ejecutor (una vez por paso físico, antes de `mj_step`):
 *
 *   const params = resolveObstacleParams({ name: "hip", shape: "bar", tori_mass_kg: 75, start_m: [...], phases: [...] });
 *   // MJCF: appendToWorldbody(mjcf, buildObstacleMjcf(params))   (DESPUÉS del cuerpo de Uke: ver `appendToWorldbody`)
 *   const handles = resolveObstacleHandles(model, params);
 *   let state = createObstacleState();
 *   // en cada paso, tras el controlador de Uke (que pone `qfrc_applied` a cero):
 *   const k = readObstacleKinematics(params, handles, data);
 *   const r = stepObstacle(params, state, { t, dt, ...k, contact_force_on_uke_n });
 *   state = r.state;
 *   applyObstacleCommand(handles, data, r);   // escribe `qfrc_applied` SOLO en los grados de libertad del obstáculo
 *   // registrar r.sample y r.events; r.reaction_n es la reacción de Tori (−F) para el futuro presupuesto
 *   // al terminar: state = settleObstacleWork(state, velocity)
 *
 * Convenciones: unidades SI; todos los vectores están en el mundo (+X delante, +Y izquierda, +Z arriba). La fuerza
 * devuelta es la ORDEN del actuador sobre el obstáculo (incluye la compensación de su propio peso) y se mantiene
 * constante durante el paso `[t, t + dt]`. No incluye la fuerza de contacto, que añade el motor.
 */
import { decodeSolvedContactForce, hasCompleteSolvedContactRows } from "../physics/contactForces";
import { minimumJerk, type Vec3 } from "./hand";

export type { Vec3 };

/** Formas del obstáculo: barra transversal, poste vertical, almohadilla (caja) y pie (caja pequeña). */
export type ObstacleShape = "bar" | "post" | "pad" | "foot";
/** Superficie contra Uke: tela (judogi) o piel. Fija el rozamiento por defecto. */
export type ObstacleSurface = "cloth" | "skin";

/** Gravedad del mundo (`WORLD_OPTION_ATTRS` de `src/scenario/worldOptions.ts`), m/s². */
export const GRAVITY_MPS2 = 9.81;

/** Rozamiento deslizante por defecto de cada superficie. Puntos de partida sin calibrar (issue #76). */
export const OBSTACLE_FRICTION: Readonly<Record<ObstacleSurface, number>> = Object.freeze({ cloth: 0.8, skin: 0.5 });

/** Valores por defecto de la issue #76. Son puntos de partida a calibrar, no verdades (ver `docs/LIMITED_OBSTACLE.md`). */
export const OBSTACLE_DEFAULTS = Object.freeze({
  kp_n_per_m: 20000,
  /** 1 = amortiguación crítica, `c = 2·ζ·sqrt(kp·m)`. */
  damping_ratio: 1,
  /** Masa por defecto del obstáculo, como fracción de la masa de Tori. */
  mass_fraction_of_tori: 0.5,
  /** Tope vertical por defecto, en pesos de Tori. */
  vertical_cap_weights: 2.0,
  /** Tope horizontal por defecto, en pesos de Tori (módulo de la fuerza en XY). */
  horizontal_cap_weights: 0.8,
  /** Brazo de palanca (m) con el que se derivan `kp` y el tope de la bisagra de las de la traslación. */
  rotation_arm_m: 0.15,
});

/** Dimensiones por defecto de cada forma (m). `bar` y `post`: tramo recto y radio; `pad` y `foot`: aristas completas. */
export const OBSTACLE_SHAPE_DEFAULTS = Object.freeze({
  bar: Object.freeze({ length_m: 0.4, radius_m: 0.11 }),
  post: Object.freeze({ length_m: 0.5, radius_m: 0.05 }),
  pad: Object.freeze({ size_m: [0.1, 0.4, 0.4] as Vec3 }),
  foot: Object.freeze({ size_m: [0.24, 0.1, 0.08] as Vec3 }),
});

/**
 * Filtro de colisión: el obstáculo lleva `contype="2" conaffinity="1"`. MuJoCo hace chocar dos geometrías si
 * `(contype_a & conaffinity_b) | (contype_b & conaffinity_a) ≠ 0`. Contra Uke, suelo y props (todos con los valores
 * por defecto 1/1): `2 & 1 = 0` pero `1 & 1 = 1` → chocan. Entre obstáculos: `2 & 1 = 0` en ambos sentidos → no chocan.
 * No se toca ninguna geometría de Uke, así que sus contactos con el suelo no cambian.
 */
export const OBSTACLE_CONTYPE = 2;
export const OBSTACLE_CONAFFINITY = 1;

/** Parámetros de contacto de las geometrías de los escenarios (los de `humanoid_step_v2.xml` y `buildModel.ts`). */
const GEOM_CONTACT = `solref="0.015 1" solimp="0.9 0.95 0.001"`;
const TORSIONAL = 0.005;
const ROLLING = 0.0001;
const EPS_T_S = 1e-9;
/** Un tope solo cuenta como alcanzado si la demanda lo supera en más de esta fracción (evita ruido numérico). */
const SATURATION_REL_TOL = 1e-9;

const ZERO: Vec3 = [0, 0, 0];

function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}
function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}
function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
function norm(a: Vec3): number {
  return Math.sqrt(dot(a, a));
}
function finiteVec(v: Vec3): boolean {
  return Array.isArray(v) && v.length === 3 && Number.isFinite(v[0]) && Number.isFinite(v[1]) && Number.isFinite(v[2]);
}
function requireFiniteVec(name: string, v: Vec3): void {
  if (!finiteVec(v)) throw new RangeError(`${name}: debe ser [x, y, z] con números finitos`);
}
function requirePositive(name: string, v: number): void {
  if (!(Number.isFinite(v) && v > 0)) throw new RangeError(`${name}: debe ser un número finito > 0 (recibido ${v})`);
}
function requireNonNegative(name: string, v: number): void {
  if (!(Number.isFinite(v) && v >= 0)) throw new RangeError(`${name}: debe ser un número finito >= 0 (recibido ${v})`);
}
/** Formato corto y estable de un número en el MJCF. */
function f(x: number): string {
  return String(Number(x.toFixed(6)));
}

// ---------------------------------------------------------------------------------------------------------------------
// Configuración y parámetros
// ---------------------------------------------------------------------------------------------------------------------

/**
 * Tramo de la trayectoria: entre `start_s` y `start_s + duration_s` el desplazamiento respecto a `start_m` pasa, con mínimo
 * tirón (el mismo perfil que `hand.ts`), del valor del tramo anterior (0 al principio) a `offset_m`; el ángulo, de
 * `angle_rad` anterior (0 al principio) al indicado. Fuera de los tramos el objetivo se mantiene en el último valor.
 */
export interface ObstaclePhase {
  readonly start_s: number;
  readonly duration_s: number;
  /** Desplazamiento ACUMULADO respecto a `start_m`, alcanzado al final del tramo. */
  readonly offset_m: Vec3;
  /** Ángulo acumulado de la bisagra (rad). Si falta, se mantiene el anterior. Solo con `rotation`. */
  readonly angle_rad?: number;
}

/** Bisagra opcional del obstáculo (cadera que gira). El eje está en el mundo (las deslizaderas no giran el cuerpo). */
export interface ObstacleRotationConfig {
  /** Eje de giro (no nulo; se normaliza). Por defecto Y (cabeceo). */
  axis?: Vec3;
  /** Rigidez (N·m/rad). Por defecto `kp · rotation_arm_m²`. */
  kp_nm_per_rad?: number;
  /** Tope de par (N·m). Por defecto `max_force_n · rotation_arm_m`. */
  max_torque_nm?: number;
}

export interface ObstacleConfig {
  /** Nombre del cuerpo (`name`), de la geometría (`name_geom`) y de las articulaciones (`name_x`, `_y`, `_z`, `_rot`). */
  name: string;
  shape: ObstacleShape;
  /** Masa de Tori (kg). Obligatoria salvo que se den `mass_kg`, `max_force_n` y `max_vertical_force_n`. */
  tori_mass_kg?: number;
  /** Masa del obstáculo (kg). Por defecto la mitad de la de Tori. */
  mass_kg?: number;
  /** Posición inicial del centro de la geometría en el mundo (m). */
  start_m: Vec3;
  /** `bar` y `post`: longitud del tramo recto (sin las tapas semiesféricas) y radio. */
  length_m?: number;
  radius_m?: number;
  /** `pad` y `foot`: aristas completas de la caja (m), en X, Y, Z. */
  size_m?: Vec3;
  /** Superficie contra Uke. Por defecto `cloth`. */
  surface?: ObstacleSurface;
  /** Rozamiento deslizante explícito; manda sobre `surface`. */
  friction?: number;
  kp_n_per_m?: number;
  damping_ratio?: number;
  /** Tope horizontal (módulo de la fuerza en XY). Por defecto `0,8 × peso de Tori`. En segado y barrido, su tope propio. */
  max_force_n?: number;
  /** Tope vertical (|Fz| de la orden, incluida la compensación del peso propio). Por defecto `2,0 × peso de Tori`. */
  max_vertical_force_n?: number;
  /** Velocidad máxima (m/s) del obstáculo; sin valor no hay límite de velocidad. */
  max_speed_mps?: number;
  /** Compensar el peso propio dentro de la orden (consume tope vertical). Por defecto `true`. */
  gravity_compensation?: boolean;
  /** Añadir `m·a` y `c·v` del objetivo a la orden (sin esto el error de seguimiento es `c·v/kp`). Por defecto `true`. */
  feedforward?: boolean;
  phases?: readonly ObstaclePhase[];
  rotation?: ObstacleRotationConfig;
}

type Geometry =
  | { readonly kind: "capsule"; readonly axis: "y" | "z"; readonly length_m: number; readonly radius_m: number }
  | { readonly kind: "box"; readonly size_m: Vec3 };

export interface ObstacleRotationParams {
  readonly axis: Vec3;
  readonly kp_nm_per_rad: number;
  readonly damping_nms_per_rad: number;
  readonly max_torque_nm: number;
  /** Momento de inercia respecto al eje (kg·m²), de la geometría con masa uniforme. */
  readonly inertia_kgm2: number;
}

/** Parámetros resueltos y validados (todos presentes). */
export interface ObstacleParams {
  readonly name: string;
  readonly shape: ObstacleShape;
  readonly tori_mass_kg: number | null;
  readonly mass_kg: number;
  readonly start_m: Vec3;
  readonly geometry: Geometry;
  readonly surface: ObstacleSurface;
  readonly friction: number;
  readonly kp_n_per_m: number;
  readonly damping_ratio: number;
  /** `2·ζ·sqrt(kp·m)`, N·s/m. */
  readonly damping_ns_per_m: number;
  readonly max_horizontal_force_n: number;
  readonly max_vertical_force_n: number;
  readonly max_speed_mps: number | null;
  readonly gravity_compensation: boolean;
  readonly feedforward: boolean;
  readonly phases: readonly ObstaclePhase[];
  readonly rotation: ObstacleRotationParams | null;
}

/** Inercias principales (kg·m²) en los ejes del cuerpo (X, Y, Z) de una geometría de masa uniforme. */
export function obstacleInertia(geometry: Geometry, mass_kg: number): Vec3 {
  if (geometry.kind === "box") {
    const [a, b, c] = geometry.size_m;
    return [
      (mass_kg / 12) * (b * b + c * c), (mass_kg / 12) * (a * a + c * c), (mass_kg / 12) * (a * a + b * b),
    ];
  }
  const { length_m: h, radius_m: r } = geometry;
  const vCyl = Math.PI * r * r * h;
  const vSph = (4 / 3) * Math.PI * r * r * r;
  const mCyl = (mass_kg * vCyl) / (vCyl + vSph);
  const mSph = mass_kg - mCyl;
  const axial = 0.5 * mCyl * r * r + 0.4 * mSph * r * r;
  const transverse = (mCyl * (3 * r * r + h * h)) / 12 + mSph * (0.4 * r * r + (h * h) / 4 + (3 * h * r) / 8);
  return geometry.axis === "y" ? [transverse, axial, transverse] : [transverse, transverse, axial];
}

function resolveGeometry(config: ObstacleConfig): Geometry {
  const { shape } = config;
  if (shape === "bar" || shape === "post") {
    if (config.size_m !== undefined) throw new RangeError(`size_m: no se admite en ${shape} (usa length_m y radius_m)`);
    const d = OBSTACLE_SHAPE_DEFAULTS[shape];
    const length = config.length_m ?? d.length_m;
    const radius = config.radius_m ?? d.radius_m;
    requirePositive("length_m", length);
    requirePositive("radius_m", radius);
    return { kind: "capsule", axis: shape === "bar" ? "y" : "z", length_m: length, radius_m: radius };
  }
  if (shape === "pad" || shape === "foot") {
    if (config.length_m !== undefined || config.radius_m !== undefined) {
      throw new RangeError(`length_m y radius_m: no se admiten en ${shape} (usa size_m)`);
    }
    const size = config.size_m ?? OBSTACLE_SHAPE_DEFAULTS[shape].size_m;
    requireFiniteVec("size_m", size);
    size.forEach((s, i) => requirePositive(`size_m[${i}]`, s));
    return { kind: "box", size_m: size };
  }
  throw new RangeError(`shape: debe ser bar, post, pad o foot (recibido ${String(shape)})`);
}

/** Valida la configuración y aplica los valores por defecto. Lanza `RangeError` con el campo y el motivo. */
export function resolveObstacleParams(config: ObstacleConfig): ObstacleParams {
  if (typeof config.name !== "string" || !/^[A-Za-z][A-Za-z0-9_]*$/.test(config.name)) {
    throw new RangeError(`name: debe empezar por letra y contener solo letras, cifras y _ (recibido ${String(config.name)})`);
  }
  requireFiniteVec("start_m", config.start_m);
  const geometry = resolveGeometry(config);
  let tori: number | null = null;
  if (config.tori_mass_kg !== undefined) {
    requirePositive("tori_mass_kg", config.tori_mass_kg);
    tori = config.tori_mass_kg;
  }
  const needTori = (field: string): number => {
    if (tori === null) throw new RangeError(`tori_mass_kg: necesaria para el valor por defecto de ${field}`);
    return tori;
  };
  const mass = config.mass_kg ?? OBSTACLE_DEFAULTS.mass_fraction_of_tori * needTori("mass_kg");
  const maxH = config.max_force_n ?? OBSTACLE_DEFAULTS.horizontal_cap_weights * needTori("max_force_n") * GRAVITY_MPS2;
  const maxV = config.max_vertical_force_n ?? OBSTACLE_DEFAULTS.vertical_cap_weights * needTori("max_vertical_force_n") * GRAVITY_MPS2;
  const kp = config.kp_n_per_m ?? OBSTACLE_DEFAULTS.kp_n_per_m;
  const ratio = config.damping_ratio ?? OBSTACLE_DEFAULTS.damping_ratio;
  requirePositive("mass_kg", mass);
  requirePositive("max_force_n", maxH);
  requirePositive("max_vertical_force_n", maxV);
  requirePositive("kp_n_per_m", kp);
  requireNonNegative("damping_ratio", ratio);
  let maxSpeed: number | null = null;
  if (config.max_speed_mps !== undefined) {
    requirePositive("max_speed_mps", config.max_speed_mps);
    maxSpeed = config.max_speed_mps;
  }
  const friction = config.friction ?? OBSTACLE_FRICTION[config.surface ?? "cloth"];
  if (config.surface !== undefined && !(config.surface in OBSTACLE_FRICTION)) {
    throw new RangeError(`surface: debe ser cloth o skin (recibido ${String(config.surface)})`);
  }
  requirePositive("friction", friction);
  const gravityCompensation = config.gravity_compensation ?? true;
  if (gravityCompensation && maxV <= mass * GRAVITY_MPS2) {
    throw new RangeError(
      `max_vertical_force_n (${maxV} N) no supera el peso propio del obstáculo (${(mass * GRAVITY_MPS2).toFixed(1)} N): no podría sostenerse`,
    );
  }

  const phases = [...(config.phases ?? [])];
  let end = 0;
  let angle = 0;
  phases.forEach((phase, i) => {
    requireNonNegative(`phases[${i}].start_s`, phase.start_s);
    requirePositive(`phases[${i}].duration_s`, phase.duration_s);
    requireFiniteVec(`phases[${i}].offset_m`, phase.offset_m);
    if (phase.start_s < end - EPS_T_S) throw new RangeError(`phases[${i}].start_s: solapa el tramo anterior (${end})`);
    if (phase.angle_rad !== undefined) {
      if (config.rotation === undefined) throw new RangeError(`phases[${i}].angle_rad: el obstáculo no tiene rotation`);
      if (!Number.isFinite(phase.angle_rad)) throw new RangeError(`phases[${i}].angle_rad: debe ser finito`);
      angle = phase.angle_rad;
    }
    end = phase.start_s + phase.duration_s;
  });
  void angle;

  const damping = 2 * ratio * Math.sqrt(kp * mass);
  let rotation: ObstacleRotationParams | null = null;
  if (config.rotation !== undefined) {
    let axis: Vec3 = config.rotation.axis ?? [0, 1, 0];
    requireFiniteVec("rotation.axis", axis);
    const length = norm(axis);
    if (!(length > 1e-12)) throw new RangeError("rotation.axis: no puede ser nulo");
    axis = scale(axis, 1 / length);
    const arm = OBSTACLE_DEFAULTS.rotation_arm_m;
    const kr = config.rotation.kp_nm_per_rad ?? kp * arm * arm;
    const maxTorque = config.rotation.max_torque_nm ?? maxH * arm;
    requirePositive("rotation.kp_nm_per_rad", kr);
    requirePositive("rotation.max_torque_nm", maxTorque);
    const diag = obstacleInertia(geometry, mass);
    const inertia = axis[0] * axis[0] * diag[0] + axis[1] * axis[1] * diag[1] + axis[2] * axis[2] * diag[2];
    rotation = {
      axis, kp_nm_per_rad: kr, damping_nms_per_rad: 2 * ratio * Math.sqrt(kr * inertia), max_torque_nm: maxTorque,
      inertia_kgm2: inertia,
    };
  }
  return {
    name: config.name, shape: config.shape, tori_mass_kg: tori, mass_kg: mass, start_m: config.start_m, geometry,
    surface: config.surface ?? "cloth", friction, kp_n_per_m: kp, damping_ratio: ratio, damping_ns_per_m: damping,
    max_horizontal_force_n: maxH, max_vertical_force_n: maxV, max_speed_mps: maxSpeed, gravity_compensation: gravityCompensation,
    feedforward: config.feedforward ?? true, phases, rotation,
  };
}

/**
 * Margen de estabilidad del esquema explícito (la orden se calcula al comienzo del paso): con `a = c·dt/m` y
 * `b = kp·dt²/m`, el seguimiento es estable si `a < 2` y `b < 4 − 2a` (mismo análisis que `docs/VIRTUAL_HAND.md`).
 */
export function obstacleStability(params: ObstacleParams, dt: number): { a: number; b: number; stable: boolean } {
  requirePositive("dt", dt);
  const a = (params.damping_ns_per_m * dt) / params.mass_kg;
  const b = (params.kp_n_per_m * dt * dt) / params.mass_kg;
  return { a, b, stable: a < 2 && b < 4 - 2 * a };
}

// ---------------------------------------------------------------------------------------------------------------------
// Trayectoria objetivo
// ---------------------------------------------------------------------------------------------------------------------

export interface ObstacleTarget {
  readonly position_m: Vec3;
  readonly velocity_mps: Vec3;
  readonly acceleration_mps2: Vec3;
  readonly angle_rad: number;
  readonly angular_velocity_rad_s: number;
  readonly angular_acceleration_rad_s2: number;
}

/** Derivadas del mínimo tirón respecto a `τ`: `s' = 30τ²(1−τ)²`, `s'' = 60τ(1−τ)(1−2τ)`. */
function jerkDerivatives(tau: number): { s1: number; s2: number } {
  const u = Math.min(1, Math.max(0, tau));
  return { s1: 30 * u * u * (1 - u) * (1 - u), s2: 60 * u * (1 - u) * (1 - 2 * u) };
}

/** Posición, velocidad y aceleración del objetivo en el instante `t` (puro; sin estado). */
export function obstacleTarget(
  params: Pick<ObstacleParams, "start_m" | "phases">, t: number,
): ObstacleTarget {
  let prev: Vec3 = ZERO;
  let prevAngle = 0;
  for (const phase of params.phases) {
    const nextAngle = phase.angle_rad ?? prevAngle;
    if (t < phase.start_s) break;
    if (t < phase.start_s + phase.duration_s) {
      const duration = phase.duration_s;
      const tau = (t - phase.start_s) / duration;
      const delta = sub(phase.offset_m, prev);
      const { s1, s2 } = jerkDerivatives(tau);
      const s = minimumJerk(tau);
      const dAngle = nextAngle - prevAngle;
      return {
        position_m: add(params.start_m, add(prev, scale(delta, s))),
        velocity_mps: scale(delta, s1 / duration),
        acceleration_mps2: scale(delta, s2 / (duration * duration)),
        angle_rad: prevAngle + dAngle * s,
        angular_velocity_rad_s: (dAngle * s1) / duration,
        angular_acceleration_rad_s2: (dAngle * s2) / (duration * duration),
      };
    }
    prev = phase.offset_m;
    prevAngle = nextAngle;
  }
  return {
    position_m: add(params.start_m, prev), velocity_mps: ZERO, acceleration_mps2: ZERO,
    angle_rad: prevAngle, angular_velocity_rad_s: 0, angular_acceleration_rad_s2: 0,
  };
}

/** Velocidad máxima (m/s) de la trayectoria planificada: `1,875·|Δ|/T` del tramo más rápido (cada tramo parte y acaba en reposo). */
export function plannedPeakSpeedMps(phases: readonly ObstaclePhase[]): number {
  let prev: Vec3 = ZERO;
  let peak = 0;
  for (const phase of phases) {
    peak = Math.max(peak, (1.875 * norm(sub(phase.offset_m, prev))) / phase.duration_s);
    prev = phase.offset_m;
  }
  return peak;
}

// ---------------------------------------------------------------------------------------------------------------------
// Estado, paso y muestras
// ---------------------------------------------------------------------------------------------------------------------

export type ObstacleEventKind = "saturated" | "recovered";

/** Suceso: el seguimiento entra en saturación (`saturated`) o vuelve a la zona lineal (`recovered`). */
export interface ObstacleEvent {
  readonly t: number;
  readonly kind: ObstacleEventKind;
}

export interface ObstacleTotals {
  /** Trabajo de la orden del actuador sobre el obstáculo (J): suma de `F · v̄ · dt` (trapecio), con la compensación del peso. */
  readonly work_j: number;
  /** Impulso de la orden (N·s). */
  readonly impulse_ns: Vec3;
  /** Impulso acumulado de la reacción de Tori, `−∫F dt` (N·s). */
  readonly reaction_impulse_ns: Vec3;
  /** Impulso de la fuerza de contacto sobre Uke (N·s), de `contact_force_on_uke_n`. */
  readonly contact_impulse_on_uke_ns: Vec3;
  readonly max_force_n: number;
  readonly max_horizontal_force_n: number;
  readonly max_vertical_force_n: number;
  readonly max_contact_force_on_uke_n: number;
  readonly max_deviation_m: number;
  readonly max_speed_mps: number;
  readonly time_saturated_s: number;
  readonly time_speed_limited_s: number;
}

export interface ObstacleState {
  readonly saturated: boolean;
  /** Orden, velocidad y paso del último `stepObstacle`, para cerrar el trabajo con la velocidad media. */
  readonly pending: { readonly force_n: Vec3; readonly velocity_mps: Vec3; readonly dt: number } | null;
  readonly totals: ObstacleTotals;
  readonly events: readonly ObstacleEvent[];
}

export interface ObstacleStepInput {
  /** Instante (s) en que se miden posición y velocidad: el comienzo del paso `[t, t + dt]`. */
  t: number;
  dt: number;
  /** Posición del centro de la geometría en el mundo (m): `start_m` más las deslizaderas. */
  position_m: Vec3;
  velocity_mps: Vec3;
  /** Solo con `rotation`. */
  angle_rad?: number;
  angular_velocity_rad_s?: number;
  /** Fuerza de contacto sobre Uke medida por el llamante (normalmente la del paso anterior); solo se registra. */
  contact_force_on_uke_n?: Vec3;
}

/** Datos para registrar por paso. */
export interface ObstacleSample {
  readonly t: number;
  readonly target_m: Vec3;
  readonly position_m: Vec3;
  /** `posición − objetivo`. */
  readonly deviation_m: Vec3;
  readonly deviation_magnitude_m: number;
  readonly speed_mps: number;
  /** Orden aplicada (tras la limitación de velocidad y los topes), con la compensación del peso. */
  readonly force_n: Vec3;
  /** Orden antes de la limitación de velocidad y de los topes. */
  readonly demand_force_n: Vec3;
  /** Algún tope (horizontal, vertical o de par) recortó la orden: el seguimiento está saturado. */
  readonly saturated: boolean;
  readonly horizontal_saturated: boolean;
  readonly vertical_saturated: boolean;
  readonly torque_saturated: boolean;
  /** El limitador de velocidad modificó la orden. */
  readonly speed_limited: boolean;
  readonly torque_nm: number;
  readonly angle_deviation_rad: number;
  readonly contact_force_on_uke_n: Vec3;
  /** Reacción de Tori: `−F`. */
  readonly reaction_n: Vec3;
  /** Trabajo acumulado (J) hasta el comienzo de este paso. */
  readonly work_j: number;
}

export interface ObstacleStepResult {
  readonly force_n: Vec3;
  /** Par de la bisagra (N·m); 0 sin `rotation`. */
  readonly torque_nm: number;
  readonly reaction_n: Vec3;
  readonly state: ObstacleState;
  readonly sample: ObstacleSample;
  readonly events: readonly ObstacleEvent[];
}

export function createObstacleState(): ObstacleState {
  return {
    saturated: false, pending: null, events: [],
    totals: {
      work_j: 0, impulse_ns: ZERO, reaction_impulse_ns: ZERO, contact_impulse_on_uke_ns: ZERO, max_force_n: 0,
      max_horizontal_force_n: 0, max_vertical_force_n: 0, max_contact_force_on_uke_n: 0, max_deviation_m: 0,
      max_speed_mps: 0, time_saturated_s: 0, time_speed_limited_s: 0,
    },
  };
}

/**
 * Un paso del obstáculo (función pura: no modifica `state` ni `params`).
 *
 * Orden: trabajo del paso anterior; objetivo `p_t(t)`; orden de seguimiento
 * `F = kp·(p_t − p) + c·(v_t − v) + m·a_t + m·g·ẑ` (los dos últimos términos, con `feedforward` y `gravity_compensation`);
 * limitador de velocidad (si `max_speed_mps`); tope horizontal (módulo en XY) y tope vertical (`|Fz|`).
 */
export function stepObstacle(params: ObstacleParams, state: ObstacleState, input: ObstacleStepInput): ObstacleStepResult {
  const { t, dt, position_m: p, velocity_mps: v } = input;
  if (!Number.isFinite(t)) throw new RangeError("t: debe ser finito");
  requirePositive("dt", dt);
  requireFiniteVec("position_m", p);
  requireFiniteVec("velocity_mps", v);
  const contact = input.contact_force_on_uke_n ?? ZERO;
  requireFiniteVec("contact_force_on_uke_n", contact);
  const m = params.mass_kg;

  // Trabajo del paso anterior con la velocidad media (trapecio), como `hand.ts`.
  let work = state.totals.work_j;
  if (state.pending) {
    const mean = scale(add(state.pending.velocity_mps, v), 0.5);
    work += dot(state.pending.force_n, mean) * state.pending.dt;
  }

  const target = obstacleTarget(params, t);
  const e = sub(target.position_m, p);
  const ev = sub(target.velocity_mps, v);
  const kp = params.kp_n_per_m;
  const c = params.damping_ns_per_m;
  let demand: Vec3 = add(scale(e, kp), scale(ev, c));
  if (params.feedforward) demand = add(demand, scale(target.acceleration_mps2, m));
  if (params.gravity_compensation) demand = add(demand, [0, 0, m * GRAVITY_MPS2]);

  // Limitador de velocidad: la aceleración que daría la orden (con la gravedad real) no puede llevar |v| por encima del tope.
  let force = demand;
  let speedLimited = false;
  if (params.max_speed_mps !== null) {
    const accel: Vec3 = [force[0] / m, force[1] / m, force[2] / m - GRAVITY_MPS2];
    const next = add(v, scale(accel, dt));
    const nextSpeed = norm(next);
    if (nextSpeed > params.max_speed_mps) {
      const desired = scale(next, params.max_speed_mps / nextSpeed);
      const aDesired = scale(sub(desired, v), 1 / dt);
      force = scale([aDesired[0], aDesired[1], aDesired[2] + GRAVITY_MPS2], m);
      speedLimited = true;
    }
  }

  // Topes: horizontal (módulo en XY) y vertical (|Fz|).
  let horizontalSaturated = false;
  let verticalSaturated = false;
  const horizontal = Math.hypot(force[0], force[1]);
  if (horizontal > params.max_horizontal_force_n * (1 + SATURATION_REL_TOL)) {
    const k = params.max_horizontal_force_n / horizontal;
    force = [force[0] * k, force[1] * k, force[2]];
    horizontalSaturated = true;
  }
  if (Math.abs(force[2]) > params.max_vertical_force_n * (1 + SATURATION_REL_TOL)) {
    force = [force[0], force[1], Math.sign(force[2]) * params.max_vertical_force_n];
    verticalSaturated = true;
  }

  // Bisagra: PD con la aceleración del objetivo y tope de par.
  let torque = 0;
  let torqueSaturated = false;
  let angleDeviation = 0;
  if (params.rotation !== null) {
    const r = params.rotation;
    const angle = input.angle_rad ?? 0;
    const omega = input.angular_velocity_rad_s ?? 0;
    angleDeviation = angle - target.angle_rad;
    let tau = r.kp_nm_per_rad * (target.angle_rad - angle) + r.damping_nms_per_rad * (target.angular_velocity_rad_s - omega);
    if (params.feedforward) tau += r.inertia_kgm2 * target.angular_acceleration_rad_s2;
    if (Math.abs(tau) > r.max_torque_nm * (1 + SATURATION_REL_TOL)) {
      tau = Math.sign(tau) * r.max_torque_nm;
      torqueSaturated = true;
    }
    torque = tau;
  }

  const saturated = horizontalSaturated || verticalSaturated || torqueSaturated;
  const events: ObstacleEvent[] = [];
  if (saturated && !state.saturated) events.push({ t, kind: "saturated" });
  if (!saturated && state.saturated) events.push({ t, kind: "recovered" });

  const magnitude = norm(force);
  const contactMagnitude = norm(contact);
  const speed = norm(v);
  const deviation = sub(p, target.position_m);
  const deviationMagnitude = norm(deviation);
  const impulse = add(state.totals.impulse_ns, scale(force, dt));
  const totals: ObstacleTotals = {
    work_j: work,
    impulse_ns: impulse,
    reaction_impulse_ns: scale(impulse, -1),
    contact_impulse_on_uke_ns: add(state.totals.contact_impulse_on_uke_ns, scale(contact, dt)),
    max_force_n: Math.max(state.totals.max_force_n, magnitude),
    max_horizontal_force_n: Math.max(state.totals.max_horizontal_force_n, Math.hypot(force[0], force[1])),
    max_vertical_force_n: Math.max(state.totals.max_vertical_force_n, Math.abs(force[2])),
    max_contact_force_on_uke_n: Math.max(state.totals.max_contact_force_on_uke_n, contactMagnitude),
    max_deviation_m: Math.max(state.totals.max_deviation_m, deviationMagnitude),
    max_speed_mps: Math.max(state.totals.max_speed_mps, speed),
    time_saturated_s: state.totals.time_saturated_s + (saturated ? dt : 0),
    time_speed_limited_s: state.totals.time_speed_limited_s + (speedLimited ? dt : 0),
  };
  const next: ObstacleState = {
    saturated,
    pending: { force_n: force, velocity_mps: v, dt },
    totals,
    events: events.length > 0 ? [...state.events, ...events] : state.events,
  };
  const reaction = scale(force, -1);
  const sample: ObstacleSample = {
    t, target_m: target.position_m, position_m: p, deviation_m: deviation, deviation_magnitude_m: deviationMagnitude,
    speed_mps: speed, force_n: force, demand_force_n: demand, saturated, horizontal_saturated: horizontalSaturated,
    vertical_saturated: verticalSaturated, torque_saturated: torqueSaturated, speed_limited: speedLimited,
    torque_nm: torque, angle_deviation_rad: angleDeviation, contact_force_on_uke_n: contact, reaction_n: reaction,
    work_j: work,
  };
  return { force_n: force, torque_nm: torque, reaction_n: reaction, state: next, sample, events };
}

/** Cierra el trabajo del último paso con la velocidad tras él. Llamar una vez, al terminar la simulación. */
export function settleObstacleWork(state: ObstacleState, velocity_mps: Vec3): ObstacleState {
  requireFiniteVec("velocity_mps", velocity_mps);
  if (!state.pending) return state;
  const mean = scale(add(state.pending.velocity_mps, velocity_mps), 0.5);
  const work = state.totals.work_j + dot(state.pending.force_n, mean) * state.pending.dt;
  return { ...state, pending: null, totals: { ...state.totals, work_j: work } };
}

// ---------------------------------------------------------------------------------------------------------------------
// MJCF
// ---------------------------------------------------------------------------------------------------------------------

/**
 * Fragmento MJCF (un `<body>`) del obstáculo, para insertar dentro de `<worldbody>`: tres deslizaderas X, Y, Z sin
 * límites ni amortiguación ni armadura (`name_x`, `name_y`, `name_z`), una bisagra opcional (`name_rot`) y una geometría
 * de masa `mass_kg`. Todos los atributos de contacto van explícitos, así que no dependen de ningún `<default>` del
 * modelo en que se inserte. `priority="1"` hace que el rozamiento del obstáculo mande sobre el de Uke y el suelo
 * (sin él MuJoCo toma el MAYOR de los dos y `cloth`/`skin` no tendrían efecto contra un Uke de rozamiento 1,0).
 */
export function buildObstacleMjcf(params: ObstacleParams): string {
  const [x, y, z] = params.start_m;
  const { name, geometry: g } = params;
  const joints = (["x", "y", "z"] as const).map((a, i) => {
    const axis = ["1 0 0", "0 1 0", "0 0 1"][i];
    return `      <joint name="${name}_${a}" type="slide" axis="${axis}" limited="false" damping="0" armature="0"/>`;
  });
  if (params.rotation) {
    const [ax, ay, az] = params.rotation.axis;
    joints.push(`      <joint name="${name}_rot" type="hinge" axis="${f(ax)} ${f(ay)} ${f(az)}" limited="false" damping="0" armature="0"/>`);
  }
  let shapeAttrs: string;
  if (g.kind === "capsule") {
    const h = g.length_m / 2;
    const fromto = g.axis === "y" ? `0 ${f(-h)} 0 0 ${f(h)} 0` : `0 0 ${f(-h)} 0 0 ${f(h)}`;
    shapeAttrs = `type="capsule" fromto="${fromto}" size="${f(g.radius_m)}"`;
  } else {
    shapeAttrs = `type="box" size="${f(g.size_m[0] / 2)} ${f(g.size_m[1] / 2)} ${f(g.size_m[2] / 2)}"`;
  }
  return `    <body name="${name}" pos="${f(x)} ${f(y)} ${f(z)}">
${joints.join("\n")}
      <geom name="${name}_geom" ${shapeAttrs} mass="${f(params.mass_kg)}" rgba="0.93 0.55 0.2 1"
            friction="${f(params.friction)} ${TORSIONAL} ${ROLLING}" condim="3" ${GEOM_CONTACT}
            contype="${OBSTACLE_CONTYPE}" conaffinity="${OBSTACLE_CONAFFINITY}" priority="1"/>
    </body>
`;
}

/**
 * Inserta XML antes del último `</worldbody>`. Los obstáculos deben ir DESPUÉS del cuerpo de Uke: el controlador
 * `tense` y el ejecutor suponen que la pelvis es el cuerpo 1 (`subtree_com[3]`, `subtree_linvel[3]`) y que los seis primeros
 * grados de libertad son la raíz libre (`qfrc_applied[0..5]`). `generateHumanoidMJCF(..., { extraWorldXml })` inserta ANTES
 * de la pelvis, y por eso no sirve para los obstáculos (ver `docs/LIMITED_OBSTACLE.md`).
 */
export function appendToWorldbody(mjcf: string, xml: string): string {
  const at = mjcf.lastIndexOf("</worldbody>");
  if (at < 0) throw new Error("El MJCF no tiene </worldbody>");
  return `${mjcf.slice(0, at)}${xml}  ${mjcf.slice(at)}`;
}

// ---------------------------------------------------------------------------------------------------------------------
// Adaptadores a MuJoCo (tipos estructurales: no importan el módulo WASM)
// ---------------------------------------------------------------------------------------------------------------------

/** Lo mínimo de `MjModel` que usan los adaptadores. */
export interface ObstacleModelLike {
  jnt(name: string): { id: number };
  geom(name: string): { id: number };
  readonly jnt_qposadr: ArrayLike<number>;
  readonly jnt_dofadr: ArrayLike<number>;
  readonly geom_bodyid: ArrayLike<number>;
}

export interface ObstacleHandles {
  readonly name: string;
  /** Direcciones en `qpos` y `qvel`/`qfrc_applied` de las deslizaderas X, Y, Z. */
  readonly qposAdr: readonly [number, number, number];
  readonly dofAdr: readonly [number, number, number];
  readonly rotation: { readonly qposAdr: number; readonly dofAdr: number } | null;
  readonly geomId: number;
  readonly bodyId: number;
}

export function resolveObstacleHandles(model: ObstacleModelLike, params: ObstacleParams): ObstacleHandles {
  const joint = (suffix: string): { qposAdr: number; dofAdr: number } => {
    const id = model.jnt(`${params.name}_${suffix}`).id;
    return { qposAdr: model.jnt_qposadr[id]!, dofAdr: model.jnt_dofadr[id]! };
  };
  const [jx, jy, jz] = [joint("x"), joint("y"), joint("z")];
  const geomId = model.geom(`${params.name}_geom`).id;
  return {
    name: params.name,
    qposAdr: [jx.qposAdr, jy.qposAdr, jz.qposAdr],
    dofAdr: [jx.dofAdr, jy.dofAdr, jz.dofAdr],
    rotation: params.rotation ? joint("rot") : null,
    geomId,
    bodyId: model.geom_bodyid[geomId]!,
  };
}

/** Posición y velocidad del obstáculo desde `qpos` y `qvel` (el cuerpo nace en `start_m` con las deslizaderas a 0). */
export function readObstacleKinematics(
  params: ObstacleParams, handles: ObstacleHandles, data: { readonly qpos: ArrayLike<number>; readonly qvel: ArrayLike<number> },
): Pick<ObstacleStepInput, "position_m" | "velocity_mps" | "angle_rad" | "angular_velocity_rad_s"> {
  const q = (i: number): number => data.qpos[handles.qposAdr[i]!]!;
  const v = (i: number): number => data.qvel[handles.dofAdr[i]!]!;
  const out: Pick<ObstacleStepInput, "position_m" | "velocity_mps" | "angle_rad" | "angular_velocity_rad_s"> = {
    position_m: [params.start_m[0] + q(0), params.start_m[1] + q(1), params.start_m[2] + q(2)],
    velocity_mps: [v(0), v(1), v(2)],
  };
  if (handles.rotation) {
    out.angle_rad = data.qpos[handles.rotation.qposAdr]!;
    out.angular_velocity_rad_s = data.qvel[handles.rotation.dofAdr]!;
  }
  return out;
}

/**
 * Escribe la orden en `qfrc_applied` de los grados de libertad del obstáculo (asignación, no suma: cada obstáculo es
 * dueño de los suyos). Hay que llamarla DESPUÉS del controlador de Uke, que pone todo `qfrc_applied` a cero.
 * Fuerza generalizada por grado de libertad: sin el acoplamiento por el jacobiano que tendría una fuerza cartesiana en
 * `xfrc_applied` cuando hay bisagra (la fuerza en el centro de masas movería también la bisagra si su eje no pasa por él).
 */
export function applyObstacleCommand(
  handles: ObstacleHandles, data: { readonly qfrc_applied: Float64Array }, command: { force_n: Vec3; torque_nm: number },
): void {
  if (!finiteVec(command.force_n) || !Number.isFinite(command.torque_nm)) throw new RangeError("orden no finita");
  for (let i = 0; i < 3; i++) data.qfrc_applied[handles.dofAdr[i]!] = command.force_n[i]!;
  if (handles.rotation) data.qfrc_applied[handles.rotation.dofAdr] = command.torque_nm;
}

/** Contacto de MuJoCo con lo que leen los adaptadores (como en `src/scenario/run.ts`). */
interface ContactLike {
  readonly geom1: number;
  readonly geom2: number;
  readonly efc_address: number;
  readonly dim: number;
  readonly friction: ArrayLike<number>;
  readonly adhesion: number;
  readonly frame: ArrayLike<number>;
  delete(): void;
}

export interface ObstacleContactData {
  readonly contact: { size(): number; get(i: number): ContactLike | null | undefined; delete(): void };
  readonly efc_force: ArrayLike<number>;
  readonly nefc: number;
}

export interface ObstacleContactForce {
  /** Suma de las fuerzas de contacto del obstáculo SOBRE los otros cuerpos aceptados (Uke, props), en el mundo (N). */
  readonly force_on_other_n: Vec3;
  /** Número de contactos con fuerza normal > 0 contados. */
  readonly count: number;
}

/**
 * Fuerza de contacto que el obstáculo ejerce sobre otros cuerpos, de la fuerza RESUELTA (`efc_force`, como el ejecutor:
 * `mj_contactForce` no devuelve bien su salida en este WASM). El eje normal del contacto va de `geom1` a `geom2` y la
 * fuerza del marco actúa sobre `geom2`; si el obstáculo es `geom2`, la que recibe Uke es la opuesta.
 * `isOtherBody` decide qué cuerpos cuentan como Uke; por defecto, cualquiera que no sea el suelo (cuerpo 0) ni el obstáculo.
 * Los contactos con el suelo (cuerpo 0) no cuentan por defecto: no son fuerza sobre Uke.
 * Tras `mj_step`, los contactos y `efc_force` son los del paso que acaba de ejecutarse (los que actuaron con la orden dada).
 */
export function readObstacleContactForce(
  model: Pick<ObstacleModelLike, "geom_bodyid">, handles: ObstacleHandles, data: ObstacleContactData, options: {
    pyramidal: boolean;
    isOtherBody?: (bodyId: number) => boolean;
  },
): ObstacleContactForce {
  const isOther = options.isOtherBody ?? ((b: number) => b !== 0 && b !== handles.bodyId);
  let total: Vec3 = ZERO;
  let count = 0;
  const contacts = data.contact;
  try {
    for (let c = 0; c < contacts.size(); c++) {
      const contact = contacts.get(c);
      if (!contact) throw new Error("Contacto MuJoCo inaccesible");
      try {
        const body1 = model.geom_bodyid[contact.geom1]!;
        const body2 = model.geom_bodyid[contact.geom2]!;
        const own1 = body1 === handles.bodyId;
        const own2 = body2 === handles.bodyId;
        if (own1 === own2) continue; // ninguno es del obstáculo (o es contacto consigo mismo)
        if (!isOther(own1 ? body2 : body1)) continue;
        if (!hasCompleteSolvedContactRows(contact.efc_address, contact.dim, options.pyramidal, data.nefc, data.efc_force.length)) continue;
        const solved = decodeSolvedContactForce({
          solverRows: data.efc_force, efcAddress: contact.efc_address, dim: contact.dim, pyramidal: options.pyramidal,
          friction: contact.friction, adhesionN: contact.adhesion,
        });
        if (!(solved.normalN > 0)) continue;
        const frame = contact.frame;
        const [t1, t2] = solved.tangentN;
        const onGeom2: Vec3 = [
          solved.normalN * frame[0]! + t1 * frame[3]! + t2 * frame[6]!,
          solved.normalN * frame[1]! + t1 * frame[4]! + t2 * frame[7]!,
          solved.normalN * frame[2]! + t1 * frame[5]! + t2 * frame[8]!,
        ];
        // `onGeom2` actúa sobre geom2. Uke es geom2 si el obstáculo es geom1.
        total = add(total, own1 ? onGeom2 : scale(onGeom2, -1));
        count++;
      } finally {
        contact.delete();
      }
    }
  } finally {
    contacts.delete();
  }
  return { force_on_other_n: total, count };
}
