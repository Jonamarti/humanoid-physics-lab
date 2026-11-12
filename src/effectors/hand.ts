/**
 * Mano virtual (issue #76, PR 1). Módulo autónomo y puro: no depende de `src/control/`, `src/physics/` ni del
 * ejecutor de escenarios. Documentación: `docs/VIRTUAL_HAND.md`.
 *
 * Una mano es un punto objetivo `p_t(t)` unido por muelle y amortiguador a un punto de agarre `p_g` del cuerpo de Uke.
 * Uso previsto desde un ejecutor (una vez por paso físico, antes de `mj_step`):
 *
 *   const params = resolveHandParams({ link: "rope", shoulder_m: [...], phases: [...] });
 *   let state = createHandState();
 *   // en cada paso, con la posición y la velocidad del punto de agarre en el instante `t`:
 *   const r = stepHand(params, state, { t, dt, grip_m, grip_velocity_mps });
 *   state = r.state;
 *   applyForceAtPoint(data, bodyId, grip_m, r.force_n);  // misma matemática r × F que el ejecutor
 *   // registrar r.sample y r.events; r.reaction_n es la reacción de Tori (−F) para el presupuesto
 *   // al terminar: state = settleHandWork(state, grip_velocity_mps) cierra el trabajo del último paso.
 *
 * Convenciones: unidades SI; todos los vectores (posiciones, desplazamientos, hombro, eje) están en el mismo marco,
 * que el llamante elige (normalmente el mundo); este módulo no convierte entre marcos. La fuerza devuelta es la que
 * actúa SOBRE el punto de agarre y se mantiene constante durante el paso `[t, t + dt]`.
 */

export type Vec3 = readonly [number, number, number];

/** Tipos de unión: solo tracción en la línea mano–agarre, tracción y empuje, o solo empuje. */
export type HandLink = "rope" | "clamp" | "push";

/** Valores por defecto de la issue #76. Son puntos de partida a calibrar, no verdades (ver `docs/VIRTUAL_HAND.md`). */
export const HAND_DEFAULTS = Object.freeze({
  k_n_per_m: 3000,
  c_ns_per_m: 200,
  /** Holgura por defecto de `rope`; `clamp` y `push` usan 0. */
  rope_slack_m: 0.03,
  max_force_n: 500,
  max_power_w: 600,
  reach_m: 0.75,
  slip_after_s: 0.4,
});

/**
 * Tramo del objetivo: entre `start_s` y `start_s + duration_s` el desplazamiento respecto al punto de agarre al cerrarse
 * la mano pasa, con mínimo tirón (minimum jerk), del valor del tramo anterior (0 al principio) a `offset_m`.
 * Fuera de los tramos el objetivo se mantiene en el último desplazamiento alcanzado.
 */
export interface HandPhase {
  /** Instante de inicio en s (mismo reloj que `t` de `stepHand`; no puede ser anterior a `close_s`). */
  readonly start_s: number;
  /** Duración en s (> 0). */
  readonly duration_s: number;
  /** Desplazamiento acumulado respecto al punto de agarre al cerrarse la mano, alcanzado al final del tramo. */
  readonly offset_m: Vec3;
}

/** Configuración de una mano; los campos opcionales toman los valores de `HAND_DEFAULTS`. */
export interface HandConfig {
  link: HandLink;
  /** Centro de la esfera de alcance (hombro de Tori), en m. Obligatorio. */
  shoulder_m: Vec3;
  /** Instante en que la mano se cierra sobre el agarre y empieza a actuar. Por defecto 0. */
  close_s?: number;
  /** Instante de suelta programada. Por defecto, no hay. */
  release_s?: number;
  phases?: readonly HandPhase[];
  /**
   * Solo `push`: dirección en la que empuja (no nula; se normaliza). Por defecto, la del hombro al punto de agarre en
   * el instante de cerrarse la mano (se aleja de Tori). Se ignora en `rope` y `clamp` (error si se indica).
   */
  axis?: Vec3;
  k_n_per_m?: number;
  c_ns_per_m?: number;
  /** Holgura en m. Por defecto 0,03 en `rope` y 0 en `push`. `clamp` no tiene holgura (error si es distinta de 0). */
  slack_m?: number;
  max_force_n?: number;
  max_power_w?: number;
  reach_m?: number;
  slip_after_s?: number;
}

/** Parámetros resueltos y validados (todos presentes). */
export interface HandParams {
  readonly link: HandLink;
  readonly shoulder_m: Vec3;
  readonly close_s: number;
  readonly release_s: number | null;
  readonly phases: readonly HandPhase[];
  readonly axis: Vec3 | null;
  readonly k_n_per_m: number;
  readonly c_ns_per_m: number;
  readonly slack_m: number;
  readonly max_force_n: number;
  readonly max_power_w: number;
  readonly reach_m: number;
  readonly slip_after_s: number;
}

export type HandEventKind = "close" | "release" | "slip";

/** Suceso de la mano: cierre, suelta programada o resbalón (fuerza en el tope más de `slip_after_s`). */
export interface HandEvent {
  readonly t: number;
  readonly kind: HandEventKind;
}

export interface HandTotals {
  /** Trabajo aportado por la fuerza sobre el punto de agarre (J): suma de `F · v̄ · dt` con la velocidad media del paso. */
  readonly work_j: number;
  /** Impulso de la fuerza sobre el agarre (N·s). */
  readonly impulse_ns: Vec3;
  /** Impulso acumulado de la reacción de Tori, `−∫F dt` (N·s). */
  readonly reaction_impulse_ns: Vec3;
  readonly max_force_n: number;
  /** Máximo de `F · v` observado (W); 0 si nunca fue positivo. */
  readonly max_power_w: number;
  /** Tiempo total, en s, con la demanda del muelle en el tope de fuerza. */
  readonly time_at_cap_s: number;
}

export interface HandState {
  readonly phase: "open" | "holding" | "released";
  /** Punto de agarre en el instante del cierre (origen de los desplazamientos). */
  readonly origin_m: Vec3 | null;
  /** Eje de empuje resuelto (solo `push`, tras el cierre). */
  readonly axis: Vec3 | null;
  /** Tiempo continuo, en s, que la demanda lleva en el tope de fuerza. */
  readonly at_cap_s: number;
  /** Fuerza, velocidad y paso del último `stepHand`, para cerrar el trabajo con la velocidad media. */
  readonly pending: { readonly force_n: Vec3; readonly velocity_mps: Vec3; readonly dt: number } | null;
  readonly totals: HandTotals;
  readonly events: readonly HandEvent[];
}

export interface HandStepInput {
  /** Instante (s) en que se miden la posición y la velocidad: el comienzo del paso `[t, t + dt]`. */
  t: number;
  /** Paso físico en s (> 0). */
  dt: number;
  /** Posición del punto de agarre en el mundo, en m. */
  grip_m: Vec3;
  /** Velocidad del punto de agarre en el mundo, en m/s. */
  grip_velocity_mps: Vec3;
}

/** Datos para registrar por paso. */
export interface HandSample {
  readonly t: number;
  /** `true` si la mano está cerrada y no soltada en este paso. */
  readonly held: boolean;
  /** Objetivo tras recortarlo al alcance; `null` si la mano no sujeta. */
  readonly target_m: Vec3 | null;
  /** Objetivo antes de recortarlo al alcance; `null` si la mano no sujeta. */
  readonly target_unclipped_m: Vec3 | null;
  /** Fuerza aplicada al punto de agarre. */
  readonly force_n: Vec3;
  readonly force_magnitude_n: number;
  /** Módulo de la fuerza del muelle antes de los topes. */
  readonly demand_force_n: number;
  /** `F · v_g` con la fuerza aplicada (W). */
  readonly power_w: number;
  /** La demanda llegó al tope de fuerza (la fuerza se recortó a `max_force_n`). */
  readonly force_limited: boolean;
  /** El tope de potencia redujo la fuerza. */
  readonly power_limited: boolean;
  /** El objetivo se recortó a la esfera de alcance. */
  readonly reach_limited: boolean;
  /** Trabajo acumulado (J) hasta el comienzo de este paso. */
  readonly work_j: number;
  /** Reacción de Tori: `−F`. */
  readonly reaction_n: Vec3;
}

export interface HandStepResult {
  /** Fuerza que debe aplicarse al punto de agarre durante `[t, t + dt]`. */
  readonly force_n: Vec3;
  /** Reacción de Tori, `−force_n`. */
  readonly reaction_n: Vec3;
  readonly state: HandState;
  readonly sample: HandSample;
  /** Sucesos de este paso (normalmente vacío). */
  readonly events: readonly HandEvent[];
}

const ZERO: Vec3 = [0, 0, 0];
/** Tolerancia de comparación de instantes (s), para que `i * dt` con error de redondeo no retrase un suceso un paso. */
const EPS_T_S = 1e-9;

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
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
function finiteVec(v: Vec3): boolean {
  return v.length === 3 && Number.isFinite(v[0]) && Number.isFinite(v[1]) && Number.isFinite(v[2]);
}

function requireFiniteVec(name: string, v: Vec3): void {
  if (!Array.isArray(v) || !finiteVec(v)) throw new RangeError(`${name}: debe ser [x, y, z] con números finitos`);
}
function requirePositive(name: string, v: number): void {
  if (!(Number.isFinite(v) && v > 0)) throw new RangeError(`${name}: debe ser un número finito > 0 (recibido ${v})`);
}
function requireNonNegative(name: string, v: number): void {
  if (!(Number.isFinite(v) && v >= 0)) throw new RangeError(`${name}: debe ser un número finito >= 0 (recibido ${v})`);
}

/** Perfil de mínimo tirón: `10τ³ − 15τ⁴ + 6τ⁵`, de 0 a 1 con velocidad y aceleración nulas en los extremos. */
export function minimumJerk(tau: number): number {
  const u = Math.min(1, Math.max(0, tau));
  return u * u * u * (10 + u * (-15 + 6 * u));
}

/** Desplazamiento del objetivo respecto al punto de agarre al cerrarse la mano, en el instante `t` (puro). */
export function handTargetOffset(phases: readonly HandPhase[], t: number): Vec3 {
  let previous: Vec3 = ZERO;
  for (const phase of phases) {
    if (t < phase.start_s) return previous;
    if (t < phase.start_s + phase.duration_s) {
      const s = minimumJerk((t - phase.start_s) / phase.duration_s);
      return add(previous, scale(sub(phase.offset_m, previous), s));
    }
    previous = phase.offset_m;
  }
  return previous;
}

/** Valida la configuración y aplica los valores por defecto. Lanza `RangeError` con el campo y el motivo. */
export function resolveHandParams(config: HandConfig): HandParams {
  if (config.link !== "rope" && config.link !== "clamp" && config.link !== "push") {
    throw new RangeError(`link: debe ser rope, clamp o push (recibido ${String(config.link)})`);
  }
  requireFiniteVec("shoulder_m", config.shoulder_m);
  const k = config.k_n_per_m ?? HAND_DEFAULTS.k_n_per_m;
  const c = config.c_ns_per_m ?? HAND_DEFAULTS.c_ns_per_m;
  const slack = config.slack_m ?? (config.link === "rope" ? HAND_DEFAULTS.rope_slack_m : 0);
  const maxForce = config.max_force_n ?? HAND_DEFAULTS.max_force_n;
  const maxPower = config.max_power_w ?? HAND_DEFAULTS.max_power_w;
  const reach = config.reach_m ?? HAND_DEFAULTS.reach_m;
  const slip = config.slip_after_s ?? HAND_DEFAULTS.slip_after_s;
  const close = config.close_s ?? 0;
  requirePositive("k_n_per_m", k);
  requireNonNegative("c_ns_per_m", c);
  requireNonNegative("slack_m", slack);
  requirePositive("max_force_n", maxForce);
  requirePositive("max_power_w", maxPower);
  requirePositive("reach_m", reach);
  requirePositive("slip_after_s", slip);
  requireNonNegative("close_s", close);
  if (config.link === "clamp" && slack !== 0) throw new RangeError("slack_m: clamp no tiene holgura (debe ser 0)");
  let release: number | null = null;
  if (config.release_s !== undefined) {
    requireNonNegative("release_s", config.release_s);
    if (config.release_s < close) throw new RangeError("release_s: no puede ser anterior a close_s");
    release = config.release_s;
  }
  let axis: Vec3 | null = null;
  if (config.axis !== undefined) {
    if (config.link !== "push") throw new RangeError("axis: solo se admite en push");
    requireFiniteVec("axis", config.axis);
    const length = norm(config.axis);
    if (!(length > 1e-12)) throw new RangeError("axis: no puede ser nulo");
    axis = scale(config.axis, 1 / length);
  }
  const phases = [...(config.phases ?? [])];
  let end = close;
  phases.forEach((phase, i) => {
    requireNonNegative(`phases[${i}].start_s`, phase.start_s);
    requirePositive(`phases[${i}].duration_s`, phase.duration_s);
    requireFiniteVec(`phases[${i}].offset_m`, phase.offset_m);
    if (phase.start_s < end - EPS_T_S) {
      throw new RangeError(`phases[${i}].start_s: solapa el tramo anterior o es anterior a close_s (${end})`);
    }
    end = phase.start_s + phase.duration_s;
  });
  return {
    link: config.link, shoulder_m: config.shoulder_m, close_s: close, release_s: release, phases, axis,
    k_n_per_m: k, c_ns_per_m: c, slack_m: slack, max_force_n: maxForce, max_power_w: maxPower, reach_m: reach,
    slip_after_s: slip,
  };
}

/** Estado inicial: mano abierta, sin totales. */
export function createHandState(): HandState {
  return {
    phase: "open", origin_m: null, axis: null, at_cap_s: 0, pending: null, events: [],
    totals: {
      work_j: 0, impulse_ns: ZERO, reaction_impulse_ns: ZERO, max_force_n: 0, max_power_w: 0, time_at_cap_s: 0,
    },
  };
}

/** Fuerza del muelle sobre el agarre según el tipo de unión, antes de los topes. `d` va del agarre al objetivo. */
function linkForce(params: HandParams, d: Vec3, v: Vec3, axis: Vec3 | null): Vec3 {
  const { k_n_per_m: k, c_ns_per_m: c, slack_m: slack } = params;
  if (params.link === "clamp") {
    return [k * d[0] - c * v[0], k * d[1] - c * v[1], k * d[2] - c * v[2]];
  }
  if (params.link === "rope") {
    const length = norm(d);
    if (!(length > slack) || length === 0) return ZERO;
    const unit = scale(d, 1 / length);
    const bracket = k * (length - slack) - c * dot(v, unit);
    return bracket > 0 ? scale(unit, bracket) : ZERO;
  }
  // push: el mismo muelle unilateral, medido sobre el eje de empuje en lugar de la línea mano–agarre.
  const along = dot(axis!, d);
  if (!(along > slack)) return ZERO;
  const bracket = k * (along - slack) - c * dot(v, axis!);
  return bracket > 0 ? scale(axis!, bracket) : ZERO;
}

/**
 * Un paso de la mano (función pura: no modifica `state` ni `params`).
 *
 * Orden: trabajo del paso anterior; cierre, suelta programada o resbalón; objetivo `p_t(t)` recortado al alcance;
 * fuerza del muelle; tope de fuerza `max_force_n`; tope de potencia `max_power_w` (si `F · v_g` lo supera, se reduce `F`).
 */
export function stepHand(params: HandParams, state: HandState, input: HandStepInput): HandStepResult {
  const { t, dt, grip_m: grip, grip_velocity_mps: velocity } = input;
  if (!Number.isFinite(t)) throw new RangeError("t: debe ser finito");
  requirePositive("dt", dt);
  requireFiniteVec("grip_m", grip);
  requireFiniteVec("grip_velocity_mps", velocity);

  // Trabajo del paso anterior con la velocidad media (trapecio), como el ejecutor.
  let work = state.totals.work_j;
  if (state.pending) {
    const mean = scale(add(state.pending.velocity_mps, velocity), 0.5);
    work += dot(state.pending.force_n, mean) * state.pending.dt;
  }

  const events: HandEvent[] = [];
  let phase = state.phase;
  let origin = state.origin_m;
  let axis = state.axis;
  let atCap = state.at_cap_s;

  if (phase === "open" && t >= params.close_s - EPS_T_S) {
    phase = "holding";
    origin = grip;
    if (params.link === "push") {
      axis = params.axis;
      if (axis === null) {
        const away = sub(grip, params.shoulder_m);
        const length = norm(away);
        if (!(length > 1e-9)) throw new RangeError("push sin axis: el agarre coincide con el hombro y el eje queda indefinido");
        axis = scale(away, 1 / length);
      }
    }
    events.push({ t, kind: "close" });
  }
  if (phase === "holding") {
    if (params.release_s !== null && t >= params.release_s - EPS_T_S) {
      phase = "released";
      events.push({ t, kind: "release" });
    } else if (atCap >= params.slip_after_s - EPS_T_S) {
      phase = "released";
      events.push({ t, kind: "slip" });
    }
  }

  let force: Vec3 = ZERO;
  let demand = 0;
  let forceLimited = false;
  let powerLimited = false;
  let reachLimited = false;
  let target: Vec3 | null = null;
  let targetRaw: Vec3 | null = null;

  if (phase === "holding") {
    targetRaw = add(origin!, handTargetOffset(params.phases, t));
    target = targetRaw;
    const fromShoulder = sub(targetRaw, params.shoulder_m);
    const distance = norm(fromShoulder);
    if (distance > params.reach_m) {
      target = add(params.shoulder_m, scale(fromShoulder, params.reach_m / distance));
      reachLimited = true;
    }
    force = linkForce(params, sub(target, grip), velocity, axis);
    demand = norm(force);
    // 1. Tope de fuerza.
    if (demand >= params.max_force_n) {
      forceLimited = true;
      if (demand > params.max_force_n) force = scale(force, params.max_force_n / demand);
    }
    // 2. Tope de potencia (solo la potencia positiva: la fuerza que frena no se limita).
    const power = dot(force, velocity);
    if (power > params.max_power_w) {
      force = scale(force, params.max_power_w / power);
      powerLimited = true;
    }
    atCap = forceLimited ? atCap + dt : 0;
  } else {
    atCap = 0;
  }

  const magnitude = norm(force);
  const power = dot(force, velocity);
  const reaction = scale(force, -1);
  const impulse = add(state.totals.impulse_ns, scale(force, dt));
  const totals: HandTotals = {
    work_j: work,
    impulse_ns: impulse,
    reaction_impulse_ns: scale(impulse, -1),
    max_force_n: Math.max(state.totals.max_force_n, magnitude),
    max_power_w: Math.max(state.totals.max_power_w, power),
    time_at_cap_s: state.totals.time_at_cap_s + (forceLimited ? dt : 0),
  };
  const next: HandState = {
    phase, origin_m: origin, axis, at_cap_s: atCap,
    pending: phase === "holding" ? { force_n: force, velocity_mps: velocity, dt } : null,
    totals,
    events: events.length > 0 ? [...state.events, ...events] : state.events,
  };
  const sample: HandSample = {
    t, held: phase === "holding", target_m: target, target_unclipped_m: targetRaw, force_n: force,
    force_magnitude_n: magnitude, demand_force_n: demand, power_w: power, force_limited: forceLimited,
    power_limited: powerLimited, reach_limited: reachLimited, work_j: work, reaction_n: reaction,
  };
  return { force_n: force, reaction_n: reaction, state: next, sample, events };
}

/** Cierra el trabajo del último paso con la velocidad del punto tras él. Llamar una vez, al terminar la simulación. */
export function settleHandWork(state: HandState, grip_velocity_mps: Vec3): HandState {
  requireFiniteVec("grip_velocity_mps", grip_velocity_mps);
  if (!state.pending) return state;
  const mean = scale(add(state.pending.velocity_mps, grip_velocity_mps), 0.5);
  const work = state.totals.work_j + dot(state.pending.force_n, mean) * state.pending.dt;
  return { ...state, pending: null, totals: { ...state.totals, work_j: work } };
}

/** Parte de los datos de MuJoCo que usa `applyForceAtPoint` (compatible con `MjData` sin importar el módulo). */
export interface WrenchData {
  /** Posición del centro de masas de cada cuerpo en el mundo (3 por cuerpo; válida tras `mj_kinematics` y `mj_comPos`). */
  readonly xipos: ArrayLike<number>;
  /** Fuerza y par externos por cuerpo en el mundo (6 por cuerpo: fuerza y después par respecto al centro de masas). */
  readonly xfrc_applied: Float64Array;
}

/**
 * Suma a `xfrc_applied` la fuerza `force_n` aplicada en `point_m` (ambos en el mundo) sobre el cuerpo `bodyId`:
 * la fuerza y el par `(p − c) × F` respecto al centro de masas `c`. Es la matemática del ejecutor de escenarios.
 * Acumula: quien llama pone `xfrc_applied` a cero al empezar cada paso.
 */
export function applyForceAtPoint(data: WrenchData, bodyId: number, point_m: Vec3, force_n: Vec3): void {
  if (!Number.isSafeInteger(bodyId) || bodyId <= 0 || 6 * bodyId + 5 >= data.xfrc_applied.length) {
    throw new RangeError("Cuerpo inválido");
  }
  requireFiniteVec("point_m", point_m);
  requireFiniteVec("force_n", force_n);
  const c = [data.xipos[3 * bodyId]!, data.xipos[3 * bodyId + 1]!, data.xipos[3 * bodyId + 2]!] as const;
  const r = [point_m[0] - c[0], point_m[1] - c[1], point_m[2] - c[2]] as const;
  const torque = [
    r[1] * force_n[2] - r[2] * force_n[1],
    r[2] * force_n[0] - r[0] * force_n[2],
    r[0] * force_n[1] - r[1] * force_n[0],
  ] as const;
  const o = 6 * bodyId;
  const w = data.xfrc_applied;
  for (let i = 0; i < 3; i++) {
    w[o + i] = w[o + i]! + force_n[i]!;
    w[o + 3 + i] = w[o + 3 + i]! + torque[i]!;
  }
}
