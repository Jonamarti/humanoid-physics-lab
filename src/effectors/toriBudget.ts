/**
 * Presupuesto de Tori (issue #76, PR 4, primera parte). Tori no tiene cuerpo: es una masa con dos pies. En cada paso se
 * suma lo que Uke le devuelve, `R = −(Σ fuerzas de las manos sobre Uke + Σ contacto de los obstáculos sobre Uke)`, y se
 * comprueba que un Tori de esa masa podría sostenerlo. Es un indicador de plausibilidad, NO una simulación de Tori.
 *
 * Topes:
 *  - `friction`: `|R_horizontal| ≤ µ · N`, con `N = m_T·g − R_z` la carga de sus pies. Si `N ≤ 0`, Tori despega.
 *  - `total_pull`: `|Σ F_manos| ≤ total_pull_max_n`.
 *  - `total_power`: `Σ F_mano · v_agarre ≤ total_power_max_w`.
 *  - `horizontal_impulse`: `|∫ R_horizontal dt| ≤ m_T · 3 m/s`. Solo la parte horizontal: sostener el peso de Uke un rato
 *    acumula impulso vertical sin que Tori cambie de velocidad, porque lo descarga en el suelo.
 *
 * `R` se promedia en una ventana de `SMOOTH_S` antes de comprobar el rozamiento: un choque de un paso físico contra la
 * cadera no hace resbalar a un cuerpo que no es rígido. Módulo puro.
 */
export type Vec3 = readonly [number, number, number];

export const TORI_BUDGET_DEFAULTS = Object.freeze({
  total_pull_max_n: 1100,
  total_power_max_w: 1500,
  /** Cambio de velocidad horizontal máximo admitido para Tori, en m/s. */
  max_speed_change_mps: 3,
});
export const GRAVITY_MPS2 = 9.81;
/** Ventana del promedio de `R` para el rozamiento, en s. */
export const SMOOTH_S = 0.04;

export interface ToriBudgetParams {
  mass_kg: number;
  friction: number;
  total_pull_max_n?: number;
  total_power_max_w?: number;
}

export type ToriViolationKind = "friction" | "feet_unloaded" | "total_pull" | "total_power" | "horizontal_impulse";

export interface ToriViolation {
  kind: ToriViolationKind;
  /** Primer instante en que se incumple, en s. */
  t_s: number;
  /** Peor valor medido y su tope (N, W o N·s; en `friction`, fuerza horizontal y `µ·N` en ese instante). */
  value: number;
  limit: number;
  /** Tiempo total incumpliendo, en s. */
  duration_s: number;
}

export interface PeakAt {
  value: number;
  t_s: number;
}

export interface ToriBudgetSummary {
  feasible: boolean;
  mass_kg: number;
  friction: number;
  /** Máximo de `|R_h| / (µ·N)`: por encima de 1, Tori resbalaría. */
  friction_use: PeakAt;
  /** Máximo de `|R_h|` (N) y de la carga de los pies `N` (N); mínimo de `N`. */
  max_horizontal_reaction_n: PeakAt;
  max_feet_load_n: PeakAt;
  min_feet_load_n: PeakAt;
  max_total_pull_n: PeakAt;
  max_total_power_w: PeakAt;
  /** Máximo de `|∫ R_h dt|` (N·s) y su tope `m_T · 3 m/s`. */
  max_horizontal_impulse_ns: PeakAt;
  horizontal_impulse_limit_ns: number;
  violations: ToriViolation[];
}

export interface ToriStepInput {
  t: number;
  dt: number;
  /** Suma de las fuerzas de las manos sobre Uke en el paso, en el mundo (N). */
  hands_force_n: Vec3;
  /** Suma de `F · v` de las manos (W). */
  hands_power_w: number;
  /** Suma de las fuerzas de contacto de los obstáculos sobre Uke (N). */
  obstacle_force_on_uke_n: Vec3;
}

export class ToriBudget {
  private readonly p: Required<ToriBudgetParams>;
  private readonly window: Vec3[] = [];
  private sum: [number, number, number] = [0, 0, 0];
  private impulse: [number, number] = [0, 0];
  private readonly violations = new Map<ToriViolationKind, ToriViolation>();
  private frictionUse: PeakAt = { value: 0, t_s: 0 };
  private maxRh: PeakAt = { value: 0, t_s: 0 };
  private maxN: PeakAt = { value: 0, t_s: 0 };
  private minN: PeakAt = { value: Infinity, t_s: 0 };
  private maxPull: PeakAt = { value: 0, t_s: 0 };
  private maxPower: PeakAt = { value: 0, t_s: 0 };
  private maxImpulse: PeakAt = { value: 0, t_s: 0 };

  constructor(params: ToriBudgetParams) {
    for (const key of ["mass_kg", "friction"] as const) {
      if (!(Number.isFinite(params[key]) && params[key] > 0)) throw new RangeError(`${key}: debe ser un número finito > 0`);
    }
    this.p = {
      mass_kg: params.mass_kg,
      friction: params.friction,
      total_pull_max_n: params.total_pull_max_n ?? TORI_BUDGET_DEFAULTS.total_pull_max_n,
      total_power_max_w: params.total_power_max_w ?? TORI_BUDGET_DEFAULTS.total_power_max_w,
    };
  }

  private violate(kind: ToriViolationKind, t: number, dt: number, value: number, limit: number, worse: boolean): void {
    const v = this.violations.get(kind);
    if (!v) this.violations.set(kind, { kind, t_s: t, value, limit, duration_s: dt });
    else {
      v.duration_s += dt;
      if (worse) { v.value = value; v.limit = limit; }
    }
  }

  step(input: ToriStepInput): void {
    const { t, dt, hands_force_n: h, obstacle_force_on_uke_n: o } = input;
    const r: Vec3 = [-(h[0] + o[0]), -(h[1] + o[1]), -(h[2] + o[2])];
    const size = Math.max(1, Math.round(SMOOTH_S / dt));
    this.window.push(r);
    for (let i = 0; i < 3; i++) this.sum[i] = this.sum[i]! + r[i]!;
    if (this.window.length > size) {
      const old = this.window.shift()!;
      for (let i = 0; i < 3; i++) this.sum[i] = this.sum[i]! - old[i]!;
    }
    const n = this.window.length;
    const rh = Math.hypot(this.sum[0] / n, this.sum[1] / n);
    const load = this.p.mass_kg * GRAVITY_MPS2 - this.sum[2] / n;
    if (rh > this.maxRh.value) this.maxRh = { value: rh, t_s: t };
    if (load > this.maxN.value) this.maxN = { value: load, t_s: t };
    if (load < this.minN.value) this.minN = { value: load, t_s: t };
    if (load <= 0) {
      this.violate("feet_unloaded", t, dt, load, 0, load < (this.violations.get("feet_unloaded")?.value ?? 0));
    } else {
      const use = rh / (this.p.friction * load);
      const worse = use > this.frictionUse.value;
      if (worse) this.frictionUse = { value: use, t_s: t };
      if (use > 1) this.violate("friction", t, dt, rh, this.p.friction * load, worse);
    }

    const pull = Math.hypot(h[0], h[1], h[2]);
    const pullWorse = pull > this.maxPull.value;
    if (pullWorse) this.maxPull = { value: pull, t_s: t };
    if (pull > this.p.total_pull_max_n * (1 + 1e-9)) this.violate("total_pull", t, dt, pull, this.p.total_pull_max_n, pullWorse);

    const powerWorse = input.hands_power_w > this.maxPower.value;
    if (powerWorse) this.maxPower = { value: input.hands_power_w, t_s: t };
    if (input.hands_power_w > this.p.total_power_max_w * (1 + 1e-9)) {
      this.violate("total_power", t, dt, input.hands_power_w, this.p.total_power_max_w, powerWorse);
    }

    this.impulse = [this.impulse[0] + r[0] * dt, this.impulse[1] + r[1] * dt];
    const j = Math.hypot(this.impulse[0], this.impulse[1]);
    const jWorse = j > this.maxImpulse.value;
    if (jWorse) this.maxImpulse = { value: j, t_s: t };
    const jLimit = this.p.mass_kg * TORI_BUDGET_DEFAULTS.max_speed_change_mps;
    if (j > jLimit) this.violate("horizontal_impulse", t, dt, j, jLimit, jWorse);
  }

  summary(): ToriBudgetSummary {
    const violations = [...this.violations.values()];
    return {
      feasible: violations.length === 0,
      mass_kg: this.p.mass_kg,
      friction: this.p.friction,
      friction_use: this.frictionUse,
      max_horizontal_reaction_n: this.maxRh,
      max_feet_load_n: this.maxN,
      min_feet_load_n: Number.isFinite(this.minN.value) ? this.minN : { value: this.p.mass_kg * GRAVITY_MPS2, t_s: 0 },
      max_total_pull_n: this.maxPull,
      max_total_power_w: this.maxPower,
      max_horizontal_impulse_ns: this.maxImpulse,
      horizontal_impulse_limit_ns: this.p.mass_kg * TORI_BUDGET_DEFAULTS.max_speed_change_mps,
      violations,
    };
  }
}
