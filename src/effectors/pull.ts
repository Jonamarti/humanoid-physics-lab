/**
 * Tirón guiado: una mano de Tori que aplica una FUERZA cuya dirección y módulo siguen una curva en el tiempo, con tope de
 * fuerza y de potencia. Complementa a la mano virtual (`hand.ts`): allí se guía la posición de un objetivo y la dirección
 * de la fuerza sale de dónde esté el agarre; aquí se guía la fuerza misma, que es lo que hace falta para describir un tirón
 * en arco (delante y arriba, delante, delante y abajo, abajo). Módulo puro, sin dependencias.
 *
 * Convenciones: SI; `t` es el reloj del escenario; todos los vectores en el mismo marco, que elige quien llama.
 */
export type Vec3 = readonly [number, number, number];

/** Valores por defecto: los mismos topes que la mano virtual (issue #76). Puntos de partida sin calibrar. */
export const PULL_DEFAULTS = Object.freeze({ max_force_n: 500, max_power_w: 600 });

/** Punto de la curva: en `t_s` la fuerza pedida es `force_n`. Entre puntos se interpola linealmente el vector. */
export interface PullKey {
  readonly t_s: number;
  readonly force_n: Vec3;
}

export interface PullConfig {
  keys: readonly PullKey[];
  max_force_n?: number;
  max_power_w?: number;
}

export interface PullParams {
  readonly keys: readonly PullKey[];
  readonly max_force_n: number;
  readonly max_power_w: number;
}

function finiteVec(v: Vec3): boolean {
  return Array.isArray(v) && v.length === 3 && Number.isFinite(v[0]) && Number.isFinite(v[1]) && Number.isFinite(v[2]);
}

/** Valida y aplica los valores por defecto. Lanza `RangeError` con el campo y el motivo. */
export function resolvePullParams(config: PullConfig): PullParams {
  const maxForce = config.max_force_n ?? PULL_DEFAULTS.max_force_n;
  const maxPower = config.max_power_w ?? PULL_DEFAULTS.max_power_w;
  if (!(Number.isFinite(maxForce) && maxForce > 0)) throw new RangeError(`max_force_n: debe ser un número finito > 0 (recibido ${maxForce})`);
  if (!(Number.isFinite(maxPower) && maxPower > 0)) throw new RangeError(`max_power_w: debe ser un número finito > 0 (recibido ${maxPower})`);
  if (!Array.isArray(config.keys) || config.keys.length < 2) throw new RangeError("keys: hacen falta al menos dos puntos");
  let last = -Infinity;
  config.keys.forEach((key, i) => {
    if (!(Number.isFinite(key.t_s) && key.t_s >= 0)) throw new RangeError(`keys[${i}].t_s: debe ser un número finito >= 0`);
    if (!(key.t_s > last)) throw new RangeError(`keys[${i}].t_s: los instantes deben ser estrictamente crecientes`);
    last = key.t_s;
    if (!finiteVec(key.force_n)) throw new RangeError(`keys[${i}].force_n: debe ser [x, y, z] con números finitos`);
    const magnitude = Math.hypot(key.force_n[0], key.force_n[1], key.force_n[2]);
    if (magnitude > maxForce * (1 + 1e-9)) {
      throw new RangeError(`keys[${i}].force_n: su módulo (${magnitude.toFixed(1)} N) supera max_force_n (${maxForce} N)`);
    }
  });
  return { keys: config.keys, max_force_n: maxForce, max_power_w: maxPower };
}

/** Fuerza pedida en el instante `t`: nula antes del primer punto y desde el último; interpolación lineal entre puntos. */
export function pullDemand(keys: readonly PullKey[], t: number): Vec3 {
  const first = keys[0]!;
  const final = keys[keys.length - 1]!;
  if (t < first.t_s || t >= final.t_s) return [0, 0, 0];
  for (let i = 1; i < keys.length; i++) {
    const b = keys[i]!;
    if (t < b.t_s) {
      const a = keys[i - 1]!;
      const u = (t - a.t_s) / (b.t_s - a.t_s);
      return [
        a.force_n[0] + (b.force_n[0] - a.force_n[0]) * u,
        a.force_n[1] + (b.force_n[1] - a.force_n[1]) * u,
        a.force_n[2] + (b.force_n[2] - a.force_n[2]) * u,
      ];
    }
  }
  return [0, 0, 0];
}

/**
 * Aplica el tope de potencia a una fuerza ya expresada en el marco de `velocity_mps` (el mundo): si `F · v` supera
 * `max_power_w`, la fuerza se reduce hasta cumplirlo. La fuerza que frena (potencia negativa) no se limita. El tope de
 * fuerza no hace falta aquí: `resolvePullParams` rechaza puntos por encima de él y la interpolación lineal no lo supera.
 */
export function limitPullPower(params: PullParams, force_n: Vec3, velocity_mps: Vec3): { force_n: Vec3; power_w: number; power_limited: boolean } {
  const power = force_n[0] * velocity_mps[0] + force_n[1] * velocity_mps[1] + force_n[2] * velocity_mps[2];
  if (!(power > params.max_power_w)) return { force_n, power_w: power, power_limited: false };
  const s = params.max_power_w / power;
  return { force_n: [force_n[0] * s, force_n[1] * s, force_n[2] * s], power_w: params.max_power_w, power_limited: true };
}
