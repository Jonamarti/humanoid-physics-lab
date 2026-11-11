/**
 * Reproductor de grabaciones `hpl.recording.v1`: clase pura (sin Three.js, DOM ni MuJoCo). Interpola entre
 * muestras (lineal en posiciones y ángulos; slerp en cuaterniones) y gestiona tiempo, velocidad y pausa.
 * NUNCA calcula física: solo mezcla estados ya grabados.
 */
import type { ContactSample, EffectorSample, Recording, RecordingSample } from "./schema";

export const PLAYBACK_SPEEDS = [1, 0.5, 0.25] as const;

/** Códigos `mjtJoint` de MuJoCo. */
export const JOINT_FREE = 0;
export const JOINT_BALL = 1;

/** Índices de `qpos` donde empieza cada cuaternión `wxyz`, a partir de los tipos de articulación (`jnt_type`). */
export function quaternionStarts(jointTypes: readonly number[]): number[] {
  const starts: number[] = [];
  let adr = 0;
  for (const type of jointTypes) {
    if (type === JOINT_FREE) {
      starts.push(adr + 3);
      adr += 7;
    } else if (type === JOINT_BALL) {
      starts.push(adr);
      adr += 4;
    } else {
      adr += 1; // deslizante o bisagra
    }
  }
  return starts;
}

/** Interpolación esférica de cuaterniones `wxyz` con corrección de signo (camino corto); resultado unitario. */
export function slerp(a: readonly number[], b: readonly number[], alpha: number): [number, number, number, number] {
  let d = a[0]! * b[0]! + a[1]! * b[1]! + a[2]! * b[2]! + a[3]! * b[3]!;
  const sign = d < 0 ? -1 : 1;
  d *= sign;
  let wa: number;
  let wb: number;
  if (d > 0.9995) {
    wa = 1 - alpha; // casi iguales: lineal normalizada
    wb = alpha * sign;
  } else {
    const theta = Math.acos(Math.min(1, d));
    const sinTheta = Math.sin(theta);
    wa = Math.sin((1 - alpha) * theta) / sinTheta;
    wb = (Math.sin(alpha * theta) / sinTheta) * sign;
  }
  const q: [number, number, number, number] = [
    wa * a[0]! + wb * b[0]!, wa * a[1]! + wb * b[1]!, wa * a[2]! + wb * b[2]!, wa * a[3]! + wb * b[3]!,
  ];
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  return n > 0 ? [q[0] / n, q[1] / n, q[2] / n, q[3] / n] : [1, 0, 0, 0];
}

export interface PlayerFrame {
  t: number;
  /** Índice de la muestra anterior (o igual) a `t`. */
  index: number;
  qpos: number[];
  effectors: EffectorSample[];
  /** Contactos de la muestra anterior: no se interpolan. */
  contacts: ContactSample[];
}

const lerp = (a: number, b: number, alpha: number): number => a + (b - a) * alpha;
const lerpVec = (a: readonly number[], b: readonly number[], alpha: number): [number, number, number] =>
  [lerp(a[0]!, b[0]!, alpha), lerp(a[1]!, b[1]!, alpha), lerp(a[2]!, b[2]!, alpha)];

export class RecordingPlayer {
  private readonly samples: readonly RecordingSample[];
  private readonly quatStarts: readonly number[];
  private timeS = 0;
  private speedValue = 1;
  private playingValue = false;

  /**
   * @param jointTypes `jnt_type` del modelo (raíz libre = 0, esférica = 1...). Sin él, todo `qpos` se interpola
   * linealmente.
   */
  constructor(recording: Pick<Recording, "samples">, jointTypes: readonly number[] = []) {
    if (recording.samples.length === 0) throw new RangeError("La grabación no tiene muestras");
    this.samples = recording.samples;
    this.quatStarts = quaternionStarts(jointTypes);
    this.timeS = this.samples[0]!.t;
  }

  get sampleCount(): number { return this.samples.length; }
  get startTime(): number { return this.samples[0]!.t; }
  get endTime(): number { return this.samples[this.samples.length - 1]!.t; }
  get duration(): number { return this.endTime - this.startTime; }
  get time(): number { return this.timeS; }
  get speed(): number { return this.speedValue; }
  get playing(): boolean { return this.playingValue; }

  /** Índice `i` tal que `samples[i].t <= t` (el mayor posible); `t` ya recortado al rango. */
  private indexAt(t: number): number {
    let lo = 0;
    let hi = this.samples.length - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.samples[mid]!.t <= t) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  }

  private clamp(t: number): number {
    if (!Number.isFinite(t)) throw new RangeError("Tiempo no finito");
    return Math.min(this.endTime, Math.max(this.startTime, t));
  }

  /** Estado en el instante `t` (recortado al rango). En el instante de una muestra devuelve exactamente esa muestra. */
  sampleAt(t: number): PlayerFrame {
    const time = this.clamp(t);
    const i = this.indexAt(time);
    const a = this.samples[i]!;
    const b = this.samples[i + 1];
    const alpha = b && time > a.t ? (time - a.t) / (b.t - a.t) : 0;
    if (!b || alpha === 0) {
      return {
        t: time, index: i, qpos: a.qpos.slice(),
        effectors: a.effectors.map((e) => ({ id: e.id, point_m: [...e.point_m], force_n: [...e.force_n] })),
        contacts: a.contacts,
      };
    }
    const qpos = a.qpos.map((v, k) => lerp(v, b.qpos[k]!, alpha));
    for (const s of this.quatStarts) {
      if (s + 4 > qpos.length) continue;
      const q = slerp(a.qpos.slice(s, s + 4), b.qpos.slice(s, s + 4), alpha);
      for (let k = 0; k < 4; k++) qpos[s + k] = q[k]!;
    }
    const effectors = a.effectors.map((e, k): EffectorSample => {
      const next = b.effectors[k];
      if (!next || next.id !== e.id) return { id: e.id, point_m: [...e.point_m], force_n: [...e.force_n] };
      return { id: e.id, point_m: lerpVec(e.point_m, next.point_m, alpha), force_n: lerpVec(e.force_n, next.force_n, alpha) };
    });
    return { t: time, index: i, qpos, effectors, contacts: a.contacts };
  }

  /** Estado en el instante actual. */
  current(): PlayerFrame { return this.sampleAt(this.timeS); }

  seek(t: number): void { this.timeS = this.clamp(t); }

  setSpeed(speed: number): void {
    if (!(speed > 0) || !Number.isFinite(speed)) throw new RangeError("Velocidad de reproducción inválida");
    this.speedValue = speed;
  }

  play(): void {
    if (this.timeS >= this.endTime) this.timeS = this.startTime; // al final, reproducir reinicia
    this.playingValue = true;
  }

  pause(): void { this.playingValue = false; }

  toggle(): void { if (this.playingValue) this.pause(); else this.play(); }

  /** Avanza `wallDtS` segundos de reloj real (por la velocidad). Al llegar al final se detiene. Devuelve el tiempo. */
  advance(wallDtS: number): number {
    if (!this.playingValue) return this.timeS;
    this.timeS = this.clamp(this.timeS + Math.max(0, wallDtS) * this.speedValue);
    if (this.timeS >= this.endTime) this.playingValue = false;
    return this.timeS;
  }

  /** Pausa y salta a la muestra siguiente (por muestras, no por tiempo). Devuelve su índice. */
  stepNext(): number {
    this.pause();
    const i = this.indexAt(this.timeS);
    const next = Math.min(this.samples.length - 1, i + 1);
    this.timeS = this.samples[next]!.t;
    return next;
  }

  /** Pausa y salta a la muestra anterior. Entre dos muestras, a la muestra inmediatamente anterior a `t`. */
  stepPrev(): number {
    this.pause();
    const i = this.indexAt(this.timeS);
    const prev = this.samples[i]!.t < this.timeS ? i : Math.max(0, i - 1);
    this.timeS = this.samples[prev]!.t;
    return prev;
  }
}
