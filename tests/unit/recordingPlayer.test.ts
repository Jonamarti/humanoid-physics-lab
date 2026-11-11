import { describe, expect, it } from "vitest";
import { RecordingPlayer, quaternionStarts, slerp } from "../../src/recording/player";
import type { RecordingSample } from "../../src/recording/schema";

/** Raíz libre (x, y, z, qw, qx, qy, qz) + una bisagra. */
const JOINT_TYPES = [0, 3];

function quatZ(angleRad: number): number[] {
  return [Math.cos(angleRad / 2), 0, 0, Math.sin(angleRad / 2)];
}

function sample(t: number, x: number, yaw: number, hinge: number, fx = 0): RecordingSample {
  return {
    t,
    qpos: [x, 0, 1, ...quatZ(yaw), hinge],
    qvel: [],
    effectors: [{ id: "pull", point_m: [x, 0, 1], force_n: [fx, 0, 0] }],
    contacts: [{ body_a: "world", body_b: "b", point_m: [x, 0, 0], force_n: [0, 0, 100 * t] }],
  };
}

const samples = [sample(0, 0, 0, 0, 0), sample(0.1, 1, Math.PI / 2, 2, 100), sample(0.2, 3, Math.PI, 4, 300)];

function make(): RecordingPlayer {
  return new RecordingPlayer({ samples }, JOINT_TYPES);
}

describe("RecordingPlayer", () => {
  it("expone la duración y el número de muestras", () => {
    const p = make();
    expect(p.duration).toBeCloseTo(0.2, 12);
    expect(p.sampleCount).toBe(3);
  });

  it("en el instante exacto de una muestra devuelve esa muestra", () => {
    const p = make();
    for (const [i, s] of samples.entries()) {
      const f = p.sampleAt(s.t);
      expect(f.index).toBe(i);
      expect(f.qpos).toEqual(s.qpos);
      expect(f.effectors).toEqual(s.effectors);
      expect(f.contacts).toEqual(s.contacts);
    }
  });

  it("entre dos muestras interpola linealmente posiciones y ángulos articulares", () => {
    const f = make().sampleAt(0.05);
    expect(f.qpos[0]).toBeCloseTo(0.5, 12);
    expect(f.qpos[7]).toBeCloseTo(1, 12);
    expect(f.effectors[0]!.force_n[0]).toBeCloseTo(50, 9);
    expect(f.effectors[0]!.point_m[0]).toBeCloseTo(0.5, 12);
  });

  it("interpola el cuaternión de la raíz con slerp: unitario y a mitad de camino angular", () => {
    const f = make().sampleAt(0.05);
    const q = f.qpos.slice(3, 7);
    expect(Math.hypot(...q)).toBeCloseTo(1, 12);
    const expected = quatZ(Math.PI / 4); // mitad de 0..90 grados
    q.forEach((v, k) => expect(v).toBeCloseTo(expected[k]!, 12));
    // Ángulo respecto al inicio = mitad del ángulo total: 2·acos(|dot|)
    const dot = q[0]!;
    expect(2 * Math.acos(dot)).toBeCloseTo(Math.PI / 4, 12);
  });

  it("slerp corrige el signo (camino corto) y el resultado sigue siendo unitario", () => {
    const a = quatZ(0.2);
    const b = quatZ(0.6).map((v) => -v); // mismo giro con signo opuesto
    const q = slerp(a, b, 0.5);
    expect(Math.hypot(...q)).toBeCloseTo(1, 12);
    const expected = quatZ(0.4);
    // q y -q son el mismo giro; debe estar cerca de +expected y no pasar por la esfera larga.
    expect(Math.abs(q[0] * expected[0]! + q[3] * expected[3]!)).toBeCloseTo(1, 12);
    expect(q[0]).toBeGreaterThan(0);
  });

  it("sin tipos de articulación interpola todo linealmente (sin slerp)", () => {
    const p = new RecordingPlayer({ samples });
    const f = p.sampleAt(0.05);
    expect(f.qpos[3]).toBeCloseTo((1 + Math.cos(Math.PI / 4)) / 2, 12);
  });

  it("quaternionStarts usa jnt_type: libre = +3, esférica = +0", () => {
    expect(quaternionStarts([0, 3, 1, 2])).toEqual([3, 8]);
    expect(quaternionStarts([])).toEqual([]);
  });

  it("seek fuera de rango se recorta; un tiempo no finito lanza error", () => {
    const p = make();
    p.seek(-5);
    expect(p.time).toBe(0);
    p.seek(99);
    expect(p.time).toBeCloseTo(0.2, 12);
    expect(p.sampleAt(99).qpos).toEqual(samples[2]!.qpos);
    expect(() => p.seek(NaN)).toThrow(RangeError);
  });

  it("paso siguiente y anterior van por muestras y pausan", () => {
    const p = make();
    p.play();
    expect(p.stepNext()).toBe(1);
    expect(p.playing).toBe(false);
    expect(p.time).toBeCloseTo(0.1, 12);
    expect(p.stepNext()).toBe(2);
    expect(p.stepNext()).toBe(2); // tope
    expect(p.stepPrev()).toBe(1);
    expect(p.stepPrev()).toBe(0);
    expect(p.stepPrev()).toBe(0); // tope
    p.seek(0.15); // entre muestras: «anterior» baja a la muestra de abajo, «siguiente» sube a la de arriba
    expect(p.stepPrev()).toBe(1);
    p.seek(0.15);
    expect(p.stepNext()).toBe(2);
  });

  it("la velocidad escala el avance del tiempo y la pausa lo congela", () => {
    const p = make();
    p.play();
    p.advance(0.04);
    expect(p.time).toBeCloseTo(0.04, 12);
    p.setSpeed(0.5);
    p.advance(0.04);
    expect(p.time).toBeCloseTo(0.06, 12);
    p.setSpeed(0.25);
    p.advance(0.04);
    expect(p.time).toBeCloseTo(0.07, 12);
    p.pause();
    p.advance(1);
    expect(p.time).toBeCloseTo(0.07, 12);
    expect(() => p.setSpeed(0)).toThrow(RangeError);
  });

  it("al llegar al final se detiene y al reproducir de nuevo reinicia", () => {
    const p = make();
    p.play();
    p.advance(10);
    expect(p.time).toBeCloseTo(0.2, 12);
    expect(p.playing).toBe(false);
    p.play();
    expect(p.time).toBe(0);
    expect(p.playing).toBe(true);
  });

  it("rechaza una grabación sin muestras", () => {
    expect(() => new RecordingPlayer({ samples: [] })).toThrow(RangeError);
  });
});
