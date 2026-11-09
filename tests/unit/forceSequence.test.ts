import { describe, expect, it } from "vitest";
import {
  editedForceAtTime, editedForceImpulseNs, editedSequenceHorizonS,
  validateForceSequence, type EditedForce,
} from "../../src/physics/forceSequence";
import { classifyForceGlyph } from "../../src/renderer/forceGlyph";

const push: EditedForce = {
  id: "f1", site: "sternum_impact", forceN: [0, -240, 0],
  startS: 1, durationS: 0.12, waveform: "half-sine",
};

describe("editor de fuerzas externas", () => {
  it("pulso con exactamente cero fuera del intervalo, pico y J vectoriales", () => {
    expect(editedForceAtTime(0.99, push)).toEqual([0, 0, 0]);
    expect(editedForceAtTime(1, push)).toEqual([0, 0, 0]);
    expect(editedForceAtTime(1.06, push)[1]).toBeCloseTo(-240, 9);
    expect(editedForceAtTime(1.12, push)).toEqual([0, 0, 0]);
    expect(editedForceImpulseNs(push)[1]).toBeCloseTo(-2 * 240 * 0.12 / Math.PI, 8);
  });

  it("fuerza sostenida y vector 3D incluidas componentes fuera del plano", () => {
    const steady: EditedForce = {
      ...push, forceN: [100, 0, -50], waveform: "constant", durationS: 2,
    };
    expect(editedForceAtTime(1, steady)).toEqual([100, 0, -50]);
    expect(editedForceAtTime(2, steady)).toEqual([100, 0, -50]);
    expect(editedForceAtTime(3, steady)).toEqual([0, 0, 0]);
    expect(editedForceImpulseNs(steady)).toEqual([200, 0, -100]);
  });

  it("valida id únicos y rechaza fuerzas peligrosamente grandes o corruptas", () => {
    expect(() => validateForceSequence([push, {...push}])).toThrow();
    expect(() => validateForceSequence([{ ...push, forceN: [0, 0, 10_001] }])).toThrow();
    expect(() => validateForceSequence([{ ...push, startS: Number.NaN }])).toThrow();
    expect(() => validateForceSequence([push])).not.toThrow();
    expect(editedSequenceHorizonS([{ ...push, startS: 5, durationS: 3 }])).toBe(10);
  });

  it("⊗ y ⊙ se interpretan respecto a la cámara, no por un eje fijo del mundo", () => {
    expect(classifyForceGlyph([0, 1, 0], [0, 1, 0])).toBe("out-of-screen");
    expect(classifyForceGlyph([0, -1, 0], [0, 1, 0])).toBe("into-screen");
    expect(classifyForceGlyph([1, 0, 0], [0, 1, 0])).toBe("arrow");
    expect(classifyForceGlyph([0, 0, 0], [0, 1, 0])).toBe("none");
    expect(classifyForceGlyph([1, 0, 0], [1, 0, 0])).toBe("out-of-screen");
  });
});
