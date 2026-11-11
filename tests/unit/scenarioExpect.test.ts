import { describe, expect, it } from "vitest";
import { angularDifferenceDeg, classifyTilt, fallDirectionDeg, tiltDeg } from "../../src/metrics/outcome";
import { compareExpect } from "../../src/scenario/expect";
import { roundSignificant } from "../../src/recording/write";
import { waveformGain } from "../../src/scenario/run";

/** xmat de una rotación de `deg` grados alrededor del eje horizontal con dirección `azimuth` (0 = +X). */
function tiltedMatrix(deg: number, azimuthDeg: number): number[] {
  const t = deg * Math.PI / 180;
  const a = azimuthDeg * Math.PI / 180;
  // El eje local Z queda en (sin t cos a, sin t sin a, cos t); el resto del marco no interviene.
  return [0, 0, Math.sin(t) * Math.cos(a), 0, 0, Math.sin(t) * Math.sin(a), 0, 0, Math.cos(t)];
}

describe("veredicto del cilindro", () => {
  it("clasifica por inclinación: <15 standing, >60 fall, entre medias tilted", () => {
    expect(classifyTilt(0)).toBe("standing");
    expect(classifyTilt(14.9)).toBe("standing");
    expect(classifyTilt(15)).toBe("tilted");
    expect(classifyTilt(60)).toBe("tilted");
    expect(classifyTilt(60.1)).toBe("fall");
    expect(classifyTilt(90)).toBe("fall");
  });

  it("mide la inclinación y la dirección de caída (0 = +X, 90 = +Y)", () => {
    expect(tiltDeg(tiltedMatrix(30, 0))).toBeCloseTo(30, 9);
    expect(tiltDeg(tiltedMatrix(120, 90))).toBeCloseTo(120, 9);
    expect(fallDirectionDeg(tiltedMatrix(45, 0))).toBeCloseTo(0, 9);
    expect(fallDirectionDeg(tiltedMatrix(45, 90))).toBeCloseTo(90, 9);
    expect(fallDirectionDeg(tiltedMatrix(45, -135))).toBeCloseTo(-135, 9);
    expect(fallDirectionDeg(tiltedMatrix(2, 40))).toBeNull();
  });

  it("la diferencia angular da la vuelta por ±180", () => {
    expect(angularDifferenceDeg(170, -170)).toBeCloseTo(20, 12);
    expect(angularDifferenceDeg(10, 350)).toBeCloseTo(20, 12);
    expect(angularDifferenceDeg(0, 180)).toBe(180);
  });
});

describe("compareExpect", () => {
  it("pass si coincide el resultado y la dirección está en tolerancia", () => {
    expect(compareExpect({ outcome: "fall", direction_deg: 20 }, { outcome: "fall", direction_deg: 0, tolerance_deg: 30 })).toBe("pass");
    expect(compareExpect({ outcome: "standing", direction_deg: null }, { outcome: "standing" })).toBe("pass");
  });

  it("devuelve las diferencias en español", () => {
    const wrongOutcome = compareExpect({ outcome: "standing", direction_deg: null }, { outcome: "fall" });
    expect(wrongOutcome).toEqual(['resultado esperado "fall", obtenido "standing"']);
    const wrongDirection = compareExpect({ outcome: "fall", direction_deg: 100 }, { outcome: "fall", direction_deg: 0, tolerance_deg: 30 });
    expect(wrongDirection).toEqual(["dirección esperada 0 ± 30 grados, obtenida 100.0"]);
    const noDirection = compareExpect({ outcome: "tilted", direction_deg: null }, { outcome: "fall", direction_deg: 0, tolerance_deg: 30 });
    expect(noDirection).toHaveLength(2);
  });
});

describe("utilidades", () => {
  it("roundSignificant redondea a 6 cifras y elimina -0", () => {
    expect(roundSignificant(0.123456789)).toBe(0.123457);
    expect(roundSignificant(123456.789)).toBe(123457);
    expect(Object.is(roundSignificant(-0), 0)).toBe(true);
    expect(() => roundSignificant(NaN)).toThrow(RangeError);
  });

  it("waveformGain: medio seno y constante, cero fuera de la ventana", () => {
    const half = { start_s: 0.3, duration_s: 0.2, waveform: "half_sine" as const };
    expect(waveformGain(half, 0.29)).toBe(0);
    expect(waveformGain(half, 0.4)).toBeCloseTo(1, 12);
    expect(waveformGain(half, 0.5)).toBe(0);
    const constant = { ...half, waveform: "constant" as const };
    expect(waveformGain(constant, 0.3)).toBe(1);
    expect(waveformGain(constant, 0.4999)).toBe(1);
    expect(waveformGain(constant, 0.5)).toBe(0);
  });
});
