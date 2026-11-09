import { describe, expect, it } from "vitest";
import { clamp, impulseFromConstantForce } from "../../src/physics/impulse";

describe("SI impulse utility", () => {
  it("computes constant force times duration in N*s", () => {
    expect(impulseFromConstantForce(300, 0.1)).toBeCloseTo(30);
  });
  it("preserves direction sign for signed components", () => {
    expect(impulseFromConstantForce(-200, 0.2)).toBeCloseTo(-40);
  });
  it("rejects negative duration and invalid inputs", () => {
    expect(() => impulseFromConstantForce(10, -0.1)).toThrow(RangeError);
    expect(() => impulseFromConstantForce(Number.NaN, 0.1)).toThrow(RangeError);
  });
});

describe("limited actuator utility", () => {
  it("saturates within bounds", () => {
    expect(clamp(200, -100, 100)).toBe(100);
    expect(clamp(-200, -100, 100)).toBe(-100);
    expect(clamp(20, -100, 100)).toBe(20);
  });
  it("rejects inverted bounds", () => {
    expect(() => clamp(1, 5, -5)).toThrow(RangeError);
  });
});
