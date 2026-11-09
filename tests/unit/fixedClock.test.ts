import { describe, expect, it } from "vitest";
import { FixedClock } from "../../src/physics/fixedClock";

describe("fixed step clock", () => {
  it("accumulates independent of render frames", () => {
    const c = new FixedClock(0.002);
    expect(c.consume(0.001)).toBe(0);
    expect(c.consume(0.003)).toBe(2);
    expect(c.consume(0.016)).toBe(8);
  });
  it("caps catch-up and never accepts negative time", () => {
    const c = new FixedClock(0.002, 10);
    expect(c.consume(60)).toBe(10);
    expect(c.consume(0)).toBe(0);
    expect(() => c.consume(-1)).toThrow(RangeError);
  });
  it("clears pending fractional steps on reset", () => {
    const c = new FixedClock(0.002);
    c.consume(0.001);
    c.reset();
    expect(c.consume(0.001)).toBe(0);
  });
});
