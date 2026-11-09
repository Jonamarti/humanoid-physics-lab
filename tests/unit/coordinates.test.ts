import { describe, expect, it } from "vitest";
import { mujocoToThreePosition } from "../../src/renderer/coordinates";

describe("MuJoCo Z-up to Three Y-up", () => {
  it("maps gravity axis to vertical scene axis", () => {
    expect(mujocoToThreePosition([0, 0, 1.5])).toEqual([0, 1.5, -0]);
  });
  it("preserves right-handed spatial axes", () => {
    expect(mujocoToThreePosition([2, 3, 4])).toEqual([2, 4, -3]);
  });
  it("rejects invalid positions", () => {
    expect(() => mujocoToThreePosition([1, Number.NaN, 0])).toThrow(RangeError);
  });
});
