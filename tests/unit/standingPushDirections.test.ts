import { describe, expect, it } from "vitest";
import {
  PUSH_DIRECTIONS, PUSH_DIRECTION_VECTORS, expectedPushImpulseNs,
  pushForceAtTime, type PushTrial,
} from "../../src/physics/standingPush";

describe("Eight-direction world-space push protocol", () => {
  const template = { peakForceN: 120, durationS: 0.12, startS: 1 };
  it("defines exactly eight unique unit vectors in XY", () => {
    expect(PUSH_DIRECTIONS).toHaveLength(8);
    const keys = PUSH_DIRECTIONS.map(direction => PUSH_DIRECTION_VECTORS[direction]);
    expect(new Set(keys.map(x => x.join(","))).size).toBe(8);
    for (const [x, y, z] of keys) {
      expect(Math.hypot(x,y,z)).toBeCloseTo(1,12);
      expect(z).toBe(0);
    }
  });
  it("maintains the exact same force peak and analytic impulse across angles", () => {
    for (const direction of PUSH_DIRECTIONS) {
      const trial: PushTrial = { ...template, direction };
      const force = pushForceAtTime(1.06, trial);
      expect(Math.hypot(...force)).toBeCloseTo(120,10);
      expect(expectedPushImpulseNs(trial)).toBeCloseTo(2 * 120 * 0.12 / Math.PI,12);
      expect(pushForceAtTime(1,trial)).toEqual([0,0,0]);
      expect(pushForceAtTime(1.12,trial)).toEqual([0,0,0]);
    }
  });
  it("confirms anterior/posterior and left/right mirror symmetry", () => {
    const at = (direction: PushTrial["direction"]) =>
      pushForceAtTime(1.03, { ...template, direction });
    for (const [a,b] of [
      ["forward-left","backward-right"],
      ["forward-right","backward-left"],
      ["forward","backward"],
      ["left","right"],
    ] as const) {
      const p = at(a), q = at(b);
      expect(p[0]).toBeCloseTo(-q[0],10);
      expect(p[1]).toBeCloseTo(-q[1],10);
    }
  });
  it("refuses an unsupported direction rather than producing undefined force", () => {
    expect(() => pushForceAtTime(1.06,{
      ...template, direction:"vertical" as PushTrial["direction"],
    })).toThrow(RangeError);
  });
});
