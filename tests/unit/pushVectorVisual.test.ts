import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { forceArrowLengthM } from "../../src/renderer/forceArrow";
import { mujocoToThreePosition } from "../../src/renderer/coordinates";
import { MujocoSimulation } from "../../src/physics/mujoco";
import {
  DEFAULT_PUSH_TRIAL, expectedPushImpulseNs, pushForceAtTime, validatePushTrial,
} from "../../src/physics/standingPush";

describe("Editable strong pushes and physically anchored force vectors", () => {
  it("accepts magnitudes beyond the former 240/2000 N presets", () => {
    for (const peakForceN of [0, 500, 2000, 4000, 10000]) {
      const trial = { ...DEFAULT_PUSH_TRIAL, peakForceN };
      expect(() => validatePushTrial(trial)).not.toThrow();
      const applied = pushForceAtTime(trial.startS + trial.durationS/2, trial);
      expect(Math.hypot(...applied)).toBeCloseTo(peakForceN, 9);
      expect(expectedPushImpulseNs(trial)).toBeCloseTo(
        2 * peakForceN * trial.durationS / Math.PI, 8);
    }
    for (const peakForceN of [-1, 10001, NaN, Infinity]) {
      expect(() => validatePushTrial({ ...DEFAULT_PUSH_TRIAL, peakForceN })).toThrow(RangeError);
    }
  });

  it("keeps visual length bounded and force direction transformed consistently", () => {
    expect(forceArrowLengthM(0)).toBe(0);
    expect(forceArrowLengthM(10000)).toBeLessThanOrEqual(1.35);
    expect(forceArrowLengthM(4000)).toBeGreaterThan(forceArrowLengthM(240));
    expect(mujocoToThreePosition([1, 0, 2])).toEqual([1, 2, -0]);
    expect(mujocoToThreePosition([0, 1, 0])).toEqual([0, 0, -1]);
    expect(() => forceArrowLengthM(NaN)).toThrow(RangeError);
    expect(() => forceArrowLengthM(-1)).toThrow(RangeError);
  });

  it("anchors visual vector on a real body-attached site that follows the falling chest", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const sim = await MujocoSimulation.create(xml);
    try {
      const before = sim.siteWorldPosition("sternum_impact");
      expect(before).toHaveLength(3);
      expect(before.every(Number.isFinite)).toBe(true);
      expect(() => sim.siteWorldPosition("nonexistent")).toThrow();
      sim.step(200);
      const after = sim.siteWorldPosition("sternum_impact");
      expect(Math.hypot(after[0]-before[0], after[1]-before[1], after[2]-before[2]))
        .toBeGreaterThan(0.001);
      expect(after.every(Number.isFinite)).toBe(true);
    } finally {
      sim.dispose();
    }
  }, 30000);
});
