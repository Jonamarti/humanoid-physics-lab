import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Humanoid dynamic sensors with the actual MuJoCo WASM runtime", () => {
  it("reports finite COM, speed, foot placement and support candidate", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const sim = await MujocoSimulation.create(xml);
    try {
      const a = sim.humanoidBalanceObservation();
      expect(a.totalMassKg).toBeCloseTo(75, 6);
      expect(a.comWorldM[2]).toBeGreaterThan(0.7);
      expect(a.comWorldM[2]).toBeLessThan(1.4);
      expect(a.candidateSupportPolygonXY.length).toBeGreaterThanOrEqual(3);
      expect(a.capturePointXY.every(Number.isFinite)).toBe(true);
      expect(a.feetWorldM.left[1]).toBeGreaterThan(0);
      expect(a.feetWorldM.right[1]).toBeLessThan(0);
      const state = sim.step(100);
      const b = sim.humanoidBalanceObservation();
      expect(b.timeS).toBeCloseTo(state.time, 8);
      expect([...b.comWorldM, ...b.comVelocityWorldMps].every(Number.isFinite)).toBe(true);
    } finally {
      sim.dispose();
    }
  }, 30000);
});
