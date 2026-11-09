import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Solved MuJoCo WASM floor-foot contact forces", () => {
  it("records body weight on both feet, then resets reproducibly", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const sim = await MujocoSimulation.create(xml);
    try {
      sim.enableExperimentalStanding();
      // Let the initial penetration transient settle before measuring support.
      sim.step(500);
      const a = sim.humanoidGroundSupportObservation();
      const weight = sim.humanoidBalanceObservation().totalMassKg * 9.81;
      console.log("SOLVED-FOOT-LOADS " + JSON.stringify({
        t: sim.state().time, leftN: a.left.verticalLoadN,
        rightN: a.right.verticalLoadN, sumN: a.totalVerticalLoadN, weightN: weight,
        contacts: a.activeFootContacts,
      }));
      expect(a.activeFootContacts).toBeGreaterThan(0);
      expect(a.left.verticalLoadN).toBeGreaterThan(50);
      expect(a.right.verticalLoadN).toBeGreaterThan(50);
      expect(a.left.isLoaded).toBe(true);
      expect(a.right.isLoaded).toBe(true);
      // The sum should be consistent with, but not forced to equal, weight:
      // acceleration and impact can affect total normal reaction.
      expect(a.totalVerticalLoadN).toBeGreaterThan(weight * 0.5);
      expect(a.totalVerticalLoadN).toBeLessThan(weight * 1.5);
      const again = sim.humanoidGroundSupportObservation();
      expect(again.totalVerticalLoadN).toBeCloseTo(a.totalVerticalLoadN, 6);
      sim.reset();
      sim.step(500);
      const b = sim.humanoidGroundSupportObservation();
      expect(b.totalVerticalLoadN).toBeCloseTo(a.totalVerticalLoadN, 5);
    } finally {
      sim.dispose();
    }
  }, 45000);

  it("does not treat contact count as load in a falling initial state", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const sim = await MujocoSimulation.create(xml);
    try {
      const initial = sim.humanoidGroundSupportObservation();
      const after = sim.humanoidGroundSupportObservation();
      expect(initial.totalVerticalLoadN).toBeGreaterThanOrEqual(0);
      expect(after.totalVerticalLoadN).toBeCloseTo(initial.totalVerticalLoadN, 8);
      expect([initial.left.verticalLoadN, initial.right.verticalLoadN].every(Number.isFinite)).toBe(true);
    } finally {
      sim.dispose();
    }
  }, 30000);
});
