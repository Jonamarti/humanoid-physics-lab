import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

const footPositions = (sim: MujocoSimulation): Map<number, readonly [number,number,number]> => {
  const ids = sim.geometrySpecs().filter(s =>
    s.shape === "box" && Math.abs(s.size[2] - 0.035) < 1e-6).map(s => s.id);
  const poses = sim.geometryPoses().filter(p => ids.includes(p.id));
  return new Map(poses.map(p => [p.id, [...p.position]]));
};
const horizontalDistance = (a: readonly number[], b: readonly number[]): number =>
  Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!);

describe("Browser runtime integration: ten-second active stance", () => {
  it("stays upright with bounded joint feedback, no foot teleportation, and repeatable reset", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const active = await MujocoSimulation.create(xml);
    const passive = await MujocoSimulation.create(xml);
    try {
      active.enableExperimentalStanding();
      const feet0 = footPositions(active);
      expect(feet0.size).toBe(2);
      const first = active.step(5000);
      const feet1 = footPositions(active);
      expect(first.time).toBeCloseTo(10, 6);
      expect(first.qpos[2]).toBeGreaterThan(0.8);
      for (const [id, p0] of feet0) {
        const p1 = feet1.get(id)!;
        expect(horizontalDistance(p0, p1)).toBeLessThan(0.02);
      }
      const uncontrolled = passive.step(5000);
      expect(uncontrolled.qpos[2]).toBeLessThan(0.8);
      active.reset();
      const repeated = active.step(5000);
      expect(repeated.time).toBeCloseTo(10, 6);
      expect(repeated.qpos[2]).toBeCloseTo(first.qpos[2]!, 3);
    } finally {
      active.dispose();
      passive.dispose();
    }
  }, 60000);
});
