import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { estimateContactPatchSlip } from "../../src/physics/contactPatchSlip";

/** v3 is opt-in; these are structural and solver-observability tests, not
 * a claim that walking / propulsion has been solved. */
describe("Articulated forefoot v3 (isolated physical model)", () => {
  it("loads at the same nominal 75kg and adds two independent toe segments", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_gait_v3.xml", import.meta.url), "utf8");
    const baseline = await readFile(new URL("../../public/models/humanoid_step_v2.xml", import.meta.url), "utf8");
    for (const side of ["left", "right"]) {
      expect(xml).toContain(`name="${side}_mtp"`);
      expect(xml).toContain(`name="${side}_toe_geom"`);
      expect(baseline).not.toContain(`name="${side}_toe_geom"`);
    }
    const sim = await MujocoSimulation.create(xml);
    const old = await MujocoSimulation.create(baseline);
    try {
      const current = sim.humanoidBalanceObservation();
      const prior = old.humanoidBalanceObservation();
      expect(current.totalMassKg).toBeCloseTo(prior.totalMassKg, 5);
      expect(current.totalMassKg).toBeCloseTo(75, 5);
      expect(sim.state().qpos.length).toBe(old.state().qpos.length + 2);
      const toeGeoms = sim.geometrySpecs().filter(g =>
        g.shape === "box" && Math.abs(g.size[0] - 0.0525) < 1e-8);
      expect(toeGeoms).toHaveLength(2);
      const v3 = sim.humanoidGroundSupportObservation();
      const v2 = old.humanoidGroundSupportObservation();
      expect(v3.left.segments).toBeDefined();
      expect(v3.right.segments).toBeDefined();
      expect(v2.left.segments).toBeUndefined();
    } finally {
      sim.dispose();
      old.dispose();
    }
  }, 45000);

  it("measures segment loads from MuJoCo without inventing toe-off", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_gait_v3.xml", import.meta.url), "utf8");
    const sim = await MujocoSimulation.create(xml);
    try {
      sim.enableExperimentalStanding();
      sim.step(250); // 0.5s; diagnostic, not evidence of stable 10s stance
      const a = sim.humanoidGroundSupportObservation();
      for (const foot of [a.left, a.right]) {
        const parts = foot.segments!;
        expect(parts).toBeDefined();
        expect(parts.rearfoot.verticalLoadN).toBeGreaterThanOrEqual(0);
        expect(parts.forefoot.verticalLoadN).toBeGreaterThanOrEqual(0);
        expect(parts.rearfoot.verticalLoadN + parts.forefoot.verticalLoadN)
          .toBeCloseTo(foot.verticalLoadN, 7);
        expect(parts.rearfoot.activeContacts + parts.forefoot.activeContacts)
          .toBe(foot.activeContacts);
        expect(parts.rearfoot.tangentialReactionXYN[0] +
          parts.forefoot.tangentialReactionXYN[0]).toBeCloseTo(foot.tangentialReactionXYN[0], 7);
      }
      expect(Number.isFinite(a.totalVerticalLoadN)).toBe(true);
      // No REQUIREMENT of a forefoot load in nominal stance: document the raw
      // physical outcome rather than synthesizing a successful roll-over.
      console.log("ARTICULATED-FOREFOOT-V3 " + JSON.stringify({
        timeS:sim.state().time,
        leftRearN:a.left.segments!.rearfoot.verticalLoadN,
        leftToeN:a.left.segments!.forefoot.verticalLoadN,
        rightRearN:a.right.segments!.rearfoot.verticalLoadN,
        rightToeN:a.right.segments!.forefoot.verticalLoadN,
        totalN:a.totalVerticalLoadN,
      }));
    } finally {
      sim.dispose();
    }
  }, 45000);

  it("does not infer material slip from a toe/heel contact centroid crossing bodies", () => {
    const pose = {centerWorldM:[0,0,0] as const,
      rotationWorld:[1,0,0,0,1,0,0,0,1] as const};
    const staticToe = estimateContactPatchSlip(pose,pose,[0.15,0,0],0.02);
    expect(staticToe.tangentialSpeedMps).toBeCloseTo(0,9);
    const displaced = estimateContactPatchSlip(pose,{
      ...pose, centerWorldM:[0.01,0,0] as const},[0.15,0,0],0.02);
    expect(displaced.tangentialSpeedMps).toBeCloseTo(0.5,7);
    // This estimator must be invoked separately with each segment's pose in
    // future rolling-gait traces; do not reuse rearfoot transform for toes.
  });
});
