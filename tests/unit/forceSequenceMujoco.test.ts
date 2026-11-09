import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { editedForceAtTime, type EditedForce } from "../../src/physics/forceSequence";

const xmlFile = new URL("../../public/models/humanoid_v1.xml", import.meta.url);

describe("Multiple simultaneous forces in actual MuJoCo", () => {
  it("accepts new attachment sites and additive physics without fabricating root motion", async () => {
    const sim = await MujocoSimulation.create(await readFile(xmlFile, "utf8"));
    try {
      for (const site of ["sternum_impact", "pelvis_origin", "left_shoulder_impact",
        "right_shoulder_impact", "left_forearm_impact", "right_forearm_impact"]) {
        expect(sim.siteWorldPosition(site).every(Number.isFinite)).toBe(true);
      }
      const forces: EditedForce[] = [
        { id: "a", site: "left_forearm_impact", forceN: [50, 0, 0],
          startS: 0, durationS: 0.2, waveform: "constant" },
        { id: "b", site: "right_forearm_impact", forceN: [-50, 0, 0],
          startS: 0, durationS: 0.2, waveform: "constant" },
      ];
      sim.step(100, timeS => {
        for (const f of forces) {
          const force = editedForceAtTime(timeS, f);
          if (Math.hypot(...force)) sim.applyWorldForceAtSite(f.site, force);
        }
      });
      const state = sim.state();
      expect(state.time).toBeCloseTo(0.2, 6);
      expect([...state.qpos, ...state.qvel].every(Number.isFinite)).toBe(true);
    } finally {
      sim.dispose();
    }
  }, 30000);
});
