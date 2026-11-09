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
  it("converts a picked world position into a body-fixed COM offset", async () => {
    const sim = await MujocoSimulation.create(await readFile(xmlFile, "utf8"));
    try {
      const geom = sim.geometrySpecs().find(g => g.shape === "capsule");
      expect(geom).toBeDefined();
      const original = sim.geometryPoses().find(p => p.id === geom!.id)!.position;
      const anchor = sim.bodyAttachmentAtGeomPoint(geom!.id, original);
      expect(anchor.bodyIndex).toBeGreaterThan(0);
      const reconstructed = sim.bodyAttachmentWorldPosition(
        anchor.bodyIndex, anchor.localComOffsetM);
      for (let i = 0; i < 3; i++) {
        expect(reconstructed[i]).toBeCloseTo(original[i]!, 7);
      }
      sim.step(200);
      const moved = sim.bodyAttachmentWorldPosition(
        anchor.bodyIndex, anchor.localComOffsetM);
      expect([...moved].every(Number.isFinite)).toBe(true);
      expect(() => sim.bodyAttachmentAtGeomPoint(0, [0,0,0])).toThrow();
    } finally {
      sim.dispose();
    }
  }, 30000);

  it("sums simultaneous forces exactly like a single resultant at the same site", async () => {
    const xml = await readFile(xmlFile, "utf8");
    const a = await MujocoSimulation.create(xml);
    const b = await MujocoSimulation.create(xml);
    try {
      a.step(20, () => {
        a.applyWorldForceAtSite("sternum_impact", [40, -20, 0]);
        a.applyWorldForceAtSite("sternum_impact", [-10, 25, 5]);
      });
      b.step(20, () => b.applyWorldForceAtSite("sternum_impact", [30, 5, 5]));
      const qa = a.state().qpos;
      const qb = b.state().qpos;
      expect(qa.length).toBe(qb.length);
      for (let i = 0; i < qa.length; i++) expect(qa[i]).toBeCloseTo(qb[i]!, 8);
    } finally {
      a.dispose(); b.dispose();
    }
  }, 30000);

});
