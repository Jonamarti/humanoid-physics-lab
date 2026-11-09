import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";

const xmlFile = new URL("../../public/models/humanoid_v1.xml", import.meta.url);

/** Integration tests against the actual native WebAssembly physics bindings.
 * This is an UNCONTROLLED ragdoll; staying upright belongs to issue #9.
 */
describe("Public humanoid MJCF v1", () => {
  it("loads, has a 75kg segmented body and free pelvis without motors", async () => {
    const mj = await loadMujoco();
    const xml = await readFile(xmlFile, "utf8");
    const model = mj.MjModel.from_xml_string(xml);
    expect(model).toBeTruthy();
    const data = new mj.MjData(model);

    try {
      mj.mj_forward(model, data);
      expect(model.nbody).toBeGreaterThanOrEqual(15);
      expect(model.njnt).toBeGreaterThanOrEqual(20);
      expect(model.nq).toBeGreaterThan(20);
      expect(model.nu).toBe(0); // no motors or external root support
      const totalMass = Array.from(model.body_mass as Float64Array).reduce((sum, mass) => sum + mass, 0);
      expect(totalMass).toBeCloseTo(75, 3);
      expect(data.qpos[2]).toBeCloseTo(0.99, 5);
      expect(Array.from(data.qpos).every(Number.isFinite)).toBe(true);
    } finally {
      data.delete();
      model.delete();
    }
  }, 30000);

  it("responds to gravity and a passive disturbance, then resets", async () => {
    const mj = await loadMujoco();
    const model = mj.MjModel.from_xml_string(await readFile(xmlFile, "utf8"));
    const data = new mj.MjData(model);

    try {
      mj.mj_forward(model, data);
      // Free pelvis: a small lateral velocity should propagate through the physical rig.
      data.qvel[1] = 0.35;
      let sawGroundContact = false;
      for (let i = 0; i < 500; i++) {
        mj.mj_step(model, data);
        sawGroundContact ||= data.ncon > 0;
        if (i % 50 === 0) {
          expect(Array.from(data.qpos).every(Number.isFinite)).toBe(true);
          expect(Array.from(data.qvel).every(Number.isFinite)).toBe(true);
        }
      }
      expect(data.time).toBeCloseTo(1.0, 6);
      expect(sawGroundContact).toBe(true);
      expect(Math.abs(data.qpos[1]!)).toBeGreaterThan(0.001);
      mj.mj_resetData(model, data);
      mj.mj_forward(model, data);
      expect(data.time).toBe(0);
      expect(data.qpos[2]).toBeCloseTo(0.99, 5);
    } finally {
      data.delete();
      model.delete();
    }
  }, 30000);
});
