import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { GRAVITY_DEMO_MJCF } from "../../src/physics/demoModel";

/** Uses the actual WASM physics engine, not a mock.
 * Node can load the packaged WASM relative to @mujoco/mujoco.
 */
describe("MuJoCo WASM rigid body integration", () => {
  it("falls under gravity, contacts the floor, and resets", async () => {
    const mj = await loadMujoco();
    const model = mj.MjModel.from_xml_string(GRAVITY_DEMO_MJCF);
    expect(model).toBeTruthy();
    const data = new mj.MjData(model);
    try {
      mj.mj_forward(model, data);
      expect(data.qpos[2]).toBeCloseTo(1.5, 8);
      for (let i = 0; i < 400; i++) mj.mj_step(model, data);
      expect(data.time).toBeCloseTo(0.8, 8);
      // Sphere comes to rest near center z = radius 0.12m.
      expect(data.qpos[2]).toBeGreaterThan(0.09);
      expect(data.qpos[2]).toBeLessThan(0.2);
      mj.mj_resetData(model, data);
      mj.mj_forward(model, data);
      expect(data.qpos[2]).toBeCloseTo(1.5, 8);
      expect(data.time).toBe(0);
    } finally {
      data.delete();
      model.delete();
    }
  }, 30000);
});
