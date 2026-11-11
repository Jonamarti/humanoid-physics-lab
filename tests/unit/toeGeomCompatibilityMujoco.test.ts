import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Regression: querying an absent name with MuJoCo's Embind named accessor
 * can poison the WASM runtime even when a JavaScript catch hides the error.
 * v1/v2 must never probe v3-specific geoms on every support sample.
 */
describe("MuJoCo named-geom compatibility after articulated toe support", () => {
  it("samples all three models repeatedly and still advances the legacy gait", async () => {
    for (const filename of ["humanoid_v1.xml","humanoid_step_v2.xml",
      "humanoid_gait_v3.xml"] as const) {
      const xml = await readFile(new URL("../../public/models/"+filename,import.meta.url),"utf8");
      const sim=await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        for(let index=0;index<50;index++){
          sim.step(10);
          const support=sim.humanoidGroundSupportObservation();
          expect(Number.isFinite(support.totalVerticalLoadN)).toBe(true);
          expect(support.totalVerticalLoadN).toBeGreaterThanOrEqual(0);
          expect(Boolean(support.left.segments)).toBe(filename==="humanoid_gait_v3.xml");
          expect(Boolean(support.right.segments)).toBe(filename==="humanoid_gait_v3.xml");
        }
        const time=sim.state().time;
        expect(time).toBeCloseTo(1,5);
        sim.reset();
        sim.step(25);
        expect(sim.state().time).toBeCloseTo(0.05,6);
      } finally {sim.dispose();}
    }
  },60000);
});
