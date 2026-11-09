import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Real MuJoCo voluntary-step actuation, not a keyframe animation", () => {
  it("attempts a step with internal torques and measures actual load and foot movement", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows = [];
    for (const side of ["left","right"] as const) {
      const sim = await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial(side,side==="left" ? [0.12,0.08] : [0.12,-0.08],0.8);
        const initial = sim.siteWorldPosition(side+"_sole");
        const state = sim.step(2600);
        const status = sim.experimentalStepStatus();
        const final = sim.siteWorldPosition(side+"_sole");
        const physicalTravelM = Math.hypot(final[0]-initial[0],final[1]-initial[1]);
        rows.push({
          side,phase:status?.phase,maxRiseM:status?.maxSoleRiseM,
          maxTravelM:status?.maxSoleTravelXYM,maxSupportFraction:status?.maxSupportLoadFraction,
          landingConfirmed:status?.stepContactConfirmed,finalTravelM:physicalTravelM,
          pelvisM:state.qpos[2],
        });
        expect(state.time).toBeCloseTo(5.2,5);
        expect([...state.qpos,...state.qvel].every(Number.isFinite)).toBe(true);
        expect(status).not.toBeNull();
        expect(status!.maxSupportLoadFraction).toBeGreaterThan(0);
        expect(status!.maxSoleTravelXYM).toBeGreaterThan(0.001);
        // An incomplete step is deliberately not called complete; phase
        // and measured take-off/landing are evidence, not fixed presets.
      } finally {sim.dispose();}
    }
    console.log("EXPERIMENTAL-STEP-PHYSICS "+JSON.stringify(rows));
    expect(rows).toHaveLength(2);
  }, 120000);
});
