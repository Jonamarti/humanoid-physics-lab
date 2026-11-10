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
        if (status!.phase === "COMPLETE") {
          // Passing a test suite does not prove a step: only solver-backed
          // sustained loaded support with low slip can be called complete.
          expect(status!.settledDurationS).toBeGreaterThanOrEqual(1);
          expect(status!.maxLandingSlipM).toBeLessThanOrEqual(0.02);
          expect(status!.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
          expect(status!.targetErrorXYM).toBeLessThanOrEqual(0.05);
        }
        // An incomplete step is deliberately not called complete; phase
        // and measured take-off/landing are evidence, not fixed presets.
      } finally {sim.dispose();}
    }
    console.log("EXPERIMENTAL-STEP-PHYSICS "+JSON.stringify(rows));
    expect(rows).toHaveLength(2);
  }, 120000);
  it("characterizes outward, straight and backward steps independently", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url),"utf8");
    const rows = [];
    for (const side of ["left","right"] as const) {
      for (const direction of ["outward","forward","backward"] as const) {
        const sign = side === "left" ? 1 : -1;
        const delta: readonly [number, number] = direction === "outward"
          ? [0, sign * 0.12] : direction === "forward" ? [0.12,0] : [-0.10,0];
        const sim = await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial(side,delta,0.8);
          const first = sim.siteWorldPosition(side+"_sole");
          sim.step(2600);
          const last = sim.siteWorldPosition(side+"_sole");
          const status = sim.experimentalStepStatus()!;
          rows.push({
            side,direction,outcome:status.phase,contact:status.stepContactConfirmed,
            maxRiseM:+status.maxSoleRiseM.toFixed(4),
            maxTravelM:+status.maxSoleTravelXYM.toFixed(4),
            actualDeltaX:+(last[0]-first[0]).toFixed(4),
            actualDeltaY:+(last[1]-first[1]).toFixed(4),
          });
          expect(status.maxSoleTravelXYM).toBeGreaterThan(0.001);
          expect([last[0],last[1],last[2]].every(Number.isFinite)).toBe(true);
        } finally {sim.dispose();}
      }
    }
    console.log("DIRECTIONAL-PHYSICAL-STEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(6);
  }, 120000);

});
