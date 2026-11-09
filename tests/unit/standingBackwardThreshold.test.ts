import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { pushForceAtTime, rootTiltDegrees, expectedPushImpulseNs, type PushTrial } from "../../src/physics/standingPush";

describe("Backward push failure boundary: measured, not pre-programmed", () => {
  it("locates the first failed force in a fixed 0.12 s backward pulse series", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const peakForces = [120, 160, 180, 200, 220, 240, 260] as const;
    const outcomes: Array<{ peakN: number; impulseNs: number; lowestPelvisM: number;
      maxTiltDeg: number; finalPelvisM: number; fell: boolean }> = [];
    for (const peakForceN of peakForces) {
      const sim = await MujocoSimulation.create(xml);
      const trial: PushTrial = { direction: "backward", peakForceN, startS: 1, durationS: 0.12 };
      try {
        sim.enableExperimentalStanding();
        let lowestPelvisM = sim.state().qpos[2]!;
        let maxTiltDeg = 0;
        for (let step = 0; step < 3000; step += 20) {
          const state = sim.step(20, t => {
            const f = pushForceAtTime(t, trial);
            if (f[0] !== 0) sim.applyWorldForceAtSite("sternum_impact", f);
          });
          lowestPelvisM = Math.min(lowestPelvisM, state.qpos[2]!);
          maxTiltDeg = Math.max(maxTiltDeg, rootTiltDegrees(state.qpos));
        }
        const finalPelvisM = sim.state().qpos[2]!;
        outcomes.push({
          peakN: peakForceN,
          impulseNs: Number(expectedPushImpulseNs(trial).toFixed(2)),
          lowestPelvisM: Number(lowestPelvisM.toFixed(3)),
          maxTiltDeg: Number(maxTiltDeg.toFixed(2)),
          finalPelvisM: Number(finalPelvisM.toFixed(3)),
          fell: lowestPelvisM < 0.8 || maxTiltDeg > 45,
        });
      } finally {
        sim.dispose();
      }
    }
    console.log("BACKWARD-THRESHOLD-6S " + JSON.stringify(outcomes));
    expect(outcomes).toHaveLength(peakForces.length);
    expect(outcomes[0]!.fell).toBe(false); // from the baseline 120 N measurement
    expect(outcomes.find(r => r.peakN === 240)?.fell).toBe(true); // documented failure
  }, 120000);
});
