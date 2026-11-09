import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { makeNeutralStandingPD, stepNeutralStandingPD } from "../../src/control/standingBaseline";

function rootTiltDegrees(qpos: Float64Array): number {
  const x = qpos[4]!, y = qpos[5]!;
  const worldVertical = Math.max(-1, Math.min(1, 1 - 2 * (x * x + y * y)));
  return Math.acos(worldVertical) * 180 / Math.PI;
}

describe("Experimental 10-second neutral standing (actual MuJoCo WASM)", () => {
  it("holds posture only using bounded internal torques and ground contact", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const mj = await loadMujoco();
    const model = mj.MjModel.from_xml_string(xml);
    const data = new mj.MjData(model);
    const gains = makeNeutralStandingPD(model);
    const report: { t: number; pelvisZ: number; tiltDeg: number; contacts: number }[] = [];
    try {
      mj.mj_forward(model, data);
      const initialHeight = data.qpos[2]!;
      let worstTilt = 0;
      let lowestPelvis = initialHeight;
      let saturatedSteps = 0;
      for (let i = 0; i < 5000; i++) {
        stepNeutralStandingPD(model, data, gains);
        const applied = data.qfrc_applied;
        expect(Array.from(applied.subarray(0, 6))).toEqual([0, 0, 0, 0, 0, 0]);
        const maxNm = Math.max(...Array.from(applied, Math.abs));
        if (maxNm >= 169.9) saturatedSteps++;
        mj.mj_step(model, data);
        worstTilt = Math.max(worstTilt, rootTiltDegrees(data.qpos));
        lowestPelvis = Math.min(lowestPelvis, data.qpos[2]!);
        if ((i + 1) % 1000 === 0) {
          report.push({
            t: Number(data.time.toFixed(3)),
            pelvisZ: Number(data.qpos[2]!.toFixed(3)),
            tiltDeg: Number(rootTiltDegrees(data.qpos).toFixed(1)),
            contacts: data.ncon,
          });
        }
      }
      console.log("10s neutral stand telemetry", JSON.stringify({
        report, finalPelvisZ:data.qpos[2], initialHeight, worstTilt, lowestPelvis, saturatedSteps,
      }));
      expect(data.time).toBeCloseTo(10, 6);
      expect(Array.from(data.qpos).every(Number.isFinite)).toBe(true);
      expect(lowestPelvis).toBeGreaterThan(initialHeight * 0.80);
      expect(worstTilt).toBeLessThan(15);
    } finally {
      data.delete();
      model.delete();
    }
  }, 60000);
});
