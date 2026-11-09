import { readFile } from "node:fs/promises";
import { describe, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { makeNeutralStandingPD, stepNeutralStandingPD } from "../../src/control/standingBaseline";

/** Exploratory sweep: NOT an acceptance test and never interpreted as stable.
 * Records the physical outcome for different joint gains and ankle feedback.
 */
describe("Exploratory physically grounded balance controller tuning", () => {
  it("compares bounded ankle-COM feedback and joint PD configurations", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const mj = await loadMujoco();
    const model = mj.MjModel.from_xml_string(xml);
    const base = makeNeutralStandingPD(model);
    const footIds = ["left_foot_geom", "right_foot_geom"].map(name =>
      (model.geom_bodyid as Int32Array)[model.geom(name).id]!);
    const ankleIds = ["left_ankle_pitch","right_ankle_pitch","left_ankle_roll","right_ankle_roll"].map(name =>
      (model.jnt_dofadr as Int32Array)[model.jnt(name).id]!);
    const setups = [
      { scale: 1, kp: 0, kd: 0, kr: 0, dr: 0 },
      { scale: 0.25, kp: 0, kd: 0, kr: 0, dr: 0 },
      { scale: 0.5, kp: 0, kd: 0, kr: 0, dr: 0 },
      { scale: 0.1, kp: 0, kd: 0, kr: 0, dr: 0 },
      { scale: 0.5, kp: 700, kd: 170, kr: 700, dr: 170 },
      { scale: 0.5, kp: 1200, kd: 300, kr: 1200, dr: 300 },
      { scale: 1, kp: 700, kd: 170, kr: 700, dr: 170 },
      { scale: 1, kp: 1200, kd: 300, kr: 1200, dr: 300 },
      { scale: 0.25, kp: 1200, kd: 300, kr: 1200, dr: 300 },
      { scale: 0.5, kp: 2500, kd: 500, kr: 2000, dr: 400 },
    ];
    const results = [];
    try {
      for (const cfg of setups) {
        const data = new mj.MjData(model);
        try {
          mj.mj_forward(model, data);
          const initialCom = Array.from((data.subtree_com as Float64Array).slice(3, 6));
          const gains = new Map(Array.from(base, ([id, gain]) =>
            [id, {
              ...gain,
              stiffnessNmPerRad: gain.stiffnessNmPerRad * cfg.scale,
              dampingNmsPerRad: gain.dampingNmsPerRad * cfg.scale,
            }] as const));
          let lowest = data.qpos[2]!;
          let maxTilt = 0;
          let contacted = 0;
          let firstFallTime = -1;
          for (let step = 0; step < 1500; step++) {
            stepNeutralStandingPD(model, data, gains);
            if (cfg.kp) {
              mj.mj_subtreeVel(model, data);
              const com = data.subtree_com as Float64Array;
              const vel = data.subtree_linvel as Float64Array;
              const xpos = data.xpos as Float64Array;
              const root = 3;
              const ankX = (xpos[footIds[0]! * 3]! + xpos[footIds[1]! * 3]!) / 2;
              const ankY = (xpos[footIds[0]! * 3 + 1]! + xpos[footIds[1]! * 3 + 1]!) / 2;
              const sagittal = 75 * 9.81 * (com[root]! - ankX) +
                cfg.kp * (com[root]! - initialCom[0]!) + cfg.kd * vel[root]!;
              const lateral = -75 * 9.81 * (com[root+1]! - ankY) -
                cfg.kr * (com[root+1]! - initialCom[1]!) - cfg.dr * vel[root+1]!;
              const cl = (v: number) => Math.max(-160, Math.min(160, v));
              for (const i of ankleIds.slice(0,2)) data.qfrc_applied[i]! = cl(data.qfrc_applied[i]! + sagittal/2);
              for (const i of ankleIds.slice(2)) data.qfrc_applied[i]! = cl(data.qfrc_applied[i]! + lateral/2);
            }
            mj.mj_step(model, data);
            lowest = Math.min(lowest, data.qpos[2]!);
            const qx = data.qpos[4]!, qy = data.qpos[5]!;
            const tilt = Math.acos(Math.max(-1,Math.min(1,1-2*(qx*qx+qy*qy)))) *180/Math.PI;
            maxTilt = Math.max(maxTilt, tilt);
            if (data.ncon > 0) contacted++;
            if (firstFallTime < 0 && (data.qpos[2]! < .8 || tilt > 25)) firstFallTime = data.time;
          }
          results.push({
            ...cfg, minPelvisZ:Number(lowest.toFixed(3)),
            finalPelvisZ:Number(data.qpos[2]!.toFixed(3)),
            maxTilt:Number(maxTilt.toFixed(1)),
            firstFallS:Number(firstFallTime.toFixed(3)),
            contactFraction:Number((contacted/1500).toFixed(2)),
          });
        } finally {
          data.delete();
        }
      }
      console.log("COM-ANKLE STRATEGY SWEEP", JSON.stringify(results));
    } finally {
      model.delete();
    }
  }, 60000);
});
