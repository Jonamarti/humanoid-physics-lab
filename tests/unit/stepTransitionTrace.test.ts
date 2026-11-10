import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Experimental step debug trace: real per-phase foot contacts", () => {
  it("records physical phase changes without asserting a successful step", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url),"utf8");
    for (const [label,delta] of [
      ["left-outward",[0,0.12]],
      ["left-diagonal",[0.12,0.08]],
      ["left-forward",[0.12,0]],
    ] as const) {
      const sim = await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial("left",delta,0.8);
        const origin = sim.siteWorldPosition("left_sole");
        let previous = "";
        const changes = [];
        for (let s=0;s<2600;s+=10) {
          const state=sim.step(10);
          const st=sim.experimentalStepStatus()!;
          if(st.phase!==previous) {
            previous=st.phase;
            const pos=sim.siteWorldPosition("left_sole");
            const loads=sim.humanoidGroundSupportObservation();
            changes.push({t:+state.time.toFixed(3),phase:st.phase,
              x:+(pos[0]-origin[0]).toFixed(3),y:+(pos[1]-origin[1]).toFixed(3),
              rise:+(pos[2]-origin[2]).toFixed(3),
              swingN:+loads.left.verticalLoadN.toFixed(0),
              supportN:+loads.right.verticalLoadN.toFixed(0),
              progress:+st.currentTargetProgressM.toFixed(3),
              pelvis:+state.qpos[2]!.toFixed(3),
              reason:st.reason,
            });
          }
        }
        console.log("STEP-TRACE "+label+" "+JSON.stringify(changes));
        expect(changes.length).toBeGreaterThan(2);
      } finally {sim.dispose();}
    }
  },120000);
});
