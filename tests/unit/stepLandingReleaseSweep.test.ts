import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Physical exploration, not acceptance; compare only one parameter at a time.
 * Never label 'COMPLETE' from touchdown alone.
 */
describe("Landing joint-release duration comparison in MuJoCo", () => {
  it("measures post-contact slip and recovery for two identical initial states",async () => {
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const results=[];
    for(const servo of [0,0.2]) {
      for(const releaseS of [0,0.55,1.1]) {
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo,releaseS);
          sim.step(2600);
          const status=sim.experimentalStepStatus()!;
          const state=sim.state();
          results.push({
            servo,releaseS,phase:status.phase,
            landing:status.stepContactConfirmed,
            maxLandingSlipM:+status.maxLandingSlipM.toFixed(4),
            maxSupportSlipM:+status.maxSupportFootSlipM.toFixed(4),
            targetErrorM:+status.targetErrorXYM.toFixed(4),
            minPelvisM:+status.minPelvisM.toFixed(4),
            maxTiltDeg:+status.maxRootTiltDeg.toFixed(2),
            stableS:+status.settledDurationS.toFixed(3),
          });
          expect([...state.qpos,...state.qvel].every(Number.isFinite)).toBe(true);
          expect(status.maxLandingSlipM).toBeGreaterThanOrEqual(0);
          expect(status.maxSupportFootSlipM).toBeGreaterThanOrEqual(0);
          if(status.phase==="COMPLETE") {
            expect(status.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(status.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(status.settledDurationS).toBeGreaterThanOrEqual(1.0);
          }
        } finally {sim.dispose();}
      }
    }
    console.log("LANDING-JOINT-RELEASE-SWEEP "+JSON.stringify(results));
    expect(results).toHaveLength(6);
  },120000);
});
