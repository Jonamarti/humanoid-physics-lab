import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Same MJCF, initial pose, target, integration, and nominal standing control.
 * The ONLY variable is optional friction-triggered hip torque relief.
 * Outcomes remain measured, not tuned to fake a COMPLETE.
 */
describe("Opt-in landing friction torque relief - real 75kg MuJoCo A/B", () => {
  it("compares 0/50/100% relief against baseline with and without Cartesian swing servo", async () => {
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const results=[];
    for(const servo of [0,0.2]){
      for(const relief of [0,0.5,1]){
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo,relief);
          let lowestScale=1;
          let maxFriction=0;
          for(let j=0;j<260;j++){
            sim.step(10);
            lowestScale=Math.min(lowestScale,sim.experimentalContactHipTorqueScale());
            const ground=sim.humanoidGroundSupportObservation();
            maxFriction=Math.max(maxFriction,ground.left.frictionUtilizationMean ?? 0);
          }
          const status=sim.experimentalStepStatus()!;
          const row={
            servo,relief,phase:status.phase,
            stepContactConfirmed:status.stepContactConfirmed,
            lowestHipScale:+lowestScale.toFixed(3),
            maxFriction:+maxFriction.toFixed(3),
            slipCm:+(status.maxLandingSlipM*100).toFixed(2),
            supportSlipCm:+(status.maxSupportFootSlipM*100).toFixed(2),
            targetErrorCm:+(status.targetErrorXYM*100).toFixed(2),
            preLandingSpeedMps:status.preLandingTangentialSpeedMps === null ? null :
              +status.preLandingTangentialSpeedMps.toFixed(3),
            stableS:+status.settledDurationS.toFixed(2),
            minPelvisM:+status.minPelvisM.toFixed(3),
          };
          results.push(row);
          expect(Object.values(row).every(x=>x===null || typeof x==="string" || Number.isFinite(x))).toBe(true);
          if(status.phase==="COMPLETE") {
            expect(status.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(status.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(status.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(status.settledDurationS).toBeGreaterThanOrEqual(1);
          }
          if(relief===0) expect(lowestScale).toBe(1);
        } finally { sim.dispose(); }
      }
    }
    console.log("LANDING-FRICTION-HIP-RELIEF "+JSON.stringify(results));
    expect(results).toHaveLength(6);
  },120000);
});
