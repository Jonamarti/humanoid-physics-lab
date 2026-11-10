import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Use genuinely measured ground contacts to select a useful first-step
 * actuator gain; do not change pass/fail thresholds to manufacture success.
 */
describe("Knee/trunk landing sensitivity on fixed 75kg MuJoCo model",()=>{
  it("compares foot target and bounded Cartesian swing corrections",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const rows=[];
    for(const stepLength of [0.10,0.12]){
      for(const servo of [0.2,0.35,0.5,0.75]){
        const sim=await MujocoSimulation.create(xml);
        try{
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[stepLength,0],0.8,servo,1);
          sim.step(2600);
          const s=sim.experimentalStepStatus()!;
          const q=sim.humanoidBalanceObservation();
          const row={
            stepLength,servo,result:s.phase,
            landed:s.stepContactConfirmed,
            advanceM:+s.currentTargetProgressM.toFixed(3),
            slipM:+s.maxLandingSlipM.toFixed(3),
            targetErrorM:+s.targetErrorXYM.toFixed(3),
            supportSlipM:+s.maxSupportFootSlipM.toFixed(3),
            finalComX:+q.comWorldM[0].toFixed(3),
            stableTimeS:+s.settledDurationS.toFixed(2),
            minPelvisM:+s.minPelvisM.toFixed(3),
          };
          rows.push(row);
          expect([row.advanceM,row.slipM,row.targetErrorM,
            row.supportSlipM,row.finalComX,row.minPelvisM].every(Number.isFinite)).toBe(true);
          if(s.phase==="COMPLETE") {
            expect(s.stepContactConfirmed).toBe(true);
            expect(s.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(s.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
          }
        }finally{sim.dispose();}
      }
    }
    console.log("GAIT-LANDING-SERVO-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(8);
  },120000);
});
