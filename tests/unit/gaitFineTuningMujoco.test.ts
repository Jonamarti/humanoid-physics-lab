import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** A narrow, deterministic grid around two promising joint-controller
 * settings. No changes to model, torque bounds, contact physics, root or
 * acceptance thresholds.
 */
describe("Narrow knee extension guard sweep for one stable forward step",()=>{
  it("measures whether any tested setting satisfies all unchanged stability gates for >=1s",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const rows=[];
    for(const servo of [0.45,0.48,0.5,0.52]){
      for(const guard of [0.38,0.42,0.46,0.48]){
        const sim=await MujocoSimulation.create(xml);
        try{
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0],0.8,servo,1,guard);
          let maxStableS=0;
          sim.step(2600);
          const s=sim.experimentalStepStatus()!;
          maxStableS=Math.max(maxStableS,s.settledDurationS);
          const row={
            servo,guard,phase:s.phase,
            contacted:s.stepContactConfirmed,
            slipCm:+(100*s.maxLandingSlipM).toFixed(2),
            targetErrorCm:+(100*s.targetErrorXYM).toFixed(2),
            supportSlipCm:+(100*s.maxSupportFootSlipM).toFixed(2),
            stableS:+maxStableS.toFixed(2),
            failedGates:Object.entries(s.stability?.checks??{})
              .filter(([,ok])=>!ok).map(([name])=>name),
            minPelvisM:+s.minPelvisM.toFixed(3),
          };
          rows.push(row);
          expect([row.slipCm,row.targetErrorCm,row.supportSlipCm,row.minPelvisM]
            .every(Number.isFinite)).toBe(true);
          if(s.phase==="COMPLETE"){
            expect(s.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(s.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(s.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
          }
        }finally{sim.dispose();}
      }
    }
    console.log("GAIT-FINE-TUNING-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(16);
  },120000);
});
