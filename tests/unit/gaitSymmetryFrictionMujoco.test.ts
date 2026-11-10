import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Important: first complete step is not proof of robust walking.
 * Cross-check left and right mirror and small friction variations.
 * Every contact and all outcomes come from solver (not interpolated).
 */
describe("Step completion and symmetric friction sensitivity",()=>{
  it("checks bilateral mirror and friction ±10% with unchanged acceptance criteria",async()=>{
    const base=await readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const rows=[];
    for(const friction of [0.9,1,1.1]){
      for(const side of ["left","right"] as const) {
        const xml=base.replaceAll('friction="1.0 0.025 0.002"',
          'friction="'+friction.toFixed(1)+' 0.025 0.002"');
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial(side,[0.12,0],0.8,0.52,1,0.48);
          sim.step(2600);
          const st=sim.experimentalStepStatus()!;
          const row={
            friction,side,result:st.phase,
            landed:st.stepContactConfirmed,
            landingSlipCm:+(st.maxLandingSlipM*100).toFixed(2),
            supportSlipCm:+(st.maxSupportFootSlipM*100).toFixed(2),
            placementErrorCm:+(st.targetErrorXYM*100).toFixed(2),
            stableTimeS:+st.settledDurationS.toFixed(2),
            minPelvisM:+st.minPelvisM.toFixed(3),
            failedGates:Object.entries(st.stability?.checks??{})
              .filter(([,good])=>!good).map(([k])=>k),
          };
          rows.push(row);
          expect([row.landingSlipCm,row.supportSlipCm,row.placementErrorCm,
            row.stableTimeS,row.minPelvisM].every(Number.isFinite)).toBe(true);
          if(st.phase==="COMPLETE"){
            expect(st.stepContactConfirmed).toBe(true);
            expect(st.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(st.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(st.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(st.settledDurationS).toBeGreaterThanOrEqual(1);
          }
        } finally {sim.dispose();}
      }
    }
    console.log("GAIT-SYMMETRY-FRICTION-ROBUSTNESS "+JSON.stringify(rows));
    expect(rows).toHaveLength(6);
  },120000);
});
