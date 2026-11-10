import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Diagnostic sweep; do not turn legitimate falls into Vitest failures.
 * Find whether increasing trunk/knee coordination improves or degrades
 * the actual ability to shift COM and land on a loaded foot.
 */
describe("Sagittal walking controlled-fall intensity sweep",()=>{
  it("measures actual COM, trunk pitch, knee bend and landing without faking success",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const result=[];
    for(const servo of [0,0.2]) {
      for(const intensity of [0.25,0.5,0.75,1]) {
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0],0.8,servo,intensity);
          const com0=sim.humanoidBalanceObservation().comWorldM;
          let kneeRad=0,torsoRad=0,maxComAdvance=-Infinity;
          for(let sample=0;sample<260;sample++){
            sim.step(10);
            kneeRad=Math.max(kneeRad,-sim.jointAngleRad("left_knee"));
            torsoRad=Math.max(torsoRad,
              sim.jointAngleRad("lumbar_pitch")+sim.jointAngleRad("thorax_pitch"));
            const com=sim.humanoidBalanceObservation().comWorldM;
            maxComAdvance=Math.max(maxComAdvance,com[0]-com0[0]);
          }
          const s=sim.experimentalStepStatus()!;
          const row={
            servo,intensity,outcome:s.phase,
            kneeDeg:+(kneeRad*180/Math.PI).toFixed(1),
            forwardTrunkDeg:+(torsoRad*180/Math.PI).toFixed(1),
            maxComAdvanceM:+maxComAdvance.toFixed(3),
            footRiseM:+s.maxSoleRiseM.toFixed(3),
            contacted:s.stepContactConfirmed,
            finalTargetErrorM:+s.targetErrorXYM.toFixed(3),
            landingSlipM:+s.maxLandingSlipM.toFixed(3),
            minPelvisM:+s.minPelvisM.toFixed(3),
          };
          result.push(row);
          expect([row.kneeDeg,row.forwardTrunkDeg,row.maxComAdvanceM,
            row.footRiseM,row.finalTargetErrorM,row.landingSlipM,row.minPelvisM]
            .every(Number.isFinite)).toBe(true);
          if(s.phase==="COMPLETE"){
            expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
            expect(s.maxLandingSlipM).toBeLessThanOrEqual(0.02);
          }
        }finally{sim.dispose();}
      }
    }
    console.log("GAIT-COM-LEAN-SWEEP "+JSON.stringify(result));
    expect(result).toHaveLength(8);
  },120000);
});
