import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { ExperimentalStepSequence } from "../../src/control/experimentalStep";

describe("Experimental LIFT hip-angle sweep with measured contact velocity", () => {
  it("preserves baseline and validates the optional hip reference",()=>{
    const s=new ExperimentalStepSequence("left",[0.12,0.08]);
    expect(s.liftHipPitchRad).toBe(-0.15);
    expect(new ExperimentalStepSequence("left",[0.12,0.08],0.8,0).liftHipPitchRad).toBe(0);
    expect(()=>new ExperimentalStepSequence("left",[0.12,0.08],0.8,NaN)).toThrow(RangeError);
    expect(()=>new ExperimentalStepSequence("left",[0.12,0.08],0.8,0.5)).toThrow(RangeError);
  });
  it("compares 3 lift hip targets at baseline and 20% task-space servo",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows=[];
    for (const servo of [0,0.2]){
      for(const liftHipPitchRad of [-0.15,-0.06,0]){
        const sim=await MujocoSimulation.create(xml);
        try{
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo,liftHipPitchRad);
          sim.step(2600);
          const status=sim.experimentalStepStatus()!;
          rows.push({
            servo,liftHipPitchRad,outcome:status.phase,
            loadedLanding:status.stepContactConfirmed,
            preLandingXYSpeedMps:status.preLandingTangentialSpeedMps===null ? null :
              +status.preLandingTangentialSpeedMps.toFixed(3),
            preLandingDownMps:status.preLandingDownwardSpeedMps===null ? null :
              +status.preLandingDownwardSpeedMps.toFixed(3),
            touchdownFriction:status.frictionUtilizationAtLanding===null ? null :
              +status.frictionUtilizationAtLanding.toFixed(3),
            slipM:+status.maxLandingSlipM.toFixed(3),
            supportSlipM:+status.maxSupportFootSlipM.toFixed(3),
            targetErrorM:+status.targetErrorXYM.toFixed(3),
            minPelvisM:+status.minPelvisM.toFixed(3),
          });
          expect([status.maxLandingSlipM,status.targetErrorXYM,status.minPelvisM]
            .every(Number.isFinite)).toBe(true);
          if(status.phase==="COMPLETE"){
            expect(status.settledDurationS).toBeGreaterThanOrEqual(1);
            expect(status.maxLandingSlipM).toBeLessThanOrEqual(0.02);
          }
        }finally{sim.dispose();}
      }
    }
    console.log("LIFT-HIP-ANGLE-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(6);
  },120000);
});
