import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Small deterministic experiment: never silently promote contact to success.
 * Compare exact same physics and step objective, varying only task-space gain.
 */
describe("Landing friction/GRF comparison in real MuJoCo",()=>{
  it("traces base and 20% Jacobian control after loading each foot",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const results=[];
    for(const servo of [0,0.2]) {
      const sim=await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo);
        let oldPhase="";
        const phases=[];
        let maxLeftFriction=0,maxRightFriction=0;
        for(let i=0;i<260;i++) {
          const state=sim.step(10);
          const status=sim.experimentalStepStatus()!;
          const ground=sim.humanoidGroundSupportObservation();
          for(const side of ["left","right"] as const) {
            const mean=ground[side].frictionUtilizationMean;
            if(mean!==null && Number.isFinite(mean)) {
              if(side==="left")maxLeftFriction=Math.max(maxLeftFriction,mean);
              else maxRightFriction=Math.max(maxRightFriction,mean);
            }
          }
          if(status.phase!==oldPhase) {
            oldPhase=status.phase;
            phases.push({
              t:+state.time.toFixed(2),phase:oldPhase,
              leftN:+ground.left.verticalLoadN.toFixed(1),
              rightN:+ground.right.verticalLoadN.toFixed(1),
              leftFriction:ground.left.frictionUtilizationMean?.toFixed(3)??"n/a",
              rightFriction:ground.right.frictionUtilizationMean?.toFixed(3)??"n/a",
              landingSlipM:+status.maxLandingSlipM.toFixed(3),
              supportSlipM:+status.maxSupportFootSlipM.toFixed(3),
              errorM:Number.isFinite(status.targetErrorXYM)
                ? +status.targetErrorXYM.toFixed(3) : null,
            });
          }
        }
        const outcome=sim.experimentalStepStatus()!;
        results.push({servo,outcome:outcome.phase,phases,
          maxLeftFriction:+maxLeftFriction.toFixed(3),
          maxRightFriction:+maxRightFriction.toFixed(3),
          maxLandingSlipM:+outcome.maxLandingSlipM.toFixed(3),
          maxSupportSlipM:+outcome.maxSupportFootSlipM.toFixed(3),
          finalErrorM:+outcome.targetErrorXYM.toFixed(3),
          stableS:+outcome.settledDurationS.toFixed(2),
          preLandingXYSpeedMps:outcome.preLandingTangentialSpeedMps===null
            ? null : +outcome.preLandingTangentialSpeedMps.toFixed(3),
          preLandingDownSpeedMps:outcome.preLandingDownwardSpeedMps===null
            ? null : +outcome.preLandingDownwardSpeedMps.toFixed(3),
          frictionAtLanding:outcome.frictionUtilizationAtLanding===null
            ? null : +outcome.frictionUtilizationAtLanding.toFixed(3)});
        if(outcome.phase==="COMPLETE") {
          expect(outcome.settledDurationS).toBeGreaterThanOrEqual(1);
          expect(outcome.maxLandingSlipM).toBeLessThanOrEqual(0.02);
        }
        expect([outcome.maxLandingSlipM,outcome.maxSupportFootSlipM,
          outcome.targetErrorXYM].every(Number.isFinite)).toBe(true);
      } finally {sim.dispose();}
    }
    console.log("FRICTION-STEP-COMPARISON "+JSON.stringify(results));
    expect(results).toHaveLength(2);
  },120000);
});
