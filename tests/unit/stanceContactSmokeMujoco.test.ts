import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { estimateContactPatchSlip, materialContactLocalM, type MeasuredFootPose } from "../../src/physics/contactPatchSlip";

/**
 * Short, real-physics A/B smoke test. This does NOT certify walking, but
 * unlike opt-in long sweeps it makes the experimental contact code execute
 * in the normal test suite. It compares both feet with identical MuJoCo
 * ground/model, gait settings and measured physical contact forces.
 *
 * A non-COMPLETE experimental result is evidence of failure, not a pass
 * disguised as success. The calibrated baseline MUST remain COMPLETE.
 */
describe("Real MuJoCo contact-controller A/B smoke (both feet)", () => {
  it("records forward ground reaction, material slip and actual step outcome", async () => {
    const xml = await readFile(
      new URL("../../public/models/humanoid_step_v2.xml", import.meta.url), "utf8");
    const rows = [];
    for (const swing of ["left", "right"] as const) {
      for (const {stanceGain,mode,toeRollStrength} of [
        {stanceGain:0,mode:"combined",toeRollStrength:0},
        {stanceGain:0.35,mode:"combined",toeRollStrength:0},
        {stanceGain:0.35,mode:"push-only",toeRollStrength:0},
        {stanceGain:0.35,mode:"slip-only",toeRollStrength:0},
        {stanceGain:0.35,mode:"recovery-only",toeRollStrength:0},
        {stanceGain:0.35,mode:"toe-gated",toeRollStrength:0},
        {stanceGain:0,mode:"combined",toeRollStrength:0.5},
        {stanceGain:0.35,mode:"toe-gated",toeRollStrength:0.5},
        {stanceGain:0.35,mode:"toe-gated",toeRollStrength:1},
      ] as const) {
        const sim = await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial(swing, [0.12, 0], 0.8,
            0.52, 1, 0.48, stanceGain, mode, toeRollStrength);
          const support = swing === "left" ? "right" : "left";
          const specs = sim.geometrySpecs().filter(spec =>
            spec.shape === "box" && Math.abs(spec.size[2] - 0.035) < 1e-6);
          const initialPoses = sim.geometryPoses();
          const supportGeom = specs.find(spec => {
            const geom = initialPoses.find(p => p.id === spec.id);
            return geom && (support === "left" ? geom.position[1] > 0 : geom.position[1] < 0);
          });
          const landingGeom = specs.find(spec => {
            const geom = initialPoses.find(p => p.id === spec.id);
            return geom && (swing === "left" ? geom.position[1] > 0 : geom.position[1] < 0);
          });
          expect(supportGeom).toBeDefined();
          expect(landingGeom).toBeDefined();
          const getPose = (geomId: number): MeasuredFootPose => {
            const p = sim.geometryPoses().find(p => p.id === geomId)!;
            return { centerWorldM: p.position, rotationWorld: p.rotation };
          };
          let lastSupport = getPose(supportGeom!.id);
          let lastLanding = getPose(landingGeom!.id);
          let lastTimeS = sim.state().time;
          let sampledForwardImpulseNs = 0;
          let sampledBrakingImpulseNs = 0;
          let supportMaterialPathM = 0;
          let landingMaterialPathM = 0;
          let requestedPushImpulseNs = 0;
          let loadedSupportSamples = 0;
          let loadedLandingSamples = 0;
          let activeControllerSamples = 0;
          let minSupportLocalXM = Infinity, maxSupportLocalXM = -Infinity;
          let maxActualStanceAnklePitchRad = -Infinity;

          for (let i = 0; i < 260; i++) {
            const state = sim.step(10);
            const fsm = sim.experimentalStepStatus()!;
            const ground = sim.humanoidGroundSupportObservation();
            const rear = ground[support];
            const front = ground[swing];
            const currentSupport = getPose(supportGeom!.id);
            const currentLanding = getPose(landingGeom!.id);
            const dt = state.time - lastTimeS;
            maxActualStanceAnklePitchRad=Math.max(maxActualStanceAnklePitchRad,
              sim.jointAngleRad(support+"_ankle_pitch"));
            if (dt > 0 && rear.verticalLoadN > 40 &&
                rear.verticalLoadCentroidWorldM &&
                ["LIFT", "SWING", "LAND"].includes(fsm.phase)) {
              const reactionX = rear.tangentialReactionXYN[0];
              sampledForwardImpulseNs += Math.max(0, reactionX) * dt;
              sampledBrakingImpulseNs += Math.max(0, -reactionX) * dt;
              supportMaterialPathM += estimateContactPatchSlip(lastSupport,
                currentSupport, rear.verticalLoadCentroidWorldM, dt)
                .estimatedSlipDistanceThisSampleM;
              loadedSupportSamples++;
              const localX=materialContactLocalM(currentSupport,
                rear.verticalLoadCentroidWorldM)[0];
              minSupportLocalXM=Math.min(minSupportLocalXM,localX);
              maxSupportLocalXM=Math.max(maxSupportLocalXM,localX);
              const controller = sim.experimentalStanceContactStatus();
              if (controller) {
                activeControllerSamples++;
                requestedPushImpulseNs += controller.intendedPushOffN * dt;
              }
            }
            if (dt > 0 && fsm.stepContactConfirmed && front.verticalLoadN > 40 &&
                front.verticalLoadCentroidWorldM) {
              landingMaterialPathM += estimateContactPatchSlip(lastLanding,
                currentLanding, front.verticalLoadCentroidWorldM, dt)
                .estimatedSlipDistanceThisSampleM;
              loadedLandingSamples++;
            }
            lastSupport = currentSupport;
            lastLanding = currentLanding;
            lastTimeS = state.time;
          }

          const status = sim.experimentalStepStatus()!;
          const row = {
            swing, stanceGain, mode, toeRollStrength, phase: status.phase,
            maxActualStanceAnklePitchDeg:+(maxActualStanceAnklePitchRad*180/Math.PI).toFixed(3),
            sampledForwardImpulseNs: +sampledForwardImpulseNs.toFixed(4),
            sampledBrakingImpulseNs: +sampledBrakingImpulseNs.toFixed(4),
            requestedPushImpulseNs: +requestedPushImpulseNs.toFixed(4),
            supportMaterialPathCm: +(supportMaterialPathM * 100).toFixed(3),
            landingMaterialPathCm: +(landingMaterialPathM * 100).toFixed(3),
            supportSiteDriftCm: +(status.maxSupportFootSlipM * 100).toFixed(3),
            landingSiteDriftCm: +(status.maxLandingSlipM * 100).toFixed(3),
            targetErrorCm: +(status.targetErrorXYM * 100).toFixed(3),
            settledS: +status.settledDurationS.toFixed(3),
            loadedSupportSamples, loadedLandingSamples, activeControllerSamples,
            maxRearContactLocalXCm:loadedSupportSamples>0 ?
              +(maxSupportLocalXM*100).toFixed(3) : null,
            rearContactMigrationCm:loadedSupportSamples>0 ?
              +((maxSupportLocalXM-minSupportLocalXM)*100).toFixed(3) : null,
          };
          rows.push(row);
          expect([row.sampledForwardImpulseNs, row.sampledBrakingImpulseNs,
            row.supportMaterialPathCm, row.landingMaterialPathCm,
            row.targetErrorCm, row.settledS].every(Number.isFinite)).toBe(true);
          expect(loadedSupportSamples).toBeGreaterThan(0);
          if (stanceGain === 0) {
            expect(activeControllerSamples).toBe(0);
          }
          if (stanceGain===0 && toeRollStrength===0) {
            expect(status.phase).toBe("COMPLETE");
            expect(status.settledDurationS).toBeGreaterThanOrEqual(1);
          }
          if (status.phase === "COMPLETE") {
            expect(status.stepContactConfirmed).toBe(true);
            expect(status.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(status.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(status.targetErrorXYM).toBeLessThanOrEqual(0.05);
          }
        } finally {
          sim.dispose();
        }
      }
    }
    console.log("STANCE-CONTACT-ABLATION " + JSON.stringify(rows));
    expect(rows).toHaveLength(18);
  }, 120000);
});
