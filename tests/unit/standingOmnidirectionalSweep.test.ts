import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import {
  PUSH_DIRECTIONS, expectedPushImpulseNs, pushForceAtTime,
  rootTiltDegrees, type PushDirection, type PushTrial,
} from "../../src/physics/standingPush";

/** This benchmark characterizes behavior, not universal force thresholds.
 * It deliberately records FALL and UNSTABLE as data, not test failures.
 */
type Outcome = "recovered_no_step" | "unstable" | "fall";
interface TrialResult {
  direction: PushDirection;
  peakForceN: number;
  impulseNs: number;
  outcome: Outcome;
  minimumPelvisM: number;
  maxRootTiltDeg: number;
  finalComOffsetM: number;
  finalComSpeedMps: number;
  maxFootDriftM: number;
  maxVerticalContactCount: number;
}

const SAMPLE_STEPS = 20; // 40ms; not yet a per-timestep contact record
const SIM_STEPS = 3000; // 6 seconds at dt=0.002s

describe("Physical 8-direction standing-perturbation baseline (#38)", () => {
  it("records outcome classes for all eight directions and two loads", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url),"utf8");
    const results: TrialResult[] = [];
    for (const direction of PUSH_DIRECTIONS) {
      for (const peakForceN of [120,240]) {
        const sim = await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          const base = sim.humanoidBalanceObservation();
          const feet = sim.geometrySpecs().filter(g => g.shape === "box" &&
            Math.abs(g.size[2] - 0.035) < 1e-6).map(g => g.id);
          expect(feet).toHaveLength(2);
          const footPositions = () =>
            sim.geometryPoses().filter(g => feet.includes(g.id)).map(g => g.position);
          const initialFeet = footPositions();
          const trial: PushTrial = { direction, peakForceN, startS: 1, durationS: 0.12 };
          let minPelvis = sim.state().qpos[2]!;
          let maxTilt = 0;
          let maxContacts = 0;
          let lateSettled = true;
          for (let s = 0; s < SIM_STEPS; s += SAMPLE_STEPS) {
            const state = sim.step(SAMPLE_STEPS,timeS => {
              const f = pushForceAtTime(timeS,trial);
              if (f[0] !== 0 || f[1] !== 0) sim.applyWorldForceAtSite("sternum_impact",f);
            });
            const obs = sim.humanoidBalanceObservation();
            const offset = Math.hypot(obs.comWorldM[0]-base.comWorldM[0],
              obs.comWorldM[1]-base.comWorldM[1]);
            const speed = Math.hypot(obs.comVelocityWorldMps[0],obs.comVelocityWorldMps[1]);
            const tilt = rootTiltDegrees(state.qpos);
            maxTilt = Math.max(maxTilt,tilt);
            minPelvis = Math.min(minPelvis,state.qpos[2]!);
            maxContacts = Math.max(maxContacts,obs.contactCount);
            if (state.time > 5 && (offset > 0.03 || speed > 0.08 || tilt > 8))
              lateSettled = false;
          }
          const final = sim.humanoidBalanceObservation();
          const finalFeet = footPositions();
          const footDrift = Math.max(...finalFeet.map((p,i) =>
            Math.hypot(p[0]-initialFeet[i]![0],p[1]-initialFeet[i]![1])));
          const finalOffset = Math.hypot(final.comWorldM[0]-base.comWorldM[0],
            final.comWorldM[1]-base.comWorldM[1]);
          const finalSpeed = Math.hypot(final.comVelocityWorldMps[0],final.comVelocityWorldMps[1]);
          const fell = minPelvis < 0.7 || maxTilt > 50;
          const recovered = !fell && minPelvis > 0.86 && maxTilt < 25 &&
            footDrift < 0.02 && lateSettled;
          const result: TrialResult = {
            direction, peakForceN,
            impulseNs: +expectedPushImpulseNs(trial).toFixed(3),
            outcome: fell ? "fall" : recovered ? "recovered_no_step" : "unstable",
            minimumPelvisM: +minPelvis.toFixed(4),
            maxRootTiltDeg: +maxTilt.toFixed(3),
            finalComOffsetM: +finalOffset.toFixed(4),
            finalComSpeedMps: +finalSpeed.toFixed(4),
            maxFootDriftM: +footDrift.toFixed(4),
            maxVerticalContactCount: maxContacts,
          };
          expect(Object.values(result).every(v =>
            typeof v === "string" || Number.isFinite(v))).toBe(true);
          results.push(result);
        } finally {
          sim.dispose();
        }
      }
    }
    console.log("OMNIDIRECTIONAL-STANDING-6S " + JSON.stringify(results));
    expect(results).toHaveLength(16);
    expect(new Set(results.map(r => r.direction)).size).toBe(8);
  }, 150000);
});
