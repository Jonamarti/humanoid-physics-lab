import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import {
  expectedPushImpulseNs, pushForceAtTime, rootTiltDegrees,
  type PushDirection, type PushTrial,
} from "../../src/physics/standingPush";

const DIRECTIONS: readonly PushDirection[] = ["forward", "backward", "left", "right"];
const PEAKS_N = [20, 60, 120, 240] as const;
const SAMPLE_STEPS = 20;
const HORIZON_STEPS = 3000; // 6s at 0.002s, push begins at 1s.

interface SweepRow {
  readonly direction: PushDirection;
  readonly peakN: number;
  readonly impulseNs: number;
  readonly maxTiltDeg: number;
  readonly minimumPelvisM: number;
  readonly maximumComDisplacementM: number;
  readonly finalComOffsetM: number;
  readonly finalComSpeedMps: number;
  readonly footDriftM: number;
  readonly finalPelvisM: number;
  /** All physical contacts, regardless of geometry; NOT sole pressure. */
  readonly samplesWithAtLeastTwoContacts: number;
  readonly recovered: boolean;
}

function horizontalDistance(a: readonly number[], b: readonly number[]): number {
  return Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!);
}

describe("Exploratory symmetric pushes on an unanchored standing humanoid", () => {
  it("reports actual 6-second recovery outcomes for multiple push strengths and directions", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const rows: SweepRow[] = [];
    for (const direction of DIRECTIONS) {
      for (const peakForceN of PEAKS_N) {
        const sim = await MujocoSimulation.create(xml);
        const trial: PushTrial = { direction, peakForceN, durationS: 0.12, startS: 1.0 };
        try {
          sim.enableExperimentalStanding();
          const initial = sim.humanoidBalanceObservation();
          const footIds = sim.geometrySpecs().filter(s => s.shape === "box" &&
            Math.abs(s.size[2] - 0.035) < 1e-6).map(s => s.id);
          expect(footIds).toHaveLength(2);
          const getFeet = (): Map<number, readonly number[]> =>
            new Map(sim.geometryPoses().filter(p => footIds.includes(p.id)).map(p => [p.id, p.position]));
          const initialFeet = getFeet();
          let minimumPelvisM = sim.state().qpos[2]!;
          let maxTiltDeg = 0;
          let maximumComDisplacementM = 0;
          let samplesWithAtLeastTwoContacts = 0;
          let settledLastSecond = true;
          for (let step = 0; step < HORIZON_STEPS; step += SAMPLE_STEPS) {
            const state = sim.step(SAMPLE_STEPS, timeS => {
              const f = pushForceAtTime(timeS, trial);
              if (f.some(component => component !== 0)) sim.applyWorldForceAtSite("sternum_impact", f);
            });
            const obs = sim.humanoidBalanceObservation();
            const tiltDeg = rootTiltDegrees(state.qpos);
            const delta = horizontalDistance(obs.comWorldM, initial.comWorldM);
            const speed = Math.hypot(obs.comVelocityWorldMps[0], obs.comVelocityWorldMps[1]);
            minimumPelvisM = Math.min(minimumPelvisM, state.qpos[2]!);
            maxTiltDeg = Math.max(maxTiltDeg, tiltDeg);
            maximumComDisplacementM = Math.max(maximumComDisplacementM, delta);
            if (obs.contactCount >= 2) samplesWithAtLeastTwoContacts++;
            if (state.time > 5 && (delta > 0.03 || speed > 0.08 || tiltDeg > 8)) {
              settledLastSecond = false;
            }
          }
          const finalState = sim.state();
          const obs = sim.humanoidBalanceObservation();
          const footDriftM = Math.max(...Array.from(getFeet(),
            ([id, point]) => horizontalDistance(point, initialFeet.get(id)!)));
          const finalComOffsetM = horizontalDistance(obs.comWorldM, initial.comWorldM);
          const finalComSpeedMps = Math.hypot(obs.comVelocityWorldMps[0], obs.comVelocityWorldMps[1]);
          const recovered = minimumPelvisM > 0.86 &&
            maxTiltDeg < 25 && footDriftM < 0.02 && settledLastSecond;
          rows.push({
            direction, peakN: peakForceN,
            impulseNs: Number(expectedPushImpulseNs(trial).toFixed(2)),
            maxTiltDeg: Number(maxTiltDeg.toFixed(2)),
            minimumPelvisM: Number(minimumPelvisM.toFixed(3)),
            maximumComDisplacementM: Number(maximumComDisplacementM.toFixed(3)),
            finalComOffsetM: Number(finalComOffsetM.toFixed(3)),
            finalComSpeedMps: Number(finalComSpeedMps.toFixed(3)),
            footDriftM: Number(footDriftM.toFixed(3)),
            finalPelvisM: Number(finalState.qpos[2]!.toFixed(3)),
            samplesWithAtLeastTwoContacts,
            recovered,
          });
          expect([finalState.time, ...finalState.qpos, ...finalState.qvel].every(Number.isFinite)).toBe(true);
          expect(finalState.time).toBeCloseTo(6, 5);
        } finally {
          sim.dispose();
        }
      }
    }
    console.log("STANDING-PUSH-SWEEP-6S " + JSON.stringify(rows));
    expect(rows).toHaveLength(DIRECTIONS.length * PEAKS_N.length);
    expect(rows.every(r => Number.isFinite(r.maxTiltDeg) && Number.isFinite(r.minimumPelvisM))).toBe(true);
    // Exploratory runs intentionally record failures; recovery targets for #11
    // must be set from evidence, never chosen to make the test green.
  }, 150000);
});
