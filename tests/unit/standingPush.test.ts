import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import {
  DEFAULT_PUSH_TRIAL, expectedPushImpulseNs, pushForceAtTime,
  rootTiltDegrees, validatePushTrial, type PushTrial,
} from "../../src/physics/standingPush";

describe("Deterministic physical chest-impulse protocol", () => {
  it("has a half-sine profile with a measured impulse, including all four signs", () => {
    const base = { ...DEFAULT_PUSH_TRIAL, peakForceN: 100, startS: 1, durationS: 0.2 };
    expect(pushForceAtTime(0.99, base)).toEqual([0, 0, 0]);
    expect(pushForceAtTime(1, base)).toEqual([0, 0, 0]);
    expect(pushForceAtTime(1.1, base)[0]).toBeCloseTo(100, 8);
    expect(pushForceAtTime(1.2, base)).toEqual([0, 0, 0]);
    expect(pushForceAtTime(1.1, { ...base, direction: "backward" })[0]).toBeCloseTo(-100, 8);
    expect(pushForceAtTime(1.1, { ...base, direction: "left" })[1]).toBeCloseTo(100, 8);
    expect(pushForceAtTime(1.1, { ...base, direction: "right" })[1]).toBeCloseTo(-100, 8);
    const dt = 0.002;
    let sum = 0;
    for (let i = 0; i < 800; i++) sum += Math.abs(pushForceAtTime(i * dt, base)[0]) * dt;
    expect(sum).toBeCloseTo(expectedPushImpulseNs(base), 2);
  });

  it("rejects non-physical intervals, invalid forces and NaN", () => {
    expect(() => validatePushTrial({ ...DEFAULT_PUSH_TRIAL, direction: "wrong" as PushTrial["direction"] })).toThrow(RangeError);
    expect(() => validatePushTrial({ ...DEFAULT_PUSH_TRIAL, durationS: 0 })).toThrow(RangeError);
    expect(() => validatePushTrial({ ...DEFAULT_PUSH_TRIAL, peakForceN: -1 })).toThrow(RangeError);
    expect(() => pushForceAtTime(NaN, DEFAULT_PUSH_TRIAL)).toThrow(RangeError);
    expect(rootTiltDegrees([0,0,0,1,0,0,0])).toBeCloseTo(0, 8);
  });

  it("applies real chest wrench without actuating pelvis root and changes the trajectory", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const pushed = await MujocoSimulation.create(xml);
    const quiet = await MujocoSimulation.create(xml);
    try {
      pushed.enableExperimentalStanding();
      quiet.enableExperimentalStanding();
      const trial = { ...DEFAULT_PUSH_TRIAL, peakForceN: 80, direction: "left" as const };
      let activeSamples = 0;
      for (let i = 0; i < 1000; i += 20) {
        const forced = pushed.step(20, timeS => {
          const f = pushForceAtTime(timeS, trial);
          if (f.some(component => component !== 0)) {
            pushed.applyWorldForceAtSite("sternum_impact", f);
            activeSamples++;
          }
        });
        quiet.step(20);
        expect(forced.qpos.every(Number.isFinite)).toBe(true);
      }
      expect(activeSamples).toBeGreaterThan(50);
      const pa = pushed.humanoidBalanceObservation();
      const pb = quiet.humanoidBalanceObservation();
      expect(Math.abs(pa.comWorldM[1] - pb.comWorldM[1])).toBeGreaterThan(0.000001);
      expect(pa.timeS).toBeCloseTo(2.0, 6);
    } finally {
      pushed.dispose();
      quiet.dispose();
    }
  }, 60000);
});
