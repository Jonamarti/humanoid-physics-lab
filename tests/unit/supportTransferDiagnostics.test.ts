import { describe, expect, it } from "vitest";
import {
  assessSupportTransfer, type SupportTransferInput,
} from "../../src/control/supportTransferDiagnostics";

const sample = (changes: Partial<SupportTransferInput> = {}): SupportTransferInput => ({
  supportLoadN: 545,
  swingLoadN: 191,
  comVelocityWorldMps: [0.4, 0, 0],
  comProjectionInsideLoadedSupport: true,
  capturePointInsideLoadedSupport: false,
  pelvisHeightM: 0.98,
  rootTiltDeg: 3,
  ...changes,
});

describe("v3 solved-contact pre-lift diagnostics (no changes to release gates)", () => {
  it("identifies a qualifying emergency snapshot but not a complete physical lift", () => {
    const result = assessSupportTransfer(sample(), true);
    expect(result.totalLoadN).toBe(736);
    expect(result.supportFraction).toBeCloseTo(545 / 736);
    expect(result.ready).toBe(true);
    expect(result.gates.capturePoint).toBe(true); // emergency capture lies outside
  });

  it("retains stricter voluntary thresholds and capture-region requirement", () => {
    expect(assessSupportTransfer(sample(), false).ready).toBe(false);
    const voluntary = assessSupportTransfer(sample({
      supportLoadN: 600,
      swingLoadN: 136,
      comVelocityWorldMps: [0.1, 0, 0],
      capturePointInsideLoadedSupport: true,
    }), false);
    expect(voluntary.ready).toBe(true);
  });

  it.each([
    ["supportLoad", { supportLoadN: 510, swingLoadN: 226 }],
    ["swingUnloaded", { supportLoadN: 600, swingLoadN: 236 }],
    ["comProjection", { comProjectionInsideLoadedSupport: false }],
    ["pelvisHeight", { pelvisHeightM: 0.82 }],
    ["rootTilt", { rootTiltDeg: 25 }],
    ["comSpeed", { comVelocityWorldMps: [1.2, 0, 0] }],
  ] as const)("reports blocker %s instead of claiming readiness", (gate, changed) => {
    const result = assessSupportTransfer(sample(changed), true);
    expect(result.gates[gate]).toBe(false);
    expect(result.ready).toBe(false);
  });

  it("requires real measured support even if other posture gates look good", () => {
    const unloaded = assessSupportTransfer(sample({
      supportLoadN: 0, swingLoadN: 0,
    }), true);
    expect(unloaded.supportFraction).toBe(0);
    expect(unloaded.gates.supportLoad).toBe(false);
    expect(unloaded.ready).toBe(false);
  });

  it("never lifts the overloaded swing leg when the selected stance leg has zero solved load",()=>{
    const diagnostic=assessSupportTransfer(sample({
      supportLoadN:0,swingLoadN:600,
      comProjectionInsideLoadedSupport:true,
      capturePointInsideLoadedSupport:false,
      pelvisHeightM:1.0,rootTiltDeg:4,
    }),true);
    expect(diagnostic.gates.comProjection).toBe(true);
    expect(diagnostic.gates.supportLoad).toBe(false);
    expect(diagnostic.gates.swingUnloaded).toBe(false);
    expect(diagnostic.supportFraction).toBe(0);
    expect(diagnostic.ready).toBe(false);
  });
  it("does not accidentally require old capture support for emergency readiness", () => {
    const a = assessSupportTransfer(sample({ capturePointInsideLoadedSupport: false }), true);
    const b = assessSupportTransfer(sample({ capturePointInsideLoadedSupport: true }), true);
    expect(a.ready).toBe(b.ready);
    expect(a.ready).toBe(true);
  });
});
