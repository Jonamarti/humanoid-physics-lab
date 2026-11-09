import { describe, expect, it } from "vitest";
import { buildBalanceObservation, candidateFootCornersXY, capturePointXY, convexHullXY, insideConvexHullXY, type FootGeom } from "../../src/control/balanceSensors";

const I = [1,0,0,0,1,0,0,0,1] as const;

const foot = (x: number, y: number, z = 0.035): FootGeom => ({
  centerM: [x, y, z],
  halfSizeM: [0.135, 0.055, 0.035],
  rotationWorld: I,
});

describe("Classical balance geometry (without AI)", () => {
  it("computes the LIPM capture point from COM and velocity", () => {
    expect(capturePointXY([0, 0], [0, 0], 1)).toEqual([0, 0]);
    const p = capturePointXY([0.1, -0.2], [1, 0.5], 0.981);
    expect(p[0]).toBeCloseTo(0.1 + Math.sqrt(0.1), 6);
    expect(p[1]).toBeCloseTo(-0.2 + 0.5 * Math.sqrt(0.1), 6);
    expect(() => capturePointXY([0, 0], [1, 1], 0)).toThrow(RangeError);
  });

  it("computes a convex support polygon with well-defined interior", () => {
    const hull = convexHullXY([[1,1], [0,0], [1,0], [0,1], [0.5,0.5]]);
    expect(hull).toEqual([[0,0], [1,0], [1,1], [0,1]]);
    expect(insideConvexHullXY([0.5,0.5], hull)).toBe(true);
    expect(insideConvexHullXY([1.1,0.5], hull)).toBe(false);
    expect(insideConvexHullXY([0,0], hull)).toBe(true);
    expect(insideConvexHullXY([0,0], [])).toBe(false);
  });

  it("excludes a foot when it is far above the ground", () => {
    expect(candidateFootCornersXY(foot(0, 0.105))).toHaveLength(4);
    expect(candidateFootCornersXY(foot(0, -0.105, 0.20))).toHaveLength(0);
    expect(() => candidateFootCornersXY({ ...foot(0,0), halfSizeM:[0,0,0] })).toThrow(RangeError);
  });

  it("detects COM and capture point leaving projected biped support", () => {
    const base = {
      timeS: 0, totalMassKg: 75, contactCount: 2,
      comWorldM: [0.02, 0, 0.95] as const,
      comVelocityWorldMps: [0, 0, 0] as const,
      left: foot(0.065, 0.105),
      right: foot(0.065, -0.105),
    };
    const quiet = buildBalanceObservation(base);
    expect(quiet.comProjectionInsideCandidateSupport).toBe(true);
    expect(quiet.capturePointInsideCandidateSupport).toBe(true);
    expect(quiet.candidateSupportPolygonXY).toHaveLength(4);
    const disturbed = buildBalanceObservation({ ...base,
      comVelocityWorldMps:[3, 0, 0] as const,
    });
    expect(disturbed.comProjectionInsideCandidateSupport).toBe(true);
    expect(disturbed.capturePointInsideCandidateSupport).toBe(false);
    expect(disturbed.capturePointXY[0]).toBeGreaterThan(0.5);
  });
});
