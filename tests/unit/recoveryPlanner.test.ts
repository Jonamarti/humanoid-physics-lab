import { describe, expect, it } from "vitest";
import { summarizeGroundSupport, type GroundContactSample } from "../../src/control/footContacts";
import { planRecovery } from "../../src/control/recoveryPlanner";
import { type FootGeom } from "../../src/control/balanceSensors";

const I = [1,0,0,0,1,0,0,0,1] as const;
const foot = (y: number): FootGeom => ({
  centerM: [0.065, y, 0.035],
  rotationWorld: I,
  halfSizeM: [0.135, 0.055, 0.035],
});
const floor = 0;
const left = 11;
const right = 12;
const contact = (id: number, forceN: number, y: number): GroundContactSample => ({
  geom1: floor,
  geom2: id,
  positionWorldM: [0.065, y, 0],
  contactNormalWorld: [0, 0, 1],
  contactNormalForceN: forceN,
});
const makeGround = (leftN = 380, rightN = 380) =>
  summarizeGroundSupport({
    floorGeomId: floor,
    leftFootGeomId: left,
    rightFootGeomId: right,
    leftFoot: foot(0.105), rightFoot: foot(-0.105),
    contacts: [contact(left,leftN,0.105),contact(right,rightN,-0.105)],
  });
const input = (vx = 0, vy = 0) => ({
  comXY: [0.025, 0] as const,
  comVelocityXY: [vx,vy] as const,
  comHeightM: 0.95,
  excessiveTrunkTilt: false,
  unstableDurationS: 0.3,
  ground: makeGround(),
});

describe("Ground-contact support summary (pure logic, physics adapter pending)", () => {
  it("excludes airborne/foreign contacts and uses vertical force, not contact count", () => {
    const g = summarizeGroundSupport({
      floorGeomId: floor, leftFootGeomId: left, rightFootGeomId: right,
      leftFoot: foot(0.105), rightFoot: foot(-0.105),
      contacts: [
        contact(left, 500, 0.105),
        { ...contact(right, 500, -0.105), geom1: 4 }, // non-floor collision
        { ...contact(right, 400, -0.105), contactNormalWorld: [1,0,0] },
      ],
    });
    expect(g.left.isLoaded).toBe(true);
    expect(g.right.isLoaded).toBe(false);
    expect(g.left.verticalLoadN).toBe(500);
    expect(g.right.verticalLoadN).toBe(0);
    expect(g.loadedSupportPolygonXY).toHaveLength(4);
    expect(g.left.verticalLoadCentroidXY).toEqual([0.065,0.105]);
  });

  it("handles reversed geom order and rejects invalid physical values", () => {
    const g = summarizeGroundSupport({
      floorGeomId: floor, leftFootGeomId: left, rightFootGeomId: right,
      leftFoot: foot(0.105), rightFoot: foot(-0.105),
      contacts: [{ ...contact(left, 390,0.105),
        geom1: left, geom2: floor, contactNormalWorld: [0,0,-1] }],
    });
    expect(g.left.verticalLoadN).toBe(390);
    expect(g.right.isLoaded).toBe(false);
    expect(() => makeGround(NaN)).toThrow(RangeError);
    expect(() => makeGround(-10)).toThrow(RangeError);
  });
});

describe("Pure recovery planner: proposals, not simulated steps", () => {
  it("stays quiet with two feet loaded and no disturbance", () => {
    const r = planRecovery(input());
    expect(r.decision).toBe("NONE");
    expect(r.swingFoot).toBeNull();
  });

  it("waits for delay before attempting an unstable correction", () => {
    const r = planRecovery({ ...input(0.9), unstableDurationS: 0.02 });
    expect(r.decision).toBe("SHIFT_WEIGHT");
  });

  it("proposes the lighter foot only when the support foot can bear COM", () => {
    const r = planRecovery({
      ...input(0.75,0.35),
      comXY: [0.025, -0.105],
      ground: makeGround(150,600),
      maxStepReachM: 0.5,
    });
    expect(r.decision).toBe("STEP");
    expect(r.swingFoot).toBe("left");
    expect(r.supportFoot).toBe("right");
    expect(r.targetFootXY).not.toBeNull();
    expect(Math.hypot(r.targetFootXY![0] - 0.065,
      r.targetFootXY![1] - 0.105)).toBeLessThanOrEqual(0.5);
  });

  it("refuses impossible far capture, missing support, and excessive rotation", () => {
    expect(planRecovery(input(6,0)).decision).toBe("FALL");
    expect(planRecovery({ ...input(0.8), ground: makeGround(0,0) }).decision).toBe("FALL");
    expect(planRecovery({ ...input(), excessiveTrunkTilt: true }).decision).toBe("FALL");
  });

  it("respects symmetry and validates non-finite input", () => {
    expect(planRecovery(input(0,0.1)).capturePointXY[1]).toBeGreaterThan(0);
    expect(planRecovery(input(0,-0.1)).capturePointXY[1]).toBeLessThan(0);
    expect(() => planRecovery({ ...input(), comHeightM: NaN })).toThrow(RangeError);
  });
});
