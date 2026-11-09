import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import {
  CYLINDER, DEFAULT_CYLINDER_TRIAL, cylinderForceAt,
  cylinderForwardTiltDeg, cylinderMJCF, idealPivotPrediction,
  localGripPoint, validateCylinderTrial, type CylinderTrial,
} from "../../src/physics/cylinderBenchmark";
import { renderableGeomType } from "../../src/physics/geometry";

describe("Pre-seoi rigid cylinder and obstacle", () => {
  it("uses the recovered SI dimensions and exposes a 3D cylinder primitive", () => {
    expect(CYLINDER.massKg).toBe(75);
    expect(CYLINDER.heightM).toBe(1.75);
    expect(CYLINDER.radiusM).toBe(0.18);
    expect(renderableGeomType(5)).toBe("cylinder");
    expect(localGripPoint()).toEqual([0.18, 0, 0.5249999999999999]);
  });

  it("validates trial parameters and applies a pulse only during the chosen interval", () => {
    const trial = { ...DEFAULT_CYLINDER_TRIAL, horizontalForceN: 600, upwardRatio: 0.3, durationS: 0.15 };
    expect(cylinderForceAt(0.299, trial)).toEqual([0, 0, 0]);
    expect(cylinderForceAt(0.30, trial)).toEqual([600, 0, 180]);
    expect(cylinderForceAt(0.449, trial)).toEqual([600, 0, 180]);
    expect(cylinderForceAt(0.451, trial)).toEqual([0, 0, 0]);
    expect(() => validateCylinderTrial({ ...trial, obstacleHeightM: -1 })).toThrow(RangeError);
    expect(() => validateCylinderTrial({ ...trial, horizontalForceN: Infinity })).toThrow(RangeError);
    expect(() => validateCylinderTrial({ ...trial, durationS: 2 })).toThrow(RangeError);
  });

  it("keeps the theoretical pivot model separate from simulated contacts", () => {
    const highImpulse = { ...DEFAULT_CYLINDER_TRIAL, horizontalForceN: 1500 };
    const lowObstacle = idealPivotPrediction({ ...highImpulse, obstacleHeightM: 0.8 });
    const tallObstacle = idealPivotPrediction({ ...highImpulse, obstacleHeightM: 1.2 });
    expect(lowObstacle.initialKineticEnergyJ).toBeGreaterThan(lowObstacle.potentialBarrierJ);
    expect(lowObstacle.tips).toBe(true);
    expect(tallObstacle.tips).toBe(false);
    expect(idealPivotPrediction({ ...highImpulse, horizontalForceN: 0 }).tips).toBe(false);
    expect(tallObstacle.initialKineticEnergyJ).toBeLessThan(tallObstacle.potentialBarrierJ);
  });

  it("computes signed forward tilt from the freejoint quaternion", () => {
    expect(cylinderForwardTiltDeg([0, 0, 1, 1, 0, 0, 0])).toBeCloseTo(0, 8);
    const rad = Math.PI / 4;
    expect(cylinderForwardTiltDeg([0, 0, 1, Math.cos(rad / 2), 0, Math.sin(rad / 2), 0])).toBeCloseTo(45, 8);
  });

  it("loads every historical obstacle height in the real WASM physics engine", async () => {
    const mj = await loadMujoco();
    for (const height of [0.10, 0.50, 0.80, 0.90, 1.20]) {
      const model = mj.MjModel.from_xml_string(cylinderMJCF(height));
      const data = new mj.MjData(model);
      try {
        mj.mj_forward(model, data);
        expect(model.nbody).toBe(2);
        expect(model.ngeom).toBe(3);
        expect(model.nu).toBe(0);
        expect(Array.from(model.body_mass as Float64Array).reduce((sum, mass) => sum + mass, 0)).toBeCloseTo(75, 6);
        expect(data.qpos[2]).toBeCloseTo(CYLINDER.heightM / 2 + 0.001, 6);
        expect(Array.from(data.geom_xmat).every(Number.isFinite)).toBe(true);
        expect(Array.from(model.geom_type)).toContain(5);
      } finally {
        data.delete();
        model.delete();
      }
    }
  }, 30000);

  it("responds differently with and without a physical off-centre pull", async () => {
    const mj = await loadMujoco();
    const trial: CylinderTrial = {
      obstacleHeightM: 0.1, horizontalForceN: 600, upwardRatio: 0.3, durationS: 0.15,
    };
    const model = mj.MjModel.from_xml_string(cylinderMJCF(trial.obstacleHeightM));
    const passive = new mj.MjData(model);
    const pulled = new mj.MjData(model);
    try {
      mj.mj_forward(model, passive);
      mj.mj_forward(model, pulled);
      const offset = localGripPoint();
      let appliedSteps = 0;
      for (let i = 0; i < 800; i++) {
        pulled.xfrc_applied.fill(0);
        const f = cylinderForceAt(pulled.time, trial);
        if (f[0] > 0) {
          const b = 1;
          const m = pulled.xmat;
          const x = m[b * 9]! * offset[0] + m[b * 9 + 2]! * offset[2];
          const z = m[b * 9 + 6]! * offset[0] + m[b * 9 + 8]! * offset[2];
          pulled.xfrc_applied[6] = f[0];
          pulled.xfrc_applied[8] = f[2];
          pulled.xfrc_applied[10] = z * f[0] - x * f[2];
          appliedSteps++;
        }
        mj.mj_step(model, passive);
        mj.mj_step(model, pulled);
      }
      expect(appliedSteps).toBeGreaterThanOrEqual(149);
      expect(appliedSteps).toBeLessThanOrEqual(151);
      expect(pulled.time).toBeCloseTo(0.8, 6);
      expect(Math.abs(pulled.qpos[0]! - passive.qpos[0]!)).toBeGreaterThan(0.01);
      expect(Array.from(pulled.qpos).every(Number.isFinite)).toBe(true);
      expect(Array.from(pulled.qvel).every(Number.isFinite)).toBe(true);
    } finally {
      passive.delete();
      pulled.delete();
      model.delete();
    }
  }, 30000);
});
