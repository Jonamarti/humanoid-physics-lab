import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { applyHingePD, limitedJointTorque, type JointPDSettings } from "../../src/control/jointPD";

const config: JointPDSettings = {
  targetRad: 0,
  stiffnessNmPerRad: 150,
  dampingNmsPerRad: 16,
  maxTorqueNm: 40,
};

describe("Internal-joint PD, no learned control", () => {
  it("uses signed position and velocity feedback in SI units", () => {
    expect(limitedJointTorque(0, 0, config)).toBe(0);
    expect(limitedJointTorque(0.1, 0, config)).toBeCloseTo(-15, 6);
    expect(limitedJointTorque(-0.1, 0, config)).toBeCloseTo(15, 6);
    expect(limitedJointTorque(0, 0.5, config)).toBeCloseTo(-8, 6);
  });

  it("saturates both torque directions", () => {
    expect(limitedJointTorque(2, 0, config)).toBe(-40);
    expect(limitedJointTorque(-2, 0, config)).toBe(40);
    expect(limitedJointTorque(0, 0, { ...config, maxTorqueNm: 0 })).toBe(0);
  });

  it("rejects NaNs, infinity and negative physical limits", () => {
    expect(() => limitedJointTorque(NaN, 0, config)).toThrow(RangeError);
    expect(() => limitedJointTorque(0, 0, { ...config, maxTorqueNm: -1 })).toThrow(RangeError);
    expect(() => limitedJointTorque(0, Infinity, config)).toThrow(RangeError);
  });

  it("only writes internal hinge DoFs, not the root/free pelvis", () => {
    const model = {
      njnt: 2, jnt_type: [0, 3], jnt_qposadr: [0, 7], jnt_dofadr: [0, 6],
    };
    const data = {
      qpos: [0, 0, 0, 1, 0, 0, 0, 0.1],
      qvel: [0, 0, 0, 0, 0, 0, 0.5],
      qfrc_applied: new Float64Array(7),
    };
    expect(() => applyHingePD(model, data, new Map([[0, config]]))).toThrow(TypeError);
    expect(applyHingePD(model, data, new Map([[1, config]]))).toBe(1);
    expect(Array.from(data.qfrc_applied.slice(0, 6))).toEqual([0, 0, 0, 0, 0, 0]);
    expect(data.qfrc_applied[6]).toBe(-23);
  });

  it("moves an internal hinge in the real MuJoCo humanoid without root actuation", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const mj = await loadMujoco();
    const model = mj.MjModel.from_xml_string(xml);
    const passive = new mj.MjData(model);
    const driven = new mj.MjData(model);
    try {
      mj.mj_forward(model, passive);
      mj.mj_forward(model, driven);
      const hingeIndex = 1; // root is joint 0, first internal hinge is lumbar_pitch
      expect(model.jnt_type[hingeIndex]).toBe(3);
      const q = model.jnt_qposadr[hingeIndex]!;
      const profile = new Map([[hingeIndex, {
        targetRad: 0.35,
        stiffnessNmPerRad: 120,
        dampingNmsPerRad: 14,
        maxTorqueNm: 35,
      }]]);
      for (let i = 0; i < 100; i++) {
        applyHingePD(model, driven, profile);
        mj.mj_step(model, driven);
        mj.mj_step(model, passive);
      }
      expect(Array.from(driven.qfrc_applied).slice(0, 6)).toEqual([0, 0, 0, 0, 0, 0]);
      expect(Math.abs(driven.qpos[q]! - passive.qpos[q]!)).toBeGreaterThan(0.001);
      expect(Array.from(driven.qpos).every(Number.isFinite)).toBe(true);
      expect(Array.from(driven.qvel).every(Number.isFinite)).toBe(true);
    } finally {
      driven.delete();
      passive.delete();
      model.delete();
    }
  }, 30000);
});
