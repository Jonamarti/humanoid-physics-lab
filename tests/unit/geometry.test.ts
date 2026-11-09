import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { renderableGeomType } from "../../src/physics/geometry";

describe("Primitive geometry rendering contract", () => {
  it("maps MuJoCo's stable geom enum and rejects unsupported shapes", () => {
    expect(renderableGeomType(0)).toBeUndefined(); // floor plane rendered separately
    expect(renderableGeomType(2)).toBe("sphere");
    expect(renderableGeomType(3)).toBe("capsule");
    expect(renderableGeomType(4)).toBe("ellipsoid");
    expect(renderableGeomType(6)).toBe("box");
    expect(renderableGeomType(7)).toBeUndefined(); // meshes are not in our synthetic rig
  });

  it("provides actual world transforms for the synthetic humanoid", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_v1.xml", import.meta.url), "utf8");
    const mj = await loadMujoco();
    const model = mj.MjModel.from_xml_string(xml);
    const data = new mj.MjData(model);
    try {
      mj.mj_forward(model, data);
      const types = model.geom_type as Int32Array;
      const xyz = data.geom_xpos as Float64Array;
      const rot = data.geom_xmat as Float64Array;
      expect(types.length).toBe(model.ngeom);
      expect(xyz.length).toBe(model.ngeom * 3);
      expect(rot.length).toBe(model.ngeom * 9);
      const renderable = Array.from(types).filter(t => renderableGeomType(t) !== undefined);
      expect(renderable.length).toBeGreaterThanOrEqual(14);
      expect(Array.from(xyz).every(Number.isFinite)).toBe(true);
      expect(Array.from(rot).every(Number.isFinite)).toBe(true);
      const initialHeight = xyz[2]!; // floor is geom 0, so this is zero; compare moving bodies after step
      const pelvis = Array.from(types).findIndex(t => renderableGeomType(t) !== undefined);
      const pz = pelvis * 3 + 2;
      const initialPelvisZ = xyz[pz]!;
      for (let i = 0; i < 250; i++) mj.mj_step(model, data);
      mj.mj_forward(model, data);
      expect(data.time).toBeCloseTo(0.5, 6);
      expect(xyz[pz]).not.toBe(initialPelvisZ);
      expect(initialHeight).toBeCloseTo(0, 5);
    } finally {
      data.delete();
      model.delete();
    }
  }, 30000);
});
