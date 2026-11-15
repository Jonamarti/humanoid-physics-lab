import { describe, expect, it } from "vitest";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { generateHumanoidMJCF } from "../../src/uke/generate";
import { appendArticulatedHumanoid } from "../../src/uke/actorModel";
import { namespaceHumanoidXML } from "../../src/uke/modelNamespace";
import { applyStance } from "../../src/uke/stances";
import { createUkeController } from "../../src/uke/attitudes";
import { attitudeJointDamping } from "../../src/uke/damping";
import type { StandingModel, StandingState } from "../../src/control/standingBaseline";
import type { HumanoidActor } from "../../src/scenario/schema";
const actor = (mass = 75): HumanoidActor => ({ id: "actor", kind: "humanoid", mass_kg: mass, height_m: 1.75, strength: 1, attitude: "tense", stance: "neutral", joint_limit_timeconst_s: .004 });
function single(a: HumanoidActor): string { return generateHumanoidMJCF(a, { jointDampingByName: attitudeJointDamping(a.attitude, a.mass_kg, a.height_m, a.strength), jointLimitTimeconstS: .004 }); }
describe("humanoides articulados independientes", () => {
  it("el prefijo conserva suelo/props y cambia todas las referencias del actor", async () => {
    const xml = generateHumanoidMJCF(actor(), { namePrefix: "tori_", extraWorldXml: '<body name="prop" pos="4 0 1"><geom name="prop_geom" type="sphere" size=".1"/></body>' });
    expect(xml).toContain('name="floor"');
    expect(xml).toContain('name="prop"');
    expect(xml).toContain('name="tori_pelvis_root"');
    expect(xml).toContain('body1="tori_abdomen" body2="tori_left_thigh"');
    expect(() => namespaceHumanoidXML(xml, "../bad")).toThrow();
    const mj = await getMujoco();
    const m = mj.MjModel.from_xml_string(xml);
    try {
      expect(m.jnt("tori_left_knee").id).toBeGreaterThan(0);
      expect(m.nmocap).toBe(0);
    }
    finally {
      m.delete();
    }
  });
  it("dos raíces libres, límites, masas separadas y control que no borra al otro actor", async () => {
    const mj = await getMujoco();
    const xml = appendArticulatedHumanoid(single(actor()), actor(100), "tori_", { xy_m: [.6, 0], heading_deg: 180 }, 1);
    const m = mj.MjModel.from_xml_string(xml);
    const d = new mj.MjData(m);
    try {
      const targetsA = applyStance(mj, m, d, "neutral").targets;
      const targetsB = applyStance(mj, m, d, "neutral", "tori_").targets;
      mj.mj_forward(m, d);
      mj.mj_subtreeVel(m, d);
      expect(m.nmocap).toBe(0);
      expect(Array.from(m.jnt_type as Int32Array).filter(x => x === 0)).toHaveLength(2);
      const a = createUkeController({ ...actor(), targets: targetsA, scope_to_actor: true }, m as unknown as StandingModel, d as unknown as StandingState);
      const b = createUkeController({ ...actor(100), targets: targetsB, prefix: "tori_", scope_to_actor: true }, m as unknown as StandingModel, d as unknown as StandingState);
      d.qfrc_applied.fill(0);
      a.step(m as unknown as StandingModel, d as unknown as StandingState, 0);
      const dof = m.jnt_dofadr[m.jnt("left_knee").id]!;
      d.qfrc_applied[dof] = 123;
      b.step(m as unknown as StandingModel, d as unknown as StandingState, 0);
      expect(d.qfrc_applied[dof]).toBe(123);
      const p = m.body("tori_pelvis").id;
      expect(d.subtree_com[3 * p]!).toBeGreaterThan(.5);
    }
    finally {
      d.delete();
      m.delete();
    }
  });
  it("dos humanoides frente a frente a 60 cm aguantan diez segundos con ambas pelvis libres", async () => {
    const mj = await getMujoco();
    const aSpec = actor(), bSpec = actor();
    const m = mj.MjModel.from_xml_string(appendArticulatedHumanoid(single(aSpec), bSpec, "tori_", { xy_m: [.6, 0], heading_deg: 180 }, 1));
    const d = new mj.MjData(m);
    try {
      const ta = applyStance(mj, m, d, "neutral").targets;
      const tb = applyStance(mj, m, d, "neutral", "tori_").targets;
      mj.mj_forward(m, d);
      mj.mj_subtreeVel(m, d);
      const controls = [createUkeController({ ...aSpec, targets: ta, scope_to_actor: true }, m as unknown as StandingModel, d as unknown as StandingState), createUkeController({ ...bSpec, targets: tb, prefix: "tori_", scope_to_actor: true }, m as unknown as StandingModel, d as unknown as StandingState)];
      const roots = [m.body("pelvis").id, m.body("tori_pelvis").id];
      const z0 = roots.map(b => d.xpos[3 * b + 2]!);
      const min = z0.slice();
      const rootDofs = [m.jnt_dofadr[m.jnt("pelvis_root").id]!, m.jnt_dofadr[m.jnt("tori_pelvis_root").id]!];
      let maxTilt = 0;
      const started = performance.now();
      for (let i = 0; i < 5000; i++) {
        mj.mj_forward(m, d);
        mj.mj_subtreeVel(m, d);
        d.qfrc_applied.fill(0);
        for (const c of controls)
          c.step(m as unknown as StandingModel, d as unknown as StandingState, i * .002);
        for (const v of rootDofs)
          for (let k = 0; k < 6; k++)
            expect(d.qfrc_applied[v + k]).toBe(0);
        mj.mj_step(m, d);
        for (let j = 0; j < roots.length; j++) {
          const b = roots[j]!;
          min[j] = Math.min(min[j]!, d.xpos[3 * b + 2]!);
          maxTilt = Math.max(maxTilt, Math.atan2(Math.hypot(d.xmat[9 * b + 2]!, d.xmat[9 * b + 5]!), d.xmat[9 * b + 8]!) * 180 / Math.PI);
        }
      }
      expect(min[0]!).toBeGreaterThan(.9 * z0[0]!);
      expect(min[1]!).toBeGreaterThan(.9 * z0[1]!);
      expect(maxTilt).toBeLessThan(5);
      expect(Array.from(d.qpos).every(Number.isFinite)).toBe(true);
      console.log("TWO_ACTORS_10S", JSON.stringify({ min_pelvis_m: min, max_root_tilt_deg: maxTilt, elapsed_ms: performance.now() - started, controls: controls.map(c => c.status()) }));
    }
    finally {
      d.delete();
      m.delete();
    }
  }, 30000);
});
