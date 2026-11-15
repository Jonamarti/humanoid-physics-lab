import { describe, expect, it } from "vitest";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { validateScenario } from "../../src/scenario/validate";
import { runScenario } from "../../src/scenario/run";
import { readFileSync } from "node:fs";
import { FallReflex, type FallModel, type FallObservation } from "../../src/uke/fallReflex";
import type { StandingState } from "../../src/control/standingBaseline";
import type { HumanoidActor } from "../../src/scenario/schema";
import { UKE_HINGE_JOINTS } from "../../src/uke/stances";
const load = (id: string) => validateScenario(JSON.parse(readFileSync("scenarios/fall_refs/" + id + ".json", "utf8")));
const observation = (t: number, limb = false, body = false): FallObservation => ({ t, dt: .002, capture_margin_m: -.2, com_velocity_mps: [-1, 0, -.5], tilt_deg: 65, heading_rad: 0, nonfoot_ground_contact: limb, torso_ground_contact: body });
describe("protección de caída: tono, apoyo y retirada de esfuerzo", () => {
  it("no extiende una postura ya plegada ni modifica amortiguación antes de activarse", async () => {
    const mj = await getMujoco(), m = mj.MjModel.from_xml_string(buildScenarioModel(load("passive_neutral")).mjcf), d = new mj.MjData(m);
    try {
      const q = (n: string) => m.jnt_qposadr[m.jnt(n).id]!, v = (n: string) => m.jnt_dofadr[m.jnt(n).id]!;
      for (const side of ["left", "right"]) {
        d.qpos[q(side + "_knee")] = -2.3;
        d.qpos[q(side + "_hip_pitch")] = -1.5;
      }
      d.qpos[q("lumbar_pitch")] = .9;
      const original = Array.from(m.dof_damping as Float64Array), r = new FallReflex(load("passive_neutral").actors[0] as HumanoidActor, m as unknown as FallModel);
      r.step(d as unknown as StandingState, observation(0));
      expect(Array.from(m.dof_damping)).toEqual(original);
      for (let i = 1; i <= 250; i++) {
        d.qfrc_applied.fill(0);
        r.step(d as unknown as StandingState, observation(i * .002));
      }
      for (const side of ["left", "right"]) {
        expect(d.qfrc_applied[v(side + "_knee")]!).toBeLessThanOrEqual(1e-10);
        expect(d.qfrc_applied[v(side + "_hip_pitch")]!).toBeLessThanOrEqual(1e-10);
      }
      expect(d.qfrc_applied[v("lumbar_pitch")]!).toBeGreaterThanOrEqual(-1e-10);
      expect(m.dof_damping[v("left_knee")]!).toBeGreaterThan(original[v("left_knee")]!);
      expect(Array.from(d.qfrc_applied as Float64Array).slice(0, 6)).toEqual([0, 0, 0, 0, 0, 0]);
    }
    finally {
      d.delete();
      m.delete();
    }
  });
  it("distingue un apoyo de miembros de la llegada del tronco al suelo", async () => {
    const mj = await getMujoco(), m = mj.MjModel.from_xml_string(buildScenarioModel(load("passive_neutral")).mjcf), d = new mj.MjData(m);
    try {
      const r = new FallReflex(load("passive_neutral").actors[0] as HumanoidActor, m as unknown as FallModel);
      for (let i = 0; i <= 300; i++) {
        d.qfrc_applied.fill(0);
        r.step(d as unknown as StandingState, observation(i * .002, i > 150, false));
      }
      expect(r.status().phase).toBe("brace");
      expect(Math.max(...Array.from(d.qfrc_applied as Float64Array).map(Math.abs))).toBeGreaterThan(1);
      for (let i = 301; i <= 450; i++) {
        d.qfrc_applied.fill(10);
        d.qfrc_applied.fill(0, 0, 6);
        r.step(d as unknown as StandingState, observation(i * .002, true, true));
      }
      expect(r.status().phase).toBe("settle");
      for (const n of UKE_HINGE_JOINTS)
        expect(d.qfrc_applied[m.jnt_dofadr[m.jnt(n).id]!]!).toBe(0);
    }
    finally {
      d.delete();
      m.delete();
    }
  });
  for (const id of ["passive_neutral", "passive_heavy"])
    it(id + ": elimina la amplificación grave de impacto con física real", async () => {
      const r = await runScenario(load(id));
      expect(r.summary.outcome).toBe("fall");
      expect(r.summary.uke!.fall!.impact.head).toBeNull();
      expect(r.diagnostics!.contact_force.value).toBeLessThan(10000);
      expect(r.diagnostics!.ground_penetration.value).toBeLessThan(.01);
      expect(r.diagnostics!.joint_speed!.value).toBeLessThan(25);
      expect(r.diagnostics!.root_generalized_force).toBe(0);
      const status = r.summary.uke!.controller.fall_response as {
        settle_s: number;
      };
      let settledSamples = 0;
      for (const sample of r.samples)
        if (sample.t > status.settle_s + .152 && sample.joint_torques_nm) {
          for (const tau of Object.values(sample.joint_torques_nm))
            expect(tau).toBe(0);
          settledSamples++;
        }
      expect(settledSamples).toBeGreaterThan(50);
      expect(r.summary.uke!.balance!.final_com_speed_mps).toBeLessThan(.02);
    }, 30000);
  it("el empujón frontal conserva su apoyo de brazos y no añade impacto de cabeza", async () => {
    const r = await runScenario(load("push_front"));
    expect(r.diagnostics!.contact_force.value).toBeLessThan(10000);
    expect(r.summary.uke!.fall!.impact.head).toBeNull();
    expect(r.summary.uke!.controller.fall_response).toMatchObject({ phase: "brace", settle_s: null });
  }, 30000);
  it("la protección del actor con prefijo queda aislada del humanoide estable", async () => {
    const raw = JSON.parse(readFileSync("scenarios/articulated_refs/secondary_passive.json", "utf8"));
    raw.record_hz = 50;
    raw.actors[1].fall = "instinctive";
    const r = await runScenario(validateScenario(raw));
    expect(r.summary.actors!.uke!.outcome).toBe("standing");
    expect(r.summary.actors!.tori!.outcome).toBe("fall");
    expect(r.summary.actors!.tori!.fall!.impact.head).toBeNull();
    expect(r.summary.actors!.tori!.diagnostics.contact_force.value).toBeLessThan(10000);
    expect(r.summary.actors!.tori!.max_root_generalized_force).toBe(0);
    expect(r.summary.actors!.uke!.max_root_generalized_force).toBe(0);
  }, 30000);
  it("la respuesta corregida conserva la independencia de la frecuencia de grabación", async () => {
    const s = load("passive_neutral"), a = await runScenario({ ...s, record_hz: 50 }), b = await runScenario({ ...s, record_hz: 125 });
    expect(a.samples.at(-1)!.qpos).toEqual(b.samples.at(-1)!.qpos);
    expect(a.samples.at(-1)!.qvel).toEqual(b.samples.at(-1)!.qvel);
  }, 30000);
});
