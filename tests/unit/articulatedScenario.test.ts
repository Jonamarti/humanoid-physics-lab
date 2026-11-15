import { describe, expect, it } from "vitest";
import { validateScenario } from "../../src/scenario/validate";
import { runScenario } from "../../src/scenario/run";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { parseRecording } from "../../src/recording/read";
import { serializeRecording } from "../../src/recording/write";
const scene = () => ({ schema: "hpl.scenario.v1", id: "two_articulated", duration_s: 3, record_hz: 50, ground: { friction: 1 }, actors: [
    { id: "uke", kind: "humanoid", mass_kg: 75, height_m: 1.75, joint_limit_timeconst_s: .004 },
    { id: "tori", kind: "humanoid", mass_kg: 100, height_m: 1.85, joint_limit_timeconst_s: .004, placement: { xy_m: [.8, 0], heading_deg: 180 } },
  ], props: [], effectors: [] });
describe("escenas con dos humanoides físicos", () => {
  it("graba/verifica ambos actores, raíces libres y métricas de apoyo separadas", async () => {
    const rec = await runScenario(validateScenario(scene()));
    expect(rec.meta.humanoid_actors?.map(a => a.id)).toEqual(["uke", "tori"]);
    expect(rec.meta.tori_assist).toBeUndefined();
    expect(rec.summary.outcome).toBe("standing");
    for (const id of ["uke", "tori"]) {
      const a = rec.summary.actors![id]!;
      expect(a.outcome).toBe("standing");
      expect(a.max_root_generalized_force).toBe(0);
      expect(a.diagnostics.root_generalized_force).toBe(0);
      expect(a.max_trunk_tilt_deg).toBeLessThan(5);
      expect(a.diagnostics.actor_mass_kg).toBeCloseTo(id === "uke" ? 75 : 100, 5);
    }
    for (const sample of rec.samples) {
      expect(Object.keys(sample.actors!)).toEqual(["uke", "tori"]);
      for (const id of ["uke", "tori"]) {
        const a = sample.actors![id]!;
        expect(Object.keys(a.joint_torques_nm)).toHaveLength(28);
        expect(a.ground_reaction_n.left[2] + a.ground_reaction_n.right[2]).toBeGreaterThan(300);
      }
    }
    const roundTrip = await parseRecording(serializeRecording(rec));
    expect(roundTrip.samples[0]!.actors!.tori!.com[0]).toBeGreaterThan(.7);
  }, 30000);
  it("el efector del segundo actor usa su sitio y sus ejes iniciales; el primero queda intacto", async () => {
    const input = scene();
    input.duration_s = .25;
    input.actors[1]!.placement!.xy_m = [3, 0];
    const s = validateScenario({ ...input, effectors: [{ id: "probe", kind: "prescribed_force", actor: "tori", site: "sternum", frame: "actor_initial", force_n: [20, 0, 0], start_s: 0, duration_s: .1, waveform: "constant" }] });
    const rec = await runScenario(s);
    const e = rec.samples[0]!.effectors[0]!;
    expect(e.point_m[0]).toBeGreaterThan(2.8);
    expect(e.force_n[0]).toBeLessThan(-19.9);
    expect(Math.abs(e.force_n[1])).toBeLessThan(.01);
    expect(rec.peaks.effectors.probe!.impulse_ns[0]).toBeCloseTo(-2, 4);
    expect(rec.summary.actors!.uke!.max_trunk_tilt_deg).toBeLessThan(1);
  });
  it("la orientación inicial y un prop anterior al actor no alteran su equilibrio", async () => {
    const input=scene();
    const rec=await runScenario(validateScenario({...input,actors:[{...input.actors[0],placement:{xy_m:[1,2],heading_deg:90}}],props:[{id:"distant_prop",kind:"fixed_box",size_m:[.1,.1,.1],pos_m:[5,5,.1]}]}));
    expect(rec.summary.outcome).toBe("standing");expect(rec.summary.uke!.max_trunk_tilt_deg).toBeLessThan(1);expect(rec.summary.uke!.max_root_generalized_force).toBe(0);
  });
  it("la trayectoria completa no depende de record_hz", async () => {
    const input = scene();
    input.duration_s = .5;
    const a = await runScenario(validateScenario(input));
    const b = await runScenario(validateScenario({ ...input, record_hz: 125 }));
    expect(a.samples.at(-1)!.qpos).toEqual(b.samples.at(-1)!.qpos);
    expect(a.samples.at(-1)!.qvel).toEqual(b.samples.at(-1)!.qvel);
  });
  it("el actor relajado secundario cae sin desactivar el controlador primario", async () => {
    const input = scene();
    input.actors[1]!.placement!.xy_m = [3, 0];
    const b = input.actors[1]!;
    const rec = await runScenario(validateScenario({ ...input, actors: [input.actors[0], { ...b, attitude: "relaxed" }] }));
    expect(rec.summary.actors!.tori!.outcome).toBe("fall");
    expect(rec.summary.actors!.uke!.outcome).toBe("standing");
    expect(rec.summary.actors!.tori!.max_root_generalized_force).toBe(0);
    expect(rec.summary.artifacts?.some(a=>a.where.startsWith("tori/")&&a.type==="contact_force")).toBe(true);
    expect(rec.summary.actors!.uke!.artifacts).toEqual([]);
  }, 30000);
  it("la distancia excesiva impide el agarre y una suelta programada lo termina", async () => {
    const input = scene();
    input.duration_s = .5;
    const grip = { id: "test_grip", source: { actor: "tori", site: "right_wrist" }, target: { actor: "uke", site: "left_wrist" }, engage_s: 0, max_force_n: 20, release_s: .4 };
    const far = await runScenario(validateScenario({ ...input, actor_grips: [grip] }));
    expect(far.peaks.effectors.test_grip!.max_force_n).toBe(0);
    expect(far.peaks.effectors.test_grip!.grip_events?.some(e => e.status === "missed")).toBe(true);
    input.actors[1]!.placement = { xy_m: [0, .55], heading_deg: 0 };
    const near = validateScenario({ ...input, actors: input.actors.map(a => ({ ...a, mass_kg: 75, height_m: 1.75 })), actor_grips: [grip] });
    const a = await runScenario(near), b = await runScenario({ ...near, record_hz: 125 });
    expect(a.peaks.effectors).toEqual(b.peaks.effectors);
    expect(a.samples.at(-1)!.qpos).toEqual(b.samples.at(-1)!.qpos);
    expect(a.samples.at(-1)!.qvel).toEqual(b.samples.at(-1)!.qvel);
    expect(a.peaks.effectors.test_grip!.grip_events).toEqual([{ t_s: 0, status: "held" }, { t_s: .4, status: "released" }]);
  });
  it("rechaza escenas ambiguas y namespaces que colisionan", () => {
    const input = scene();
    expect(() => validateScenario({ ...input, tori: { mass_kg: 75 } })).toThrow(/no mezclar/);
    expect(() => validateScenario({ ...input, actors: [input.actors[0], { ...input.actors[1], placement: undefined }] })).toThrow();
    const s = validateScenario({ ...input, props: [{ id: "tori_pelvis", kind: "fixed_box", size_m: [.1, .1, .1], pos_m: [3, 0, .1] }] });
    expect(() => buildScenarioModel(s)).toThrow(/duplicado/);
  });
  it("agarra dos muñecas alcanzables y transmite la carga con reacción opuesta", async () => {
    const input = scene();
    input.duration_s = 2;
    input.actors[1]!.placement = { xy_m: [0, .55], heading_deg: 0 };
    const s = validateScenario({ ...input, actors: input.actors.map(a => ({ ...a, mass_kg: 75, height_m: 1.75 })),
      effectors: [{ id: "load", kind: "prescribed_force", actor: "tori", site: "sternum", frame: "world", force_n: [0, 100, 0], start_s: .3, duration_s: .3, waveform: "half_sine" }],
      actor_grips: [{ id: "wrist", source: { actor: "tori", site: "right_wrist" }, target: { actor: "uke", site: "left_wrist" }, engage_s: 0, max_force_n: 200 }] });
    let loadedSamples = 0;
    const rec = await runScenario(s, { onSample: (rawM, rawD) => {
        const m = rawM as {
          nbody: number;
        }, d = rawD as {
          xfrc_applied: Float64Array;
          xipos: Float64Array;
        };
        const net = [0, 0, 0], moment = [0, 0, 0];
        for (let b = 1; b < m.nbody; b++) {
          const f = Array.from(d.xfrc_applied.slice(6 * b, 6 * b + 3)), r = Array.from(d.xipos.slice(3 * b, 3 * b + 3));
          for (let k = 0; k < 3; k++)
            net[k]! += f[k]!;
          moment[0]! += d.xfrc_applied[6 * b + 3]! + r[1]! * f[2]! - r[2]! * f[1]!;
          moment[1]! += d.xfrc_applied[6 * b + 4]! + r[2]! * f[0]! - r[0]! * f[2]!;
          moment[2]! += d.xfrc_applied[6 * b + 5]! + r[0]! * f[1]! - r[1]! * f[0]!;
        }
        // Only the declared sternum load may contribute a net external wrench.
        expect(Math.abs(net[0]!)).toBeLessThan(1e-10);
        expect(Math.abs(net[2]!)).toBeLessThan(1e-10);
        const source = (rawM as {
          body: (n: string) => {
            id: number;
          };
        }).body("tori_right_forearm").id;
        const target = (rawM as {
          body: (n: string) => {
            id: number;
          };
        }).body("left_forearm").id;
        const force = d.xfrc_applied.slice(6 * target, 6 * target + 3);
        if (Math.hypot(...force) > 1)
          loadedSamples++;
        for (let k = 0; k < 3; k++)
          expect(Math.abs(force[k]! + d.xfrc_applied[6 * source + k]!)).toBeLessThan(1e-10);
        // Subtract the sternum load using its actual world point.
        const sm = rawM as {
          site: (n: string) => {
            id: number;
          };
        }, sd = rawD as {
          site_xpos: Float64Array;
        };
        const site = sm.site("tori_sternum").id;
        expect(Math.abs(moment[0]! + sd.site_xpos[3 * site + 2]! * net[1]!)).toBeLessThan(1e-10);
        expect(Math.abs(moment[1]!)).toBeLessThan(1e-10);
        expect(Math.abs(moment[2]! - sd.site_xpos[3 * site]! * net[1]!)).toBeLessThan(1e-10);
      } });
    expect(loadedSamples).toBeGreaterThan(1);
    const peak = rec.peaks.effectors.wrist!;
    console.log("REAL_GRIP", JSON.stringify({ initial: rec.samples[0]!.effectors[1], peak, actors: Object.fromEntries(Object.entries(rec.summary.actors!).map(([id, a]) => [id, { outcome: a.outcome, max_tilt: a.max_trunk_tilt_deg }])) }));
    expect(rec.samples[0]!.effectors[1]!.coupled_grip!.status).toBe("held");
    expect(peak.max_force_n).toBeGreaterThan(1);
    expect(peak.max_force_n).toBeLessThanOrEqual(200);
    expect(peak.impulse_ns[1]).toBeGreaterThan(0);
    for (const sample of rec.samples) {
      const g = sample.effectors[1]!;
      for (let k = 0; k < 3; k++)
        expect(g.force_n[k]! + g.coupled_grip!.reaction_n[k]!).toBe(0);
    }
    const unconnected = await runScenario({ ...s, actor_grips: [] });
    expect(Math.max(...rec.samples.at(-1)!.qpos.map((q, i) => Math.abs(q - unconnected.samples.at(-1)!.qpos[i]!)))).toBeGreaterThan(1e-5);
  }, 30000);
});
