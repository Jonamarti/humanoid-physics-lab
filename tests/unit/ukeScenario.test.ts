import { describe, expect, it } from "vitest";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { classifyHumanoid } from "../../src/metrics/outcome";
import { ScenarioValidationError, validateScenario } from "../../src/scenario/validate";
import { runScenario } from "../../src/scenario/run";

const base = (actor: Record<string, unknown>, extra: Record<string, unknown> = {}): Record<string, unknown> => ({
  schema: "hpl.scenario.v1", id: "uke_test", duration_s: 1, record_hz: 50, ground: { friction: 1 },
  actors: [{ id: "uke", kind: "humanoid", mass_kg: 110, height_m: 1.92, ...actor }],
  props: [], effectors: [], ...extra,
});

function message(json: unknown): string {
  try {
    validateScenario(json);
  } catch (e) {
    expect(e).toBeInstanceOf(ScenarioValidationError);
    return (e as Error).message;
  }
  throw new Error("se esperaba un error de validación");
}

describe("actor humanoide en el esquema de escenario", () => {
  it("acepta el actor mínimo y rellena strength = 1, attitude = tense y stance = neutral", () => {
    const s = validateScenario(base({}));
    expect(s.actors[0]).toEqual({
      id: "uke", kind: "humanoid", mass_kg: 110, height_m: 1.92, strength: 1, attitude: "tense", stance: "neutral",
    });
  });

  it("acepta todos los campos opcionales", () => {
    const s = validateScenario(base({
      strength: 0.5, build: 0.4, segment_lengths_m: { thigh: 0.5, neck: 0.07 }, attitude: "tense", stance: "staggered_left",
    }));
    const a = s.actors[0]!;
    expect(a.kind === "humanoid" && a.segment_lengths_m).toEqual({ thigh: 0.5, neck: 0.07 });
    expect(a.kind === "humanoid" && a.stance).toBe("staggered_left");
  });

  it("rechaza valores fuera de rango, claves desconocidas y enumeraciones inválidas", () => {
    expect(message(base({ mass_kg: 40 }))).toContain("actors[0].mass_kg");
    expect(message(base({ height_m: 2.5 }))).toContain("actors[0].height_m");
    expect(message(base({ strength: 1.6 }))).toContain("actors[0].strength");
    expect(message(base({ strength: 0.4 }))).toContain("actors[0].strength");
    expect(message(base({ build: 1.5 }))).toContain("actors[0].build");
    expect(message(base({ attitude: "relaxed" }))).toContain("actors[0].attitude");
    expect(message(base({ stance: "kneeling" }))).toContain("actors[0].stance");
    expect(message(base({ radius_m: 0.2 }))).toContain("actors[0].radius_m: campo desconocido");
    expect(message(base({ fall: "instinctive" }))).toContain("actors[0].fall: campo desconocido");
    expect(message(base({ segment_lengths_m: { arm: 0.3 } }))).toContain("actors[0].segment_lengths_m.arm");
    expect(message(base({ segment_lengths_m: { thigh: 2 } }))).toContain("actors[0].segment_lengths_m.thigh");
    expect(message(base({ mass_kg: Number.NaN }))).toContain("actors[0].mass_kg");
  });

  it("los efectores pueden apuntar a los cuerpos del humanoide", () => {
    const eff = { id: "push", kind: "prescribed_force", actor: "uke", body: "chest", local_point_m: [0.1, 0, 0.1],
      frame: "world", force_n: [10, 0, 0], start_s: 0.1, duration_s: 0.1, waveform: "half_sine" };
    expect(validateScenario(base({}, { effectors: [eff] })).effectors).toHaveLength(1);
    expect(message(base({}, { effectors: [{ ...eff, body: "cylinder" }] }))).toContain("effectors[0].body");
  });
});

describe("buildScenarioModel con humanoide", () => {
  it("devuelve cuerpos y articulaciones en el orden del MJCF y respeta props", () => {
    const s = validateScenario(base({}, { props: [{ id: "wall", kind: "fixed_box", size_m: [0.1, 1, 1], pos_m: [1, 0, 0.5] }] }));
    const m = buildScenarioModel(s);
    expect(m.actorBody).toBe("pelvis");
    expect(m.bodies.slice(0, 3)).toEqual(["world", "wall", "pelvis"]);
    expect(m.joints[0]).toBe("pelvis_root");
    expect(m.joints).toContain("neck_yaw");
    expect(m.mjcf).toContain('cone="elliptic"');
    expect(() => buildScenarioModel(validateScenario(base({}, { props: [{ id: "chest", kind: "fixed_box", size_m: [1, 1, 1], pos_m: [0, 0, 0] }] }))))
      .toThrow("reservado");
  });
});

describe("veredicto mínimo del humanoide", () => {
  it("de pie: tronco < 3° y pelvis > 90 %; caído: pelvis < 50 %; si no, inclinado", () => {
    expect(classifyHumanoid(0.5, 1)).toBe("standing");
    expect(classifyHumanoid(2.9, 0.95)).toBe("standing");
    expect(classifyHumanoid(3.1, 1)).toBe("tilted");
    expect(classifyHumanoid(1, 0.89)).toBe("tilted");
    expect(classifyHumanoid(0, 0.49)).toBe("fall");
    expect(classifyHumanoid(80, 0.3)).toBe("fall");
  });
});

describe("runScenario con humanoide (MuJoCo WASM real)", () => {
  it("un empujón grande al pecho lo tumba y el veredicto lo detecta; sin fuerza en la raíz", async () => {
    const s = validateScenario(base({}, {
      duration_s: 3,
      effectors: [{ id: "push", kind: "prescribed_force", actor: "uke", body: "chest", local_point_m: [0.1, 0, 0.15],
        frame: "world", force_n: [-2500, 0, 0], start_s: 0.2, duration_s: 0.2, waveform: "half_sine" }],
    }));
    const rec = await runScenario(s, { engineVersion: "test" });
    expect(rec.summary.outcome).toBe("fall");
    expect(rec.summary.direction_deg).not.toBeNull();
    expect(rec.summary.uke!.max_root_generalized_force).toBe(0);
    expect(rec.model.bodies[1]).toBe("pelvis");
    expect(rec.samples[0]!.qpos).toHaveLength(7 + 28);
  }, 60000);

  it("es determinista", async () => {
    const s = validateScenario(base({ stance: "natural_grip" }, { duration_s: 1 }));
    const a = await runScenario(s, { engineVersion: "test" });
    const b = await runScenario(s, { engineVersion: "test" });
    expect(JSON.stringify(a)).toBe(JSON.stringify(b));
  }, 60000);
});
