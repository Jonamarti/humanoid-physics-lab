import { readFileSync } from "node:fs";
import { beforeAll, describe, expect, it } from "vitest";
import { limitPullPower, pullDemand, resolvePullParams } from "../../src/effectors/pull";
import { ToriBudget } from "../../src/effectors/toriBudget";
import type { Recording } from "../../src/recording/schema";
import { runScenario } from "../../src/scenario/run";
import { ScenarioValidationError, validateScenario } from "../../src/scenario/validate";

/** Tirón guiado (`kind: "pull"`), presupuesto de Tori y el escenario `uke_tense_75_175_hip_roll` (MuJoCo WASM real). */
const DT = 0.002;

function raw(): Record<string, unknown> {
  return JSON.parse(readFileSync(new URL("../../scenarios/uke_tense_75_175_hip_roll.json", import.meta.url), "utf8")) as Record<string, unknown>;
}

describe("tirón guiado (puro)", () => {
  const keys = [
    { t_s: 0.2, force_n: [0, 0, 0] as const },
    { t_s: 0.4, force_n: [300, 0, 300] as const },
    { t_s: 0.8, force_n: [300, 0, -300] as const },
  ];

  it("interpola linealmente y es nulo fuera de la curva", () => {
    expect(pullDemand(keys, 0.1)).toEqual([0, 0, 0]);
    const close = (v: readonly number[], w: readonly number[]): void => v.forEach((x, i) => expect(x).toBeCloseTo(w[i]!, 9));
    close(pullDemand(keys, 0.3), [150, 0, 150]);
    close(pullDemand(keys, 0.6), [300, 0, 0]);
    expect(pullDemand(keys, 0.8)).toEqual([0, 0, 0]);
  });

  it("rechaza puntos por encima del tope de fuerza, instantes desordenados y curvas de un punto", () => {
    expect(() => resolvePullParams({ keys, max_force_n: 400 })).toThrow(/keys\[1\]\.force_n/);
    expect(() => resolvePullParams({ keys: [keys[1]!, keys[0]!] })).toThrow(/keys\[1\]\.t_s/);
    expect(() => resolvePullParams({ keys: [keys[0]!] })).toThrow(/keys/);
    expect(resolvePullParams({ keys })).toMatchObject({ max_force_n: 500, max_power_w: 600 });
  });

  it("el tope de potencia reduce la fuerza que acelera y no toca la que frena", () => {
    const params = resolvePullParams({ keys });
    const fast = limitPullPower(params, [400, 0, 0], [3, 0, 0]);
    expect(fast.power_limited).toBe(true);
    expect(fast.force_n[0]).toBeCloseTo(200, 9);
    expect(limitPullPower(params, [400, 0, 0], [-3, 0, 0])).toMatchObject({ power_limited: false, force_n: [400, 0, 0] });
  });
});

describe("presupuesto de Tori (puro)", () => {
  const run = (force: readonly [number, number, number], seconds: number, power = 0): ReturnType<ToriBudget["summary"]> => {
    const budget = new ToriBudget({ mass_kg: 75, friction: 0.8 });
    for (let i = 0; i < Math.round(seconds / DT); i++) {
      budget.step({ t: i * DT, dt: DT, hands_force_n: force, hands_power_w: power, obstacle_force_on_uke_n: [0, 0, 0] });
    }
    return budget.summary();
  };

  it("una mano que tira 2000 N durante 1 s es inviable por tiro total y por rozamiento", () => {
    const s = run([2000, 0, 0], 1);
    expect(s.feasible).toBe(false);
    expect(s.violations.map((v) => v.kind)).toEqual(expect.arrayContaining(["total_pull", "friction", "horizontal_impulse"]));
  });

  it("un tirón de 400 N durante 0,4 s con Tori de 75 kg y µ = 0,8 es viable", () => {
    const s = run([400, 0, 0], 0.4);
    expect(s.feasible).toBe(true);
    expect(s.friction_use.value).toBeCloseTo(400 / (0.8 * 75 * 9.81), 6);
    expect(s.max_horizontal_impulse_ns.value).toBeCloseTo(160, 6);
  });

  it("tirar hacia abajo más que el propio peso deja a Tori sin apoyo", () => {
    const s = run([0, 0, -800], 0.2);
    expect(s.violations.map((v) => v.kind)).toContain("feet_unloaded");
    expect(s.min_feet_load_n.value).toBeCloseTo(75 * 9.81 - 800, 6);
  });

  it("la carga de Uke sobre la cadera aumenta el apoyo de Tori y la potencia total tiene tope", () => {
    const budget = new ToriBudget({ mass_kg: 75, friction: 0.8 });
    budget.step({ t: 0, dt: DT, hands_force_n: [0, 0, 0], hands_power_w: 1600, obstacle_force_on_uke_n: [-300, 0, 700] });
    const s = budget.summary();
    expect(s.max_feet_load_n.value).toBeCloseTo(75 * 9.81 + 700, 6);
    expect(s.violations.map((v) => v.kind)).toEqual(["total_power"]);
  });
});

describe("validador: `pull` y `tori`", () => {
  it("acepta el escenario y señala los campos erróneos", () => {
    const s = validateScenario(raw());
    expect(s.effectors.map((e) => e.kind)).toEqual(["pull", "pull"]);
    expect(s.tori).toEqual({ mass_kg: 75 });
    const fails = (change: (json: Record<string, unknown>) => void): string => {
      const json = raw();
      change(json);
      try {
        validateScenario(json);
      } catch (error) {
        if (error instanceof ScenarioValidationError) return error.path;
        throw error;
      }
      return "(válido)";
    };
    const firstHand = (json: Record<string, unknown>): Record<string, unknown> => (json.effectors as Array<Record<string, unknown>>)[0]!;
    expect(fails((j) => { firstHand(j).max_force_n = 100; })).toBe("effectors[0].keys[1].force_n");
    expect(fails((j) => { firstHand(j).frame = "tori"; })).toBe("effectors[0].frame");
    expect(fails((j) => { (j.tori as Record<string, unknown>).mass_kg = 0; })).toBe("tori.mass_kg");
  });
});

describe("ejecutor: volteo con tirón en arco (`uke_tense_75_175_hip_roll`)", () => {
  let rec: Recording;
  beforeAll(async () => {
    rec = await runScenario(validateScenario(raw()), { engineVersion: "test" });
  }, 120_000);

  const elevation = (f: readonly number[]): number => (Math.atan2(f[2]!, f[0]!) * 180) / Math.PI;
  const at = (t: number) => rec.samples.find((s) => s.t >= t - 1e-9)!;

  it("la fuerza de las dos manos describe el arco: 45° arriba, horizontal, 45° abajo y casi vertical hacia Tori", () => {
    for (const k of [0, 1]) {
      expect(elevation(at(0.5).effectors[k]!.force_n)).toBeCloseTo(45, 0);
      expect(Math.abs(elevation(at(0.8).effectors[k]!.force_n))).toBeLessThan(3);
      expect(elevation(at(1.0).effectors[k]!.force_n)).toBeCloseTo(-45, 0);
      const late = at(1.2).effectors[k]!.force_n;
      expect(late[2]!).toBeLessThan(0);
      expect(late[0]!).toBeLessThan(0); // ya tira hacia Tori (−X de Uke)
      // La elevación no vuelve a subir: nunca cambia de sentido.
      let previous = 90;
      for (const s of rec.samples) {
        const f = s.effectors[k]!.force_n;
        if (Math.hypot(...f) < 20) continue;
        const e = elevation(f) > 90 ? elevation(f) - 360 : elevation(f);
        expect(e).toBeLessThanOrEqual(previous + 1e-6);
        previous = e;
      }
      const peak = rec.peaks.effectors[k === 0 ? "hand_sleeve" : "hand_lapel"]!;
      expect(peak.max_force_n).toBeLessThanOrEqual(400.1);
      expect(peak.pull!.max_power_w).toBeLessThanOrEqual(600 * (1 + 1e-9));
    }
  });

  it("Uke da la vuelta y no cae de cabeza: la cabeza no recibe ningún golpe por encima del umbral", () => {
    expect(rec.summary.outcome).toBe("fall");
    expect(rec.summary.max_tilt_deg).toBeGreaterThan(150);
    expect(rec.summary.uke!.fall!.first_part).not.toBe("head");
    expect(rec.summary.artifacts!.some((a) => a.type === "contact_force")).toBe(false);
    const headHits = rec.samples.flatMap((s) => s.contacts)
      .filter((c) => [c.body_a, c.body_b].includes("head") && [c.body_a, c.body_b].includes("world"));
    expect(Math.max(0, ...headHits.map((c) => Math.hypot(...c.force_n)))).toBeLessThan(3000);
  });

  it("la barra carga la pelvis de Uke (no los muslos) durante el volteo", () => {
    const touching = (body: string): number => rec.samples.filter((s) => s.t > 0.6 && s.contacts.some(
      (c) => [c.body_a, c.body_b].includes("hip_bar") && [c.body_a, c.body_b].includes(body),
    )).length / rec.meta.record_hz;
    expect(touching("pelvis")).toBeGreaterThan(0.3);
    expect(touching("left_thigh") + touching("right_thigh")).toBeLessThan(0.05);
  });

  it("Tori queda dentro de su presupuesto", () => {
    const b = rec.summary.tori_budget!;
    expect(b.violations).toEqual([]);
    expect(b.feasible).toBe(true);
    expect(b.friction_use.value).toBeLessThan(1);
    expect(b.max_total_pull_n.value).toBeLessThanOrEqual(800.1);
    expect(b.min_feet_load_n.value).toBeGreaterThan(0);
    expect(b.max_feet_load_n.value).toBeGreaterThan(75 * 9.81 + 500); // carga a Uke sobre la cadera
  });

  it("con la cadera resbaladiza (µ = 0,2) Uke se escurre y cae de cabeza", async () => {
    const json = raw();
    (json.obstacles as Array<Record<string, unknown>>)[0]!.friction = 0.2;
    const slippery = await runScenario(validateScenario(json), { engineVersion: "test" });
    expect(slippery.summary.artifacts!.some((a) => a.type === "contact_force" && a.where.includes("head"))).toBe(true);
  }, 120_000);
});
