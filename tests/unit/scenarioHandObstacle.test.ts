import { readFileSync } from "node:fs";
import { beforeAll, describe, expect, it } from "vitest";
import type { Recording } from "../../src/recording/schema";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { runScenario, scenarioSha256 } from "../../src/scenario/run";
import { ScenarioValidationError, validateScenario } from "../../src/scenario/validate";

/**
 * Mano virtual (`kind: "hand"`) y obstáculo con fuerza limitada (`obstacles`) dentro del formato de escenario:
 * validador, modelo y ejecutor con MuJoCo WASM real, sobre `scenarios/uke_tense_75_175_hip_tip.json`.
 */
const FILE = "uke_tense_75_175_hip_tip.json";

function raw(): Record<string, unknown> {
  return JSON.parse(readFileSync(new URL(`../../scenarios/${FILE}`, import.meta.url), "utf8")) as Record<string, unknown>;
}

function failure(json: unknown): ScenarioValidationError {
  try {
    validateScenario(json);
  } catch (error) {
    if (error instanceof ScenarioValidationError) return error;
    throw error;
  }
  throw new Error("se esperaba un error de validación");
}

describe("validador: manos y obstáculos", () => {
  it("acepta el escenario de vuelco y conserva los campos opcionales sin rellenar valores por defecto", () => {
    const s = validateScenario(raw(), { fileName: FILE });
    expect(s.effectors.map((e) => e.kind)).toEqual(["hand", "hand"]);
    const hand = s.effectors[0]!;
    expect(hand.kind === "hand" && hand.body).toBe("right_forearm"); // rellenado desde el sitio
    expect(hand.kind === "hand" && hand.max_force_n).toBeUndefined();
    expect(s.obstacles).toHaveLength(1);
    expect(s.obstacles![0]!.id).toBe("hip_bar");
  });

  it("un escenario sin obstáculos no gana la clave `obstacles` (su hash no cambia)", () => {
    const json = raw();
    delete json.obstacles;
    const s = validateScenario(json);
    expect("obstacles" in s).toBe(false);
    expect(scenarioSha256(s)).toMatch(/^[0-9a-f]{64}$/);
  });

  it("señala el campo de la mano que incumple una regla de `resolveHandParams`", () => {
    const json = raw();
    (json.effectors as Array<Record<string, unknown>>)[0]!.axis = [1, 0, 0];
    expect(failure(json).path).toBe("effectors[0].axis");
    const late = raw();
    (late.effectors as Array<Record<string, unknown>>)[1]!.release_s = 0.1;
    expect(failure(late).path).toBe("effectors[1].release_s");
  });

  it("señala los errores del obstáculo: forma, id, estabilidad e id repetido con un prop", () => {
    const edit = (change: (o: Record<string, unknown>, root: Record<string, unknown>) => void): ScenarioValidationError => {
      const json = raw();
      change((json.obstacles as Array<Record<string, unknown>>)[0]!, json);
      return failure(json);
    };
    expect(edit((o) => { o.shape = "wall"; }).path).toBe("obstacles[0].shape");
    expect(edit((o) => { o.id = "9bar"; }).path).toBe("obstacles[0].id");
    expect(edit((o) => { o.size_m = [0.1, 0.1, 0.1]; }).path).toBe("obstacles[0].size_m");
    expect(edit((o) => { o.mass_kg = 0.01; }).path).toBe("obstacles[0].kp_n_per_m");
    expect(edit((o) => { o.rotation = {}; }).reason).toBe("campo desconocido");
    expect(edit((_, root) => {
      root.props = [{ id: "hip_bar", kind: "fixed_box", size_m: [0.1, 0.1, 0.1], pos_m: [2, 0, 0.05] }];
    }).path).toBe("obstacles[0].id");
  });
});

describe("modelo: el obstáculo va después del actor", () => {
  it("pelvis sigue siendo el cuerpo 1 y las deslizaderas del obstáculo son las últimas articulaciones", () => {
    const built = buildScenarioModel(validateScenario(raw()));
    expect(built.bodies[1]).toBe("pelvis");
    expect(built.bodies[built.bodies.length - 1]).toBe("hip_bar");
    expect(built.joints.slice(-3)).toEqual(["hip_bar_x", "hip_bar_y", "hip_bar_z"]);
    expect(built.obstacles[0]!.mass_kg).toBe(37.5);
  });
});

describe("ejecutor: vuelco sobre la cadera con dos manos de 500 N y una barra", () => {
  let rec: Recording;
  beforeAll(async () => {
    rec = await runScenario(validateScenario(raw()), { engineVersion: "test" });
  }, 120_000);

  it("Uke cae después de girar por encima de la barra", () => {
    expect(rec.summary.outcome).toBe("fall");
    expect(rec.summary.max_tilt_deg).toBeGreaterThan(120);
    expect(rec.summary.expect).toBe("pass");
  });

  it("las manos respetan su tope, registran cierre y suelta y se graban como `hand`", () => {
    for (const id of ["hand_sleeve", "hand_lapel"]) {
      const peak = rec.peaks.effectors[id]!;
      expect(peak.limit_n).toBe(500);
      expect(peak.max_force_n).toBeLessThanOrEqual(500);
      expect(peak.max_force_n).toBeGreaterThan(300);
      expect(peak.hand!.max_power_w).toBeLessThanOrEqual(600 * 1.02);
      expect(peak.hand!.events.map((e) => e.kind)).toEqual(["close", "release"]);
      expect(peak.hand!.events[0]!.t_s).toBeCloseTo(0.2, 9);
      expect(peak.hand!.events[1]!.t_s).toBeCloseTo(1.3, 9);
    }
    const before = rec.samples[0]!.effectors[0]!;
    expect(before.kind).toBe("hand");
    expect(before.hand).toEqual({ held: false, target_m: null, force_limited: false, power_limited: false, reach_limited: false });
    const during = rec.samples.find((s) => s.t >= 0.4)!.effectors[1]!;
    expect(during.hand!.held).toBe(true);
    expect(Math.hypot(...during.force_n)).toBeGreaterThan(100);
    const after = rec.samples[rec.samples.length - 1]!.effectors[1]!;
    expect(after.hand!.held).toBe(false);
    expect(after.force_n).toEqual([0, 0, 0]);
    expect(rec.summary.artifacts!.some((a) => a.type === "effector_over_limit")).toBe(false);
  });

  it("la barra respeta sus topes, recibe la carga de Uke y cede una distancia medida", () => {
    const bar = rec.peaks.obstacles!.hip_bar!;
    expect(bar.limit_horizontal_n).toBeCloseTo(0.8 * 75 * 9.81, 9);
    expect(bar.limit_vertical_n).toBeCloseTo(2 * 75 * 9.81, 9);
    expect(bar.max_horizontal_force_n).toBeLessThanOrEqual(bar.limit_horizontal_n * (1 + 1e-9));
    expect(bar.max_vertical_force_n).toBeLessThanOrEqual(bar.limit_vertical_n * (1 + 1e-9));
    expect(bar.max_contact_force_on_actor_n).toBeGreaterThan(75 * 9.81); // llega a cargar más que el peso de Uke
    expect(bar.max_deviation_m).toBeGreaterThan(0.01);
    expect(bar.max_deviation_m).toBeLessThan(0.3);
    // Antes del tirón la barra está quieta en su sitio: las tres deslizaderas (últimos qpos) a cero.
    expect(rec.samples[0]!.qpos.slice(-3).every((q) => Math.abs(q) < 1e-3)).toBe(true);
  });

  it("es determinista", async () => {
    const again = await runScenario(validateScenario(raw()), { engineVersion: "test" });
    expect(again.samples[again.samples.length - 1]!.qpos).toEqual(rec.samples[rec.samples.length - 1]!.qpos);
    expect(again.peaks).toEqual(rec.peaks);
  }, 120_000);
});
