import { gunzipSync } from "node:zlib";
import { describe, expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { gzipRecording, serializeRecording } from "../../src/recording/write";
import type { Recording } from "../../src/recording/schema";
import type { Scenario } from "../../src/scenario/schema";

const MASS = 75;
const HEIGHT = 1.75;
const RADIUS = 0.18;
const G = 9.81;

interface Overrides {
  id?: string;
  duration_s?: number;
  friction?: number;
  props?: unknown[];
  effectors?: unknown[];
  expect?: unknown;
}

function scenario(o: Overrides = {}): Scenario {
  const json: Record<string, unknown> = {
    schema: "hpl.scenario.v1",
    id: o.id ?? "test_case",
    duration_s: o.duration_s ?? 2,
    record_hz: 125,
    ground: { friction: o.friction ?? 0.6 },
    actors: [{ id: "uke", kind: "rigid_cylinder", mass_kg: MASS, height_m: HEIGHT, radius_m: RADIUS }],
    props: o.props ?? [],
    effectors: o.effectors ?? [],
  };
  if (o.expect !== undefined) json.expect = o.expect;
  return validateScenario(json);
}

function pull(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    id: "pull", kind: "prescribed_force", actor: "uke", body: "cylinder",
    local_point_m: [0, 0, 0.525], frame: "world", force_n: [600, 0, 180],
    start_s: 0.3, duration_s: 0.15, waveform: "half_sine",
    ...overrides,
  };
}

/** Energía mecánica del cilindro libre a partir de qpos/qvel (qvel angular en ejes locales). */
function mechanicalEnergyJ(qpos: number[], qvel: number[]): number {
  const ixy = MASS * (3 * RADIUS * RADIUS + HEIGHT * HEIGHT) / 12;
  const iz = MASS * RADIUS * RADIUS / 2;
  const kinetic = 0.5 * MASS * (qvel[0]! ** 2 + qvel[1]! ** 2 + qvel[2]! ** 2) +
    0.5 * (ixy * qvel[3]! ** 2 + ixy * qvel[4]! ** 2 + iz * qvel[5]! ** 2);
  return kinetic + MASS * G * qpos[2]!;
}

describe("runScenario (MuJoCo WASM real)", () => {
  it("es determinista: dos ejecuciones dan el mismo JSON", async () => {
    const s = scenario({
      props: [{ id: "obstacle", kind: "fixed_box", size_m: [0.12, 0.8, 0.5], pos_m: [0.26, 0, 0.25] }],
      effectors: [pull({ force_n: [900, 100, 270] })],
    });
    const a = JSON.stringify(await runScenario(s, { engineVersion: "test" }));
    const b = JSON.stringify(await runScenario(s, { engineVersion: "test" }));
    expect(a.length).toBeGreaterThan(1000);
    expect(a === b).toBe(true);
  });

  it("sin efectores el cilindro sigue de pie y su energía no crece; el suelo sostiene su peso", async () => {
    const rec = await runScenario(scenario({ duration_s: 3 }), { engineVersion: "test" });
    expect(rec.summary.outcome).toBe("standing");
    expect(rec.summary.max_tilt_deg).toBeLessThan(0.5);
    expect(rec.summary.slide_m).toBeLessThan(1e-3);
    const e0 = mechanicalEnergyJ(rec.samples[0]!.qpos, rec.samples[0]!.qvel);
    let maxGrowth = -Infinity;
    for (const s of rec.samples) maxGrowth = Math.max(maxGrowth, mechanicalEnergyJ(s.qpos, s.qvel) - e0);
    console.log(`energía sin efectores: crecimiento máximo ${maxGrowth.toExponential(3)} J sobre ${e0.toFixed(1)} J`);
    expect(maxGrowth).toBeLessThan(0.1);
    // Convención de la fuerza de contacto: suma de lo que recibe el cilindro = su peso.
    const last = rec.samples[rec.samples.length - 1]!;
    let fz = 0;
    for (const c of last.contacts) {
      if (c.body_b === "cylinder") fz += c.force_n[2];
      else if (c.body_a === "cylinder") fz -= c.force_n[2];
    }
    console.log(`fuerza vertical de contacto sobre el cilindro: ${fz.toFixed(2)} N (peso ${(MASS * G).toFixed(2)} N)`);
    expect(Math.abs(fz - MASS * G) / (MASS * G)).toBeLessThan(0.01);
  });

  it("el impulso de una fuerza de medio seno coincide con 2·F·T/π (error < 1 %)", async () => {
    const force = 1000;
    const duration = 0.2;
    const rec = await runScenario(
      scenario({ duration_s: 1, effectors: [pull({ force_n: [force, 0, 0], start_s: 0.2, duration_s: duration })] }),
      { engineVersion: "test" },
    );
    const peak = rec.peaks.effectors.pull!;
    const expected = 2 * force * duration / Math.PI;
    const error = Math.abs(peak.impulse_ns[0] - expected) / expected;
    console.log(`impulso medio seno: ${peak.impulse_ns[0].toFixed(4)} N·s, esperado ${expected.toFixed(4)} N·s, error ${(error * 100).toFixed(4)} %`);
    expect(error).toBeLessThan(0.01);
    expect(Math.abs(peak.impulse_ns[1])).toBeLessThan(1e-9);
    expect(peak.max_force_n).toBeGreaterThan(0.999 * force);
    expect(peak.max_force_n).toBeLessThanOrEqual(force);
  });

  it("el impulso de una fuerza constante es F·T", async () => {
    const rec = await runScenario(
      scenario({ duration_s: 1, effectors: [pull({ force_n: [300, 0, 100], waveform: "constant", duration_s: 0.1 })] }),
      { engineVersion: "test" },
    );
    const peak = rec.peaks.effectors.pull!;
    expect(Math.abs(peak.impulse_ns[0] - 30) / 30).toBeLessThan(1e-6);
    expect(Math.abs(peak.impulse_ns[2] - 10) / 10).toBeLessThan(1e-6);
  });

  it("el trabajo de una fuerza por el centro de masas sobre suelo sin rozamiento es la energía cinética", async () => {
    // F = 300 N, 0,2 s, 75 kg: v = 0,8 m/s, x = 0,08 m, W = F x = 24 J = 0,5 m v².
    const rec = await runScenario(
      scenario({
        duration_s: 1, friction: 0,
        effectors: [pull({ local_point_m: [0, 0, 0], force_n: [300, 0, 0], waveform: "constant", duration_s: 0.2 })],
      }),
      { engineVersion: "test" },
    );
    const work = rec.peaks.effectors.pull!.work_j;
    const last = rec.samples[rec.samples.length - 1]!;
    const kinetic = 0.5 * MASS * (last.qvel[0]! ** 2 + last.qvel[1]! ** 2 + last.qvel[2]! ** 2);
    console.log(`trabajo ${work.toFixed(4)} J; energía cinética final ${kinetic.toFixed(4)} J; analítico 24 J`);
    expect(Math.abs(work - 24) / 24).toBeLessThan(0.01);
    expect(Math.abs(work - kinetic) / kinetic).toBeLessThan(0.01);
  });

  it("el trabajo de una fuerza fuera del centro de masas (con giro) iguala el aumento de energía mecánica", async () => {
    // Suelo sin rozamiento: la reacción normal no trabaja mientras el cilindro no rebota; el trabajo
    // del efector es todo energía cinética (traslación y giro) más potencial.
    const rec = await runScenario(
      scenario({
        duration_s: 1, friction: 0,
        effectors: [pull({ force_n: [300, 0, 0], waveform: "constant", start_s: 0.2, duration_s: 0.2 })],
      }),
      { engineVersion: "test" },
    );
    const at = (t: number) => rec.samples.find((s) => Math.abs(s.t - t) < 1e-9)!;
    const gain = mechanicalEnergyJ(at(0.4).qpos, at(0.4).qvel) - mechanicalEnergyJ(at(0.2).qpos, at(0.2).qvel);
    const work = rec.peaks.effectors.pull!.work_j;
    console.log(`trabajo con giro ${work.toFixed(4)} J; aumento de energía mecánica ${gain.toFixed(4)} J; inclinación ${at(0.4).qvel[4]!.toFixed(3)} rad/s`);
    expect(Math.abs(at(0.4).qvel[4]!)).toBeGreaterThan(0.2); // gira de verdad (poco: el borde inferior se apoya)
    expect(Math.abs(work - gain) / gain).toBeLessThan(0.02);
  });

  it("simetría: reflejar fuerza y obstáculo en Y refleja qpos y qvel (error < 1e-6 durante 1 s)", async () => {
    const build = (sy: 1 | -1): Scenario => scenario({
      duration_s: 1.2,
      props: [{ id: "obstacle", kind: "fixed_box", size_m: [0.12, 0.8, 0.5], pos_m: [0.26, 0.1 * sy, 0.25] }],
      effectors: [pull({ force_n: [900, 250 * sy, 200], local_point_m: [0.18, 0.05 * sy, 0.525] })],
    });
    const a = await runScenario(build(1), { engineVersion: "test" });
    const b = await runScenario(build(-1), { engineVersion: "test" });
    let worst = 0;
    let compared = 0;
    for (let k = 0; k < a.samples.length; k++) {
      const sa = a.samples[k]!;
      const sb = b.samples[k]!;
      if (sa.t > 1 + 1e-9) break;
      const [x, y, z, w, qx, qy, qz] = sa.qpos as [number, number, number, number, number, number, number];
      const mirrored = [x, -y, z, w, -qx, qy, -qz];
      const [vx, vy, vz, wx, wy, wz] = sa.qvel as [number, number, number, number, number, number];
      const mirroredVel = [vx, -vy, vz, -wx, wy, -wz];
      for (let i = 0; i < 7; i++) worst = Math.max(worst, Math.abs(mirrored[i]! - sb.qpos[i]!));
      for (let i = 0; i < 6; i++) worst = Math.max(worst, Math.abs(mirroredVel[i]! - sb.qvel[i]!));
      compared++;
    }
    console.log(`simetría Y: ${compared} muestras, error máximo ${worst.toExponential(3)}`);
    expect(compared).toBeGreaterThan(100);
    // Debe haber movimiento real (no es una comparación trivial de un cilindro quieto).
    expect(Math.abs(a.samples[compared - 1]!.qpos[1]!)).toBeGreaterThan(1e-3);
    expect(worst).toBeLessThan(1e-6);
  });

  it("el marco actor_initial rota la fuerza con los ejes del actor en t=0 (igual que world al empezar de pie)", async () => {
    const world = await runScenario(scenario({ duration_s: 1, effectors: [pull()] }), { engineVersion: "test" });
    const initial = await runScenario(
      scenario({ duration_s: 1, effectors: [pull({ frame: "actor_initial" })] }),
      { engineVersion: "test" },
    );
    expect(initial.peaks.effectors.pull!.impulse_ns[0]).toBeCloseTo(world.peaks.effectors.pull!.impulse_ns[0], 6);
    expect(Math.abs(initial.summary.final_tilt_deg - world.summary.final_tilt_deg)).toBeLessThan(0.5);
  });

  it("el veredicto y el resumen de una caída son coherentes con expect", async () => {
    const rec = await runScenario(
      scenario({
        duration_s: 3,
        effectors: [pull({ force_n: [900, 0, 270], start_s: 0.3, duration_s: 0.3 })],
        expect: { outcome: "fall", direction_deg: 0, tolerance_deg: 30 },
      }),
      { engineVersion: "test" },
    );
    expect(rec.summary.outcome).toBe("fall");
    expect(Math.abs(rec.summary.direction_deg!)).toBeLessThan(5);
    expect(rec.summary.expect).toBe("pass");
    expect(rec.summary.expect_diffs).toEqual([]);
    expect(rec.summary.t_outcome_s).toBeGreaterThan(0.3);
    const wrong = await runScenario(
      scenario({ duration_s: 1, expect: { outcome: "fall" } }),
      { engineVersion: "test" },
    );
    expect(wrong.summary.expect).toBe("fail");
    expect(wrong.summary.expect_diffs[0]).toContain('esperado "fall", obtenido "standing"');
  });
});

describe("grabación: formato y serialización", () => {
  it("meta, MJCF, muestreo a record_hz y redondeo a 6 cifras", async () => {
    const rec: Recording = await runScenario(
      scenario({ duration_s: 1, effectors: [pull({ force_n: [900, 0, 270] })] }),
      { engineVersion: "9.9.9" },
    );
    expect(rec.schema).toBe("hpl.recording.v1");
    expect(rec.meta).toMatchObject({
      scenario_id: "test_case", engine_version: "9.9.9", physics_hz: 500, record_hz: 125,
      frame: "x_forward_y_left_z_up", quat_order: "wxyz",
    });
    expect(rec.meta.scenario_sha256).toMatch(/^[0-9a-f]{64}$/);
    expect(rec.meta.mujoco_version).toMatch(/^\d+\.\d+\.\d+/);
    expect(rec.model.mjcf).toContain("<option ");
    expect(rec.model.mjcf).toContain('cone="elliptic"');
    expect(rec.model.bodies).toEqual(["world", "cylinder"]);
    expect(rec.model.joints).toEqual(["cylinder_free"]);
    expect(rec.samples).toHaveLength(126);
    expect(rec.samples[1]!.t).toBeCloseTo(0.008, 12);
    expect(rec.samples[0]!.qpos).toHaveLength(7);
    expect(rec.samples[0]!.qvel).toHaveLength(6);

    const text = serializeRecording(rec);
    const parsed = JSON.parse(text) as Recording;
    for (const n of parsed.samples[10]!.qpos) {
      expect(Number(n.toPrecision(6))).toBe(n);
    }
    expect(parsed.model.mjcf).toBe(rec.model.mjcf);
    const gz = gzipRecording(rec);
    expect(gunzipSync(gz).toString("utf8")).toBe(text);
    expect(gzipRecording(rec).equals(gz)).toBe(true);
  });

  it("los picos se calculan a 500 Hz, no a record_hz: un pulso de 10 ms entre dos muestras no se pierde", async () => {
    // A 50 Hz las muestras caen cada 20 ms; el pulso de 10 ms empieza en 0,31 s.
    const json = {
      schema: "hpl.scenario.v1", id: "pulse", duration_s: 1, record_hz: 50, ground: { friction: 0.6 },
      actors: [{ id: "uke", kind: "rigid_cylinder", mass_kg: 75, height_m: 1.75, radius_m: 0.18 }],
      props: [], effectors: [pull({ start_s: 0.31, duration_s: 0.01, waveform: "constant", force_n: [500, 0, 0] })],
    };
    const rec = await runScenario(validateScenario(json), { engineVersion: "test" });
    expect(rec.samples.every((s) => s.effectors[0]!.force_n[0] === 0)).toBe(true);
    expect(rec.peaks.effectors.pull!.max_force_n).toBe(500);
    expect(rec.peaks.effectors.pull!.impulse_ns[0]).toBeCloseTo(5, 9);
  });
});
