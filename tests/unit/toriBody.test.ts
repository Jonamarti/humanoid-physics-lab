import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import {
  TORI_SEGMENTS, buildToriBodyMjcf, defaultToriKeys, quatFromZ, resolveToriBodyParams, toriPoseAt, toriSegmentCommand,
  toriSegmentLength, toriSkeleton, type Vec3,
} from "../../src/effectors/toriBody";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { describeTrial } from "../../src/scenario/editing";
import { runScenario } from "../../src/scenario/run";
import { ScenarioValidationError, validateScenario } from "../../src/scenario/validate";

const dist = (a: Vec3, b: Vec3): number => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const params = resolveToriBodyParams({ mass_kg: 75, height_m: 1.75, keys: defaultToriKeys(1.75) });

function load(): Record<string, unknown> {
  return JSON.parse(readFileSync(new URL("../../scenarios/uke_tense_75_175_tori_body.json", import.meta.url), "utf8")) as Record<string, unknown>;
}

describe("cuerpo de Tori: parámetros y esqueleto (sin motor)", () => {
  it("reparte toda la masa entre los segmentos y deriva los topes del peso", () => {
    expect(params.segments.map((s) => s.segment)).toEqual([...TORI_SEGMENTS]);
    expect(params.segments.reduce((sum, s) => sum + s.mass_kg, 0)).toBeCloseTo(75, 9);
    const trunk = params.segments.find((s) => s.segment === "trunk")!;
    expect(trunk.max_horizontal_force_n).toBeCloseTo(0.8 * 75 * 9.81, 6);
    expect(trunk.max_vertical_force_n).toBeCloseTo(2 * 75 * 9.81, 6);
    expect(params.segments.filter((s) => !s.collides).map((s) => s.segment)).toEqual(
      ["left_upperarm", "left_forearm", "right_upperarm", "right_forearm"]);
  });

  it("rechaza datos fuera de rango", () => {
    const keys = defaultToriKeys(1.75);
    expect(() => resolveToriBodyParams({ mass_kg: 10, height_m: 1.75, keys })).toThrow(RangeError);
    expect(() => resolveToriBodyParams({ mass_kg: 75, height_m: 1.75, keys: [] })).toThrow(/al menos una postura/);
    expect(() => resolveToriBodyParams({ mass_kg: 75, height_m: 1.75, keys: [keys[1]!, keys[0]!] })).toThrow(/mayor que el de la postura anterior/);
  });

  it("interpola entre posturas y se queda en la primera y en la última fuera de ellas", () => {
    const keys = defaultToriKeys(1.75);
    expect(toriPoseAt(keys, 0).pelvis_m).toEqual(keys[0]!.pelvis_m);
    expect(toriPoseAt(keys, 99).yaw_deg).toBe(keys[keys.length - 1]!.yaw_deg);
    const mid = toriPoseAt(keys, (keys[0]!.t_s + keys[1]!.t_s) / 2);
    expect(mid.yaw_deg).toBeCloseTo((keys[0]!.yaw_deg + keys[1]!.yaw_deg) / 2, 9);
  });

  it("los segmentos conservan su longitud en toda la secuencia y los pies no bajan del suelo", () => {
    for (let t = 0; t <= 2.2; t += 0.05) {
      const skeleton = toriSkeleton(params, t, { left: [0.1, -0.2, 1.2], right: [0.1, 0, 1.3] });
      for (const segment of TORI_SEGMENTS) {
        const ends = skeleton[segment];
        expect(dist(ends.a, ends.b), `${segment} en t=${t.toFixed(2)}`).toBeCloseTo(toriSegmentLength(params, segment), 6);
      }
      expect(skeleton.left_foot.a[2]).toBeGreaterThan(0);
      expect(skeleton.right_foot.b[2]).toBeGreaterThan(0);
      // Rodillas y codos unen sus dos tramos.
      expect(dist(skeleton.left_thigh.b, skeleton.left_shin.a)).toBe(0);
      expect(dist(skeleton.right_upperarm.b, skeleton.right_forearm.a)).toBe(0);
    }
  });

  it("de pie, mira a Uke con la cabeza a su altura; al final de la entrada le da la espalda agachado", () => {
    const start = toriSkeleton(params, 0);
    expect(start.head.a[2] + params.dims.head_radius).toBeCloseTo(1.75 * 0.98 + 0.03, 1);
    expect(start.pelvis.a[1]).toBeLessThan(start.pelvis.b[1]); // su izquierda está en −Y: mira hacia −X
    const loaded = toriSkeleton(params, 0.85);
    expect(loaded.pelvis.a[1]).toBeGreaterThan(loaded.pelvis.b[1]);
    expect(loaded.pelvis.a[2]).toBeLessThan(start.pelvis.a[2] - 0.2);
    expect(loaded.trunk.b[0]).toBeGreaterThan(loaded.trunk.a[0] + 0.1); // tronco inclinado hacia +X, lejos de Uke
  });

  it("quatFromZ lleva el eje Z al vector pedido", () => {
    for (const axis of [[0, 0, 1], [1, 0, 0], [0, -1, 0], [0.6, 0, -0.8], [0, 0, -1]] as Vec3[]) {
      const [w, x, y, z] = quatFromZ(axis);
      // Tercera columna de la matriz de rotación del cuaternión.
      const rotated: Vec3 = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)];
      expect(dist(rotated, axis)).toBeLessThan(1e-9);
    }
  });

  it("la orden sostiene el peso en el objetivo y se recorta a los topes lejos de él", () => {
    const trunk = params.segments.find((s) => s.segment === "trunk")!;
    const ends = { a: [0, 0, 1] as Vec3, b: [0, 0, 1.4] as Vec3 };
    const still = { velocity_mps: [0, 0, 0] as Vec3, axis: [0, 0, 1] as Vec3, angular_velocity_rps: [0, 0, 0] as Vec3, inertia_transverse: 0.5, inertia_axial: 0.2 };
    const at = toriSegmentCommand(trunk, { ...still, position_m: [0, 0, 1.2] }, ends, ends, 0.002);
    expect(at.force_n[0]).toBe(0);
    expect(at.force_n[2]).toBeCloseTo(trunk.mass_kg * 9.81, 9);
    expect(at.saturated).toBe(false);
    expect(Math.hypot(...at.torque_nm)).toBe(0);
    const far = toriSegmentCommand(trunk, { ...still, position_m: [1, 0, 0.2], axis: [1, 0, 0] }, ends, ends, 0.002);
    expect(far.saturated).toBe(true);
    expect(Math.hypot(far.force_n[0], far.force_n[1])).toBeCloseTo(trunk.max_horizontal_force_n, 6);
    expect(far.force_n[2]).toBeCloseTo(trunk.max_vertical_force_n, 6);
    expect(Math.hypot(...far.torque_nm)).toBeLessThanOrEqual(trunk.max_torque_nm * (1 + 1e-9));
    expect(far.deviation_m).toBeCloseTo(Math.hypot(1, 1), 9);
  });

  it("el MJCF trae un cuerpo libre por segmento y los brazos sin colisión", () => {
    const xml = buildToriBodyMjcf(params);
    expect(xml.match(/<freejoint /g)).toHaveLength(TORI_SEGMENTS.length);
    expect(xml.match(/contype="0" conaffinity="0"/g)).toHaveLength(4);
    expect(xml).toContain('name="tori_head_geom" type="sphere"');
  });
});

describe("cuerpo de Tori en el escenario (MuJoCo WASM real)", () => {
  it("el validador comprueba las manos y las posturas", () => {
    const bad = load();
    ((bad.tori as Record<string, unknown>).body as Record<string, unknown>).hands = { left: "no_existe" };
    expect(() => validateScenario(bad)).toThrow(ScenarioValidationError);
    const empty = load();
    ((empty.tori as Record<string, unknown>).body as Record<string, unknown>).keys = [];
    expect(() => validateScenario(empty)).toThrow(/al menos una postura/);
  });

  it("Tori es opcional: con `enabled: false` el modelo no tiene sus cuerpos", () => {
    const on = buildScenarioModel(validateScenario(load()));
    expect(on.bodies.filter((b) => b.startsWith("tori_"))).toHaveLength(TORI_SEGMENTS.length);
    expect(on.bodies[1]).toBe("pelvis"); // Uke sigue siendo el primer cuerpo
    const json = load();
    ((json.tori as Record<string, unknown>).body as Record<string, unknown>).enabled = false;
    const off = buildScenarioModel(validateScenario(json));
    expect(off.bodies.some((b) => b.startsWith("tori_"))).toBe(false);
    expect(off.toriBody).toBeUndefined();
  });

  it("Uke rueda sobre la espalda de Tori, no golpea con la cabeza y Tori es viable", async () => {
    const recording = await runScenario(validateScenario(load()), { engineVersion: "test" });
    const report = describeTrial(recording).join("\n");
    expect(recording.summary.outcome).toBe("fall");
    expect(recording.summary.max_tilt_deg).toBeGreaterThan(150);
    expect(report).toMatch(/Cabeza: no toca el suelo/);
    expect(report).toMatch(/Cuerpo de Tori: cargan trunk/);
    expect(report).toMatch(/Tori \(75 kg\): viable/);
    expect(report).not.toMatch(/energía mecánica/);
    const body = recording.peaks.tori_body!;
    expect(body.segments.trunk!.contact_time_s).toBeGreaterThan(0.15);
    // Los brazos no chocan: nunca cargan a Uke.
    expect(body.segments.left_forearm!.contact_time_s).toBe(0);
    // Reproducible: dos ejecuciones dan la misma grabación.
    const again = await runScenario(validateScenario(load()), { engineVersion: "test" });
    expect(again.samples[again.samples.length - 1]!.qpos).toEqual(recording.samples[recording.samples.length - 1]!.qpos);
  });

  it("sin el cuerpo, las mismas manos no voltean a Uke sobre nada", async () => {
    const json = load();
    ((json.tori as Record<string, unknown>).body as Record<string, unknown>).enabled = false;
    delete json.expect;
    const recording = await runScenario(validateScenario(json), { engineVersion: "test" });
    expect(recording.peaks.tori_body).toBeUndefined();
    expect(recording.summary.max_tilt_deg).toBeLessThan(150);
  });
});
