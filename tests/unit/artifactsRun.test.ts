import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import type { Scenario } from "../../src/scenario/schema";
import { validateScenario } from "../../src/scenario/validate";

/** El ejecutor con el detector de artefactos conectado (MuJoCo WASM real, escenarios de `scenarios/`). */
function load(id: string, edit: (raw: Record<string, unknown>) => void = () => undefined): Scenario {
  const raw = JSON.parse(readFileSync(new URL(`../../scenarios/${id}.json`, import.meta.url), "utf8")) as Record<string, unknown>;
  edit(raw);
  return validateScenario(raw, { fileName: `${id}.json` });
}

describe("runScenario + detector de artefactos", () => {
  it("humanoide de pie 3 s: limpio y con todos los detectores evaluados; mide con paso físico (no con muestras)", async () => {
    const rec = await runScenario(load("uke_stand_75_175_neutral", (r) => { r.duration_s = 3; r.record_hz = 125; }), { engineVersion: "test" });
    expect(rec.summary.artifacts).toEqual([]);
    expect(rec.summary.artifacts_not_evaluated).toEqual([]);
    const d = rec.diagnostics!;
    expect(d.actor_mass_kg).toBeCloseTo(75, 1);
    expect(d.root_generalized_force).toBe(0);
    expect(d.ground_penetration.value).toBeGreaterThan(0); // el contacto es blando: hay penetración, pero < 1 cm
    expect(d.ground_penetration.value).toBeLessThan(0.01);
    expect(d.contact_force.value).toBeGreaterThan(75 * 9.81 * 0.1);
    expect(d.energy!.scale_j).toBeGreaterThan(500);
    expect(d.tremor!.value).toBeLessThanOrEqual(10);
    // Lo que cuenta es lo medido en cada paso: el pico es al menos el de las muestras (cota inferior, no igualdad).
    const sampled = Math.max(...rec.samples.flatMap((s) => s.qvel.slice(6).map(Math.abs)));
    expect(d.joint_speed!.value).toBeGreaterThanOrEqual(sampled - 1e-9);
  }, 60000);

  it("cilindro que cae contra el suelo: dispara fuerza de contacto y penetración en el suelo (hallazgo actual; ver docs/ARTIFACTS.md)", async () => {
    const rec = await runScenario(load("cylinder_obstacle_low"), { engineVersion: "test" });
    const types = rec.summary.artifacts!.map((a) => a.type).sort();
    expect(types).toEqual(["contact_force", "ground_penetration"]);
    const cf = rec.summary.artifacts!.find((a) => a.type === "contact_force")!;
    expect(cf.value).toBeGreaterThan(cf.threshold);
    expect(cf.where).toBe("cylinder|world");
    // El cilindro no tiene articulaciones, pies ni segmentos: eso se dice, no se da por limpio.
    const skipped = rec.summary.artifacts_not_evaluated!.map((n) => n.type);
    expect(skipped).toEqual(expect.arrayContaining(["joint_speed", "tremor", "foot_slip", "segment_penetration", "root_force", "effector_over_limit"]));
    // El tope no declarado de `pull` se reporta con su id.
    expect(rec.summary.artifacts_not_evaluated!.find((n) => n.type === "effector_over_limit")!.reason).toContain("pull");
  }, 60000);

  it("un efector que declara tope (max_force_n) pasa a evaluarse: 300 N sobre una fuerza de ~418 N dispara y 500 N no", async () => {
    const withCap = (cap: number): Scenario => {
      const sc = load("cylinder_no_obstacle");
      return { ...sc, effectors: sc.effectors.map((e) => ({ ...e, max_force_n: cap })) as Scenario["effectors"] };
    };
    const hit = await runScenario(withCap(300), { engineVersion: "test" });
    const finding = hit.summary.artifacts!.find((a) => a.type === "effector_over_limit")!;
    expect(finding).toMatchObject({ where: "pull", threshold: 300 });
    expect(finding.value).toBeCloseTo(Math.hypot(400, 120), 3);
    expect(finding.t_s).toBeGreaterThanOrEqual(0.3);
    expect(finding.t_s).toBeLessThanOrEqual(0.45);
    const ok = await runScenario(withCap(500), { engineVersion: "test" });
    expect(ok.summary.artifacts!.find((a) => a.type === "effector_over_limit")).toBeUndefined();
    expect(ok.summary.artifacts_not_evaluated!.find((n) => n.type === "effector_over_limit")).toBeUndefined();
  }, 60000);

  it("es determinista: dos ejecuciones dan los mismos diagnósticos y artefactos", async () => {
    const sc = load("uke_relaxed_75_175_collapse", (r) => { r.duration_s = 1.5; });
    const a = await runScenario(sc, { engineVersion: "test" });
    const b = await runScenario(sc, { engineVersion: "test" });
    expect(b.diagnostics).toEqual(a.diagnostics);
    expect(b.summary.artifacts).toEqual(a.summary.artifacts);
  }, 60000);
});
