import { mkdirSync, mkdtempSync, readFileSync, readdirSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { gunzipSync } from "node:zlib";
import { afterAll, describe, expect, it } from "vitest";
import { BATCH_CSV_HEADER, EXIT_EXPECT_FAILED, EXIT_INVALID, EXIT_OK, runCli } from "../../src/scenario/cli";

/**
 * Prueba del comando `npm run sim` (función principal `runCli`, la misma que usa `scripts/sim.ts`).
 * El paso de CI que ejecuta `npm run sim` de verdad queda pendiente: tocaría `.github/workflows/`.
 */
const SCENARIOS = new URL("../../scenarios/", import.meta.url);
const LOW = new URL("cylinder_obstacle_low.json", SCENARIOS);

const tempRoot = mkdtempSync(join(tmpdir(), "hpl-sim-"));
afterAll(() => rmSync(tempRoot, { recursive: true, force: true }));

function capture(): { io: { out: (l: string) => void; err: (l: string) => void }; out: string[]; err: string[] } {
  const out: string[] = [];
  const err: string[] = [];
  return { io: { out: (l) => out.push(l), err: (l) => err.push(l) }, out, err };
}

function lowScenario(): Record<string, unknown> {
  return JSON.parse(readFileSync(LOW, "utf8")) as Record<string, unknown>;
}

describe("runCli (comando sim)", () => {
  it("un escenario correcto escribe recording.json.gz y summary.json, imprime el resumen y sale con 0", async () => {
    const outDir = join(tempRoot, "cyl_low");
    const log = capture();
    const code = await runCli([fileURLToPath(LOW), "--out", outDir], log.io);
    expect(code).toBe(EXIT_OK);
    expect(readdirSync(outDir).sort()).toEqual(["recording.json.gz", "summary.json"]);
    const recording = JSON.parse(gunzipSync(readFileSync(join(outDir, "recording.json.gz"))).toString("utf8")) as {
      schema: string; samples: unknown[]; summary: { outcome: string; expect: string };
    };
    expect(recording.schema).toBe("hpl.recording.v1");
    expect(recording.samples).toHaveLength(376); // 3 s a 125 Hz
    expect(recording.summary).toMatchObject({ outcome: "fall", expect: "pass" });
    const summary = JSON.parse(readFileSync(join(outDir, "summary.json"), "utf8")) as {
      scenario_id: string; summary: { outcome: string; expect: string }; recording: { bytes: number };
    };
    expect(summary.scenario_id).toBe("cylinder_obstacle_low");
    expect(summary.summary).toMatchObject({ outcome: "fall", expect: "pass" });
    expect(summary.recording.bytes).toBeGreaterThan(1000);
    expect(log.out.join("\n")).toContain("cylinder_obstacle_low: fall");
    expect(log.out.join("\n")).toContain("expect pass");
    expect(log.err).toEqual([]);
  });

  it("sale con 1 si no se cumple expect, e imprime la diferencia", async () => {
    const dir = join(tempRoot, "fail");
    mkdirSync(dir);
    const scenario = lowScenario();
    scenario.id = "must_fail";
    scenario.expect = { outcome: "standing" };
    writeFileSync(join(dir, "must_fail.json"), JSON.stringify(scenario));
    const log = capture();
    const code = await runCli([join(dir, "must_fail.json"), "--out", join(dir, "out")], log.io);
    expect(code).toBe(EXIT_EXPECT_FAILED);
    expect(log.out.join("\n")).toContain('resultado esperado "standing", obtenido "fall"');
    expect(readdirSync(join(dir, "out")).sort()).toEqual(["recording.json.gz", "summary.json"]);
  });

  it("sale con 0 si el escenario no tiene expect", async () => {
    const dir = join(tempRoot, "noexpect");
    mkdirSync(dir);
    const scenario = lowScenario();
    scenario.id = "no_expect";
    scenario.duration_s = 1;
    delete scenario.expect;
    // Con BOM (lo que escribe PowerShell 5.1 con Set-Content -Encoding utf8): debe aceptarse.
    writeFileSync(join(dir, "no_expect.json"), String.fromCharCode(0xfeff) + JSON.stringify(scenario));
    const log = capture();
    expect(await runCli([join(dir, "no_expect.json"), "--out", join(dir, "out")], log.io)).toBe(EXIT_OK);
    expect(log.out.join("\n")).toContain("expect none");
  });

  it("sale con 2 y escribe el motivo si el escenario es inválido o el uso es incorrecto", async () => {
    const dir = join(tempRoot, "invalid");
    mkdirSync(dir);
    const scenario = lowScenario();
    scenario.id = "bad_case";
    (scenario.ground as { friction: number }).friction = Number.NaN; // JSON.stringify lo escribe como null
    writeFileSync(join(dir, "bad_case.json"), JSON.stringify(scenario));
    writeFileSync(join(dir, "garbage.json"), "{ no es json");
    for (const [file, text] of [["bad_case.json", "ground.friction"], ["garbage.json", "no se pudo leer el JSON"]] as const) {
      const log = capture();
      expect(await runCli([join(dir, file), "--out", join(dir, "out")], log.io)).toBe(EXIT_INVALID);
      expect(log.err.join("\n")).toContain(text);
    }
    const missing = capture();
    expect(await runCli([join(dir, "no_existe.json")], missing.io)).toBe(EXIT_INVALID);
    const noArgs = capture();
    expect(await runCli([], noArgs.io)).toBe(EXIT_INVALID);
    expect(noArgs.err.join("\n")).toContain("Uso:");
    const unknown = capture();
    expect(await runCli(["--nope"], unknown.io)).toBe(EXIT_INVALID);
    expect(readdirSync(dir).includes("out")).toBe(false); // un escenario inválido no deja ficheros
  });

  it("--batch ejecuta todos los escenarios de scenarios/ y escribe batch.csv", async () => {
    const outDir = join(tempRoot, "batch");
    const log = capture();
    const code = await runCli(["--batch", fileURLToPath(SCENARIOS), "--out", outDir], log.io);
    expect(code).toBe(EXIT_OK);
    const csv = readFileSync(join(outDir, "batch.csv"), "utf8").trim().split("\n");
    expect(csv[0]).toBe(BATCH_CSV_HEADER);
    const ids = readdirSync(SCENARIOS).filter((f) => f.endsWith(".json")).map((f) => f.replace(/\.json$/, "")).sort();
    expect(csv.slice(1).map((r) => r.split(",")[0])).toEqual(ids);
    for (const row of csv.slice(1)) {
      const cells = row.split(",");
      expect(cells[1]).toBe("ok");
      expect(cells[9]).toBe("pass");
    }
    for (const id of ids) {
      expect(readdirSync(join(outDir, id)).sort()).toEqual(["recording.json.gz", "summary.json"]);
    }
    expect(log.out[log.out.length - 1]).toContain(`${ids.length}/${ids.length}`);
  }, 60000);

  it("--batch con un escenario inválido lo marca en la tabla y sale con 2", async () => {
    const dir = join(tempRoot, "batch_invalid");
    mkdirSync(dir);
    const ok = lowScenario();
    ok.id = "case_ok";
    ok.duration_s = 1;
    delete ok.expect;
    writeFileSync(join(dir, "case_ok.json"), JSON.stringify(ok));
    writeFileSync(join(dir, "case_bad.json"), JSON.stringify({ ...ok, id: "otro_nombre" }));
    const log = capture();
    const code = await runCli(["--batch", dir, "--out", join(dir, "out")], log.io);
    expect(code).toBe(EXIT_INVALID);
    const csv = readFileSync(join(dir, "out", "batch.csv"), "utf8").trim().split("\n");
    expect(csv).toHaveLength(3);
    expect(csv[1]).toMatch(/^case_bad,invalid,/);
    expect(csv[2]).toMatch(/^case_ok,ok,/);
  });
});
