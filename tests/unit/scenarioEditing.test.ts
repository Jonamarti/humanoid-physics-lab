import { createHash } from "node:crypto";
import { existsSync, readdirSync, readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { validateGalleryIndex } from "../../src/recording/gallery";
import { parseRecording } from "../../src/recording/read";
import { decodeScenarioHash, describeTrial, encodeScenarioHash, forceToPolar, polarToForce } from "../../src/scenario/editing";
import { runScenario, scenarioSha256 } from "../../src/scenario/run";
import { sha256Hex } from "../../src/scenario/sha256";
import { validateScenario } from "../../src/scenario/validate";

/** Piezas puras del editor de escenarios de la web y los escenarios publicados en `public/scenarios/`. */
describe("sha256 propio", () => {
  it("coincide con node:crypto en textos de varias longitudes, también con caracteres no ASCII", () => {
    for (const text of ["", "abc", "ñ€𝄞 " + "x".repeat(55), "a".repeat(56), "b".repeat(64), "c".repeat(1000)]) {
      expect(sha256Hex(text)).toBe(createHash("sha256").update(text).digest("hex"));
    }
  });
});

describe("fuerza en módulo y elevación", () => {
  it("convierte en los dos sentidos", () => {
    expect(forceToPolar([282.8, 0, 282.8])).toEqual({ magnitude_n: 399.9, elevation_deg: 45, lateral_n: 0 });
    expect(forceToPolar([-68.4, 12, -187.9])).toMatchObject({ elevation_deg: -110, lateral_n: 12 });
    expect(forceToPolar([0, 0, 0])).toEqual({ magnitude_n: 0, elevation_deg: 0, lateral_n: 0 });
    expect(polarToForce({ magnitude_n: 300, elevation_deg: -90, lateral_n: 5 })).toEqual([0, 5, -300]);
    expect(polarToForce({ magnitude_n: 200, elevation_deg: 180, lateral_n: 0 })).toEqual([-200, 0, 0]);
  });
});

describe("escenario dentro del enlace", () => {
  it("ida y vuelta, con acentos, y aviso si está dañado", () => {
    const scenario = { id: "prueba", nota: "tirón hacia atrás €", effectors: [{ force_n: [1, -2.5, 3] }] };
    const hash = encodeScenarioHash(scenario);
    expect(hash).toMatch(/^#scn=[A-Za-z0-9_-]+$/);
    expect(decodeScenarioHash(hash)).toEqual(scenario);
    expect(decodeScenarioHash("")).toBeUndefined();
    expect(decodeScenarioHash("#otra=1")).toBeUndefined();
    expect(() => decodeScenarioHash("#scn=%%%")).toThrow(/dañado/);
  });
});

describe("escenarios publicados para el editor: public/scenarios/", () => {
  const dir = new URL("../../public/scenarios/", import.meta.url);
  const recordings = new URL("../../public/recordings/", import.meta.url);
  const index = validateGalleryIndex(JSON.parse(readFileSync(new URL("index.json", recordings), "utf8")) as unknown);

  it("hay exactamente uno por grabación de la galería", () => {
    expect(readdirSync(dir).sort()).toEqual(index.recordings.map((r) => `${r.id}.json`).sort());
  });

  for (const entry of index.recordings) {
    it(`${entry.id}: se puede validar tal cual y es el escenario de su grabación`, async () => {
      const file = new URL(`${entry.id}.json`, dir);
      expect(existsSync(file)).toBe(true);
      const scenario = validateScenario(JSON.parse(readFileSync(file, "utf8")) as unknown, { fileName: `${entry.id}.json` });
      const recording = await parseRecording(readFileSync(new URL(entry.file, recordings)));
      expect(scenarioSha256(scenario)).toBe(recording.meta.scenario_sha256);
    });
  }
});

describe("informe de un ensayo", () => {
  it("resume el volteo en arco y distingue la cadera resbaladiza", async () => {
    const json = JSON.parse(readFileSync(new URL("../../public/scenarios/uke_tense_75_175_hip_roll.json", import.meta.url), "utf8")) as Record<string, unknown>;
    const good = describeTrial(await runScenario(validateScenario(json), { engineVersion: "test" })).join("\n");
    expect(good).toMatch(/Resultado: fall/);
    expect(good).toMatch(/Obstáculo hip_bar: toca .*pelvis/);
    expect(good).toMatch(/Tori \(75 kg\): viable/);
    expect(good).toMatch(/Fuerza hand_sleeve: máximo 400 N \(tope 500 N\)/);
    expect(good).not.toMatch(/fuerza de contacto/);

    (json.obstacles as Array<Record<string, unknown>>)[0]!.friction = 0.2;
    const slippery = describeTrial(await runScenario(validateScenario(json), { engineVersion: "test" })).join("\n");
    expect(slippery).toMatch(/Cabeza: golpea el suelo/);
    expect(slippery).toMatch(/Tori \(75 kg\): INVIABLE/);
    expect(slippery).toMatch(/Artefactos: .*fuerza de contacto/);
  }, 120_000);
});
