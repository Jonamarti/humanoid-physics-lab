import { readdirSync, readFileSync } from "node:fs";
import { gunzipSync } from "node:zlib";
import { describe, expect, it } from "vitest";
import { GALLERY_INDEX_FILE, validateGalleryIndex, type GalleryEntry } from "../../src/recording/gallery";
import { buildGalleryRecording, DEFAULT_GALLERY_LIST, readGalleryList, serializeGalleryIndex } from "../../src/recording/galleryBuild";
import { parseRecording } from "../../src/recording/read";
import { roundDeep } from "../../src/recording/write";

/**
 * Las grabaciones confirmadas en `public/recordings/` (galería de la web) son las de referencia. Estas pruebas comprueban
 * que se leen con el lector de la web, que coinciden con su entrada de `index.json` y con `scenarios/gallery/gallery.json`, y
 * que no están obsoletas (se regeneran dos y se comparan). Para actualizarlas: `npm run gallery` (ver docs/REPLAY.md).
 */
const DIR = new URL("../../public/recordings/", import.meta.url);
const read = (name: string): Buffer => readFileSync(new URL(name, DIR));
const index = validateGalleryIndex(JSON.parse(read(GALLERY_INDEX_FILE).toString("utf8")) as unknown);
const list = readGalleryList(DEFAULT_GALLERY_LIST);

/**
 * Presupuesto del conjunto (se descarga cada grabación solo al elegirla). Era de 1,25 MB; sube a 2 MB al añadir las cuatro
 * secuencias de Tori guionizado con el humanoide simulado, que pesan unos 150 kB cada una.
 */
const MAX_TOTAL_BYTES = 2_000_000;

describe("galería publicada: public/recordings/", () => {
  it("el índice enumera exactamente la lista de gallery.json, en su orden y con sus textos", () => {
    expect(index.recordings.map((r) => r.id)).toEqual(list.map((l) => l.id));
    for (const [i, item] of list.entries()) {
      expect(index.recordings[i]).toMatchObject({ id: item.id, title: item.title, description: item.description });
    }
  });

  it("el índice está escrito en su forma canónica y no sobran ni faltan archivos", () => {
    // En Windows con autocrlf el índice puede estar con CRLF en el árbol de trabajo.
    expect(read(GALLERY_INDEX_FILE).toString("utf8").split("\u000d\u000a").join("\u000a")).toBe(serializeGalleryIndex(index));
    expect(readdirSync(DIR).sort()).toEqual([GALLERY_INDEX_FILE, ...index.recordings.map((r) => r.file)].sort());
  });

  it(`el conjunto pesa como mucho ${MAX_TOTAL_BYTES} bytes`, () => {
    const total = index.recordings.reduce((sum, r) => sum + r.bytes, 0);
    expect(total).toBeLessThanOrEqual(MAX_TOTAL_BYTES);
  });

  for (const e of index.recordings) {
    it(`${e.id}: se lee con el lector existente y coincide con su entrada`, async () => {
      const gz = read(e.file);
      expect(gz.length).toBe(e.bytes);
      const recording = await parseRecording(gz);
      const summary = roundDeep(recording.summary);
      expect(recording.meta.scenario_id).toBe(e.id);
      expect(recording.meta.engine_version).toBe(e.engine_version);
      expect(recording.meta.record_hz).toBe(e.record_hz);
      expect(recording.samples).toHaveLength(e.samples);
      expect(recording.samples[recording.samples.length - 1]!.t).toBeCloseTo(e.duration_s, 9);
      expect(summary.outcome).toBe(e.outcome);
      expect(summary.direction_deg).toBe(e.direction_deg);
      expect(summary.expect).toBe(e.expect);
    });
  }
});

type Json = null | boolean | number | string | Json[] | { [k: string]: Json };

/** Diferencia numérica máxima admitida entre un entorno y otro (absoluta + relativa). Solo se usa si los bytes difieren. */
const ABS_TOL = 1e-4;
const REL_TOL = 1e-3;

function closeEnough(a: Json, b: Json, path: string, problems: string[]): void {
  if (problems.length > 5) return;
  if (typeof a === "number" && typeof b === "number") {
    if (Math.abs(a - b) > ABS_TOL + REL_TOL * Math.max(Math.abs(a), Math.abs(b))) problems.push(`${path}: ${a} frente a ${b}`);
  } else if (Array.isArray(a) && Array.isArray(b)) {
    if (a.length !== b.length) problems.push(`${path}: longitud ${a.length} frente a ${b.length}`);
    else a.forEach((x, i) => closeEnough(x, b[i]!, `${path}[${i}]`, problems));
  } else if (typeof a === "object" && typeof b === "object" && a !== null && b !== null && !Array.isArray(a) && !Array.isArray(b)) {
    for (const key of new Set([...Object.keys(a), ...Object.keys(b)])) {
      closeEnough(a[key] ?? null, b[key] ?? null, `${path}.${key}`, problems);
    }
  } else if (a !== b) {
    problems.push(`${path}: ${JSON.stringify(a)} frente a ${JSON.stringify(b)}`);
  }
}

/**
 * Compara la grabación regenerada con la confirmada. Primero, bytes idénticos (lo habitual en el mismo entorno). Si difieren
 * (otra versión de Node/zlib o de la plataforma), se comparan los JSON descomprimidos con tolerancia numérica; los contactos
 * y `qvel` no se comparan elemento a elemento (un contacto puede cruzar el umbral de 1e-6 N en otro entorno).
 */
function expectSameRecording(regenerated: Buffer, committed: Buffer): void {
  if (regenerated.equals(committed)) return;
  const strip = (gz: Buffer): Json => {
    const json = JSON.parse(gunzipSync(gz).toString("utf8")) as { samples: { contacts: unknown[]; qvel: unknown }[] } & Record<string, Json>;
    json.samples = json.samples.map((s) => ({ ...s, contacts: [s.contacts.length], qvel: [] })) as never;
    return json as unknown as Json;
  };
  const problems: string[] = [];
  closeEnough(strip(regenerated), strip(committed), "grabación", problems);
  expect(problems, "La galería está obsoleta: ejecuta `npm run gallery` y confirma public/recordings/").toEqual([]);
}

describe("galería no obsoleta (se regenera y se compara con la confirmada)", () => {
  const byId = (id: string): { entry: GalleryEntry; item: (typeof list)[number] } => ({
    entry: index.recordings.find((r) => r.id === id)!, item: list.find((l) => l.id === id)!,
  });

  it("cylinder_obstacle_low: mismo escenario, mismos bytes; dos ejecuciones seguidas coinciden", async () => {
    const { entry, item } = byId("cylinder_obstacle_low");
    const first = await buildGalleryRecording(item);
    const second = await buildGalleryRecording(item);
    expect(second.gz.equals(first.gz), "el ejecutor no es determinista dentro del mismo proceso").toBe(true);
    expect(first.entry).toMatchObject({ ...entry, bytes: expect.any(Number) as number });
    expectSameRecording(first.gz, read(entry.file));
  });

  it("uke_tense_75_175_push_sternum (humanoide): coincide con la confirmada", async () => {
    const { entry, item } = byId("uke_tense_75_175_push_sternum");
    const built = await buildGalleryRecording(item);
    expect(built.entry).toMatchObject({ ...entry, bytes: expect.any(Number) as number });
    expectSameRecording(built.gz, read(entry.file));
  }, 60_000);
});
