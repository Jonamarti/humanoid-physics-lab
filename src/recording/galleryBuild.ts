/**
 * Generación de la galería de grabaciones de ejemplo (solo Node). Lee la lista `scenarios/gallery/gallery.json`, ejecuta cada
 * escenario con `runScenario` y escribe `public/recordings/<id>.json.gz` y `public/recordings/index.json`.
 * Determinista: sin marcas de tiempo ni rutas en los archivos; mismo escenario y mismo motor, mismos bytes.
 * Se ejecuta con `npm run gallery` (`scripts/build_gallery.ts`). Detalle en `docs/REPLAY.md`.
 */
import { existsSync, mkdirSync, readdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { join } from "node:path";
import { readEngineVersion } from "../scenario/engineVersion";
import { runScenario } from "../scenario/run";
import { validateScenario } from "../scenario/validate";
import {
  GALLERY_ID_PATTERN, GALLERY_INDEX_FILE, GALLERY_INDEX_SCHEMA_ID, galleryFileName, type GalleryEntry, type GalleryIndex,
  validateGalleryIndex,
} from "./gallery";
import type { Recording } from "./schema";
import { gzipRecording, roundDeep } from "./write";

export const GALLERY_LIST_SCHEMA_ID = "hpl.gallery-list.v1";

/** Rutas por defecto, relativas a la raíz del repositorio. */
export const DEFAULT_GALLERY_LIST = join("scenarios", "gallery", "gallery.json");
export const DEFAULT_SCENARIOS_DIR = "scenarios";
export const DEFAULT_GALLERY_OUT = join("public", "recordings");
/** Copia publicada de los escenarios de la galería (`<id>.json`), para el editor de la web. */
export const DEFAULT_GALLERY_SCENARIOS_OUT = join("public", "scenarios");

/** Entrada de `gallery.json`. */
export interface GalleryListItem {
  /** Id de un escenario de `scenarios/<id>.json`. */
  id: string;
  title: string;
  /** Una o dos frases cortas en español. */
  description: string;
  /** Solo para la copia de la galería: sustituye a `record_hz` del escenario (debe dividir 500). Sirve para abaratar una grabación. */
  record_hz?: number;
}

export function parseGalleryList(json: unknown, where = "gallery.json"): GalleryListItem[] {
  const bad = (reason: string): never => { throw new Error(`${where}: ${reason}`); };
  if (typeof json !== "object" || json === null || Array.isArray(json)) bad("debe ser un objeto");
  const root = json as Record<string, unknown>;
  if (root.schema !== GALLERY_LIST_SCHEMA_ID) bad(`schema debe ser "${GALLERY_LIST_SCHEMA_ID}"`);
  if (!Array.isArray(root.recordings) || root.recordings.length === 0) bad("recordings debe ser una lista no vacía");
  const seen = new Set<string>();
  return (root.recordings as unknown[]).map((raw, i) => {
    if (typeof raw !== "object" || raw === null || Array.isArray(raw)) bad(`recordings[${i}] debe ser un objeto`);
    const o = raw as Record<string, unknown>;
    const id = o.id;
    if (typeof id !== "string" || !GALLERY_ID_PATTERN.test(id)) return bad(`recordings[${i}].id no es un id válido`);
    if (seen.has(id)) bad(`recordings[${i}].id repetido: ${id}`);
    seen.add(id);
    for (const key of ["title", "description"] as const) {
      const v = o[key];
      if (typeof v !== "string" || v.trim() === "") bad(`recordings[${i}].${key} debe ser un texto no vacío`);
    }
    if (o.record_hz !== undefined && (typeof o.record_hz !== "number" || !Number.isInteger(o.record_hz))) {
      bad(`recordings[${i}].record_hz debe ser un entero`);
    }
    const item: GalleryListItem = { id, title: o.title as string, description: o.description as string };
    if (o.record_hz !== undefined) item.record_hz = o.record_hz as number;
    return item;
  });
}

export function readGalleryList(path: string): GalleryListItem[] {
  const text = readFileSync(path, "utf8");
  return parseGalleryList(JSON.parse(text.charCodeAt(0) === 0xfeff ? text.slice(1) : text) as unknown, path);
}

export interface BuiltRecording {
  entry: GalleryEntry;
  gz: Buffer;
  recording: Recording;
  /** Escenario tal como lo escribió su autor (sin los campos que rellena el validador), con `record_hz` de la galería. */
  source: Record<string, unknown>;
}

/** Ejecuta el escenario de un elemento de la lista y devuelve la grabación comprimida y su entrada del índice. */
export async function buildGalleryRecording(item: GalleryListItem, scenariosDir = DEFAULT_SCENARIOS_DIR): Promise<BuiltRecording> {
  const file = join(scenariosDir, `${item.id}.json`);
  if (!existsSync(file)) throw new Error(`${item.id}: no existe el escenario ${file}`);
  const text = readFileSync(file, "utf8");
  const json = JSON.parse(text.charCodeAt(0) === 0xfeff ? text.slice(1) : text) as Record<string, unknown>;
  if (item.record_hz !== undefined) json.record_hz = item.record_hz;
  const scenario = validateScenario(json, { fileName: `${item.id}.json` });
  if (scenario.id !== item.id) throw new Error(`${item.id}: el escenario declara el id "${scenario.id}"`);
  const recording = await runScenario(scenario, { engineVersion: readEngineVersion() });
  const gz = gzipRecording(recording);
  const summary = roundDeep(recording.summary);
  const last = recording.samples[recording.samples.length - 1]!;
  const entry: GalleryEntry = {
    id: item.id,
    title: item.title,
    description: item.description,
    file: galleryFileName(item.id),
    duration_s: roundDeep(last.t),
    record_hz: recording.meta.record_hz,
    samples: recording.samples.length,
    outcome: summary.outcome,
    direction_deg: summary.direction_deg,
    expect: summary.expect,
    engine_version: recording.meta.engine_version,
    bytes: gz.length,
  };
  return { entry, gz, recording, source: json };
}

export function serializeGalleryIndex(index: GalleryIndex): string {
  return `${JSON.stringify(index, null, 2)}\n`;
}

export interface BuildGalleryOptions {
  listPath?: string;
  scenariosDir?: string;
  outDir?: string;
  /** Carpeta de los escenarios publicados. Por defecto `public/scenarios` solo si `outDir` es el de por defecto; si no, no se escriben. */
  scenariosOutDir?: string;
  log?: (line: string) => void;
}

/** Genera toda la galería. Borra de `outDir` los `.json.gz` que ya no estén en la lista. Devuelve el índice escrito. */
export async function buildGallery(options: BuildGalleryOptions = {}): Promise<GalleryIndex> {
  const outDir = options.outDir ?? DEFAULT_GALLERY_OUT;
  const log = options.log ?? (() => undefined);
  const items = readGalleryList(options.listPath ?? DEFAULT_GALLERY_LIST);
  const built: BuiltRecording[] = [];
  for (const item of items) {
    const result = await buildGalleryRecording(item, options.scenariosDir);
    built.push(result);
    log(`${item.id}: ${result.entry.outcome}, ${result.entry.samples} muestras, ${result.entry.bytes} bytes`);
  }
  const index = validateGalleryIndex({ schema: GALLERY_INDEX_SCHEMA_ID, recordings: built.map((b) => b.entry) });
  mkdirSync(outDir, { recursive: true });
  const keep = new Set(built.map((b) => b.entry.file));
  for (const name of readdirSync(outDir)) if (name.endsWith(".json.gz") && !keep.has(name)) rmSync(join(outDir, name));
  for (const b of built) writeFileSync(join(outDir, b.entry.file), b.gz);
  writeFileSync(join(outDir, GALLERY_INDEX_FILE), serializeGalleryIndex(index));
  const scenariosOut = options.scenariosOutDir ?? (options.outDir === undefined ? DEFAULT_GALLERY_SCENARIOS_OUT : undefined);
  if (scenariosOut !== undefined) {
    mkdirSync(scenariosOut, { recursive: true });
    const wanted = new Set(built.map((b) => `${b.entry.id}.json`));
    for (const name of readdirSync(scenariosOut)) if (name.endsWith(".json") && !wanted.has(name)) rmSync(join(scenariosOut, name));
    for (const b of built) writeFileSync(join(scenariosOut, `${b.entry.id}.json`), `${JSON.stringify(b.source, null, 1)}
`);
  }
  const total = built.reduce((sum, b) => sum + b.entry.bytes, 0);
  log(`galería: ${built.length} grabaciones, ${total} bytes en ${outDir}`);
  return index;
}
