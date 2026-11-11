/**
 * Lectura de `hpl.recording.v1` (`.json` o `.json.gz`) en navegador y en Node. Valida lo mínimo que
 * necesita el reproductor; no recalcula nada. Sin dependencias: gzip con `DecompressionStream`
 * (navegador y Node >= 18) y, si no existe, `zlib` de Node.
 */
import { RECORDING_SCHEMA_ID, type Recording } from "./schema";

export type RecordingInput = string | ArrayBuffer | Uint8Array;

function isGzip(bytes: Uint8Array): boolean {
  return bytes.length >= 2 && bytes[0] === 0x1f && bytes[1] === 0x8b;
}

async function gunzip(bytes: Uint8Array): Promise<Uint8Array> {
  if (typeof DecompressionStream !== "undefined") {
    const stream = new Blob([bytes as BlobPart]).stream().pipeThrough(new DecompressionStream("gzip"));
    return new Uint8Array(await new Response(stream).arrayBuffer());
  }
  // Respaldo para entornos sin DecompressionStream (Node antiguo). El especificador se construye para que
  // el empaquetador del navegador no intente resolver `node:zlib`.
  const zlib = (await import(/* @vite-ignore */ "node:" + "zlib")) as typeof import("node:zlib");
  return new Uint8Array(zlib.gunzipSync(bytes));
}

function fail(path: string, reason: string): never {
  throw new Error(`Grabación inválida en ${path}: ${reason}`);
}

function isNumberArray(v: unknown): v is number[] {
  return Array.isArray(v) && v.every((x) => typeof x === "number" && Number.isFinite(x));
}

/** Comprueba la estructura mínima y devuelve el objeto tipado. */
export function validateRecording(json: unknown): Recording {
  if (typeof json !== "object" || json === null || Array.isArray(json)) fail("(raíz)", "debe ser un objeto");
  const r = json as Record<string, unknown>;
  if (r.schema !== RECORDING_SCHEMA_ID) {
    throw new Error(
      `Versión de grabación desconocida: ${JSON.stringify(r.schema)} (esperado "${RECORDING_SCHEMA_ID}")`,
    );
  }
  const meta = r.meta as Record<string, unknown> | undefined;
  if (!meta || typeof meta.scenario_id !== "string") fail("meta.scenario_id", "debe ser una cadena");
  if (typeof meta.record_hz !== "number" || !(meta.record_hz > 0)) fail("meta.record_hz", "debe ser > 0");
  const model = r.model as Record<string, unknown> | undefined;
  if (!model || typeof model.mjcf !== "string" || model.mjcf.length === 0) fail("model.mjcf", "debe ser texto MJCF");
  if (!Array.isArray(model.bodies) || !Array.isArray(model.joints)) fail("model", "faltan bodies o joints");
  const samples = r.samples;
  if (!Array.isArray(samples) || samples.length === 0) fail("samples", "debe ser una lista no vacía");
  let nq = -1;
  let last = -Infinity;
  samples.forEach((raw, i) => {
    const s = raw as Record<string, unknown>;
    if (typeof s !== "object" || s === null) fail(`samples[${i}]`, "debe ser un objeto");
    if (typeof s.t !== "number" || !Number.isFinite(s.t)) fail(`samples[${i}].t`, "debe ser un número finito");
    if (!(s.t > last)) fail(`samples[${i}].t`, "los tiempos deben ser estrictamente crecientes");
    last = s.t;
    if (!isNumberArray(s.qpos)) fail(`samples[${i}].qpos`, "debe ser una lista de números finitos");
    if (nq < 0) nq = s.qpos.length;
    else if (s.qpos.length !== nq) fail(`samples[${i}].qpos`, `longitud ${s.qpos.length}, esperada ${nq}`);
    if (!Array.isArray(s.effectors)) fail(`samples[${i}].effectors`, "debe ser una lista");
    if (!Array.isArray(s.contacts)) fail(`samples[${i}].contacts`, "debe ser una lista");
  });
  const summary = r.summary as Record<string, unknown> | undefined;
  if (!summary || typeof summary.outcome !== "string") fail("summary.outcome", "debe ser una cadena");
  return json as Recording;
}

/** Lee una grabación desde texto JSON, bytes de JSON o bytes gzip (se detecta por la cabecera `1f 8b`). */
export async function parseRecording(input: RecordingInput): Promise<Recording> {
  let text: string;
  if (typeof input === "string") {
    text = input;
  } else {
    let bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
    if (isGzip(bytes)) bytes = await gunzip(bytes);
    text = new TextDecoder("utf-8").decode(bytes);
  }
  if (text.charCodeAt(0) === 0xfeff) text = text.slice(1); // BOM
  let json: unknown;
  try {
    json = JSON.parse(text);
  } catch (error) {
    throw new Error("La grabación no es JSON válido: " + (error instanceof Error ? error.message : String(error)));
  }
  return validateRecording(json);
}
