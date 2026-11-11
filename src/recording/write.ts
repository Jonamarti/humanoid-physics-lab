/**
 * Serialización de `hpl.recording.v1`: números redondeados a 6 cifras significativas
 * (fichero estable y pequeño) y compresión gzip determinista (solo Node).
 */
import { gzipSync } from "node:zlib";
import type { Recording } from "./schema";

/** Cifras significativas de los números del fichero. */
export const RECORDING_SIGNIFICANT_DIGITS = 6;

/** Redondea a `digits` cifras significativas; `-0` pasa a `0`. */
export function roundSignificant(value: number, digits = RECORDING_SIGNIFICANT_DIGITS): number {
  if (!Number.isFinite(value)) throw new RangeError(`Número no finito en la grabación: ${value}`);
  if (value === 0) return 0;
  const rounded = Number(value.toPrecision(digits));
  return rounded === 0 ? 0 : rounded;
}

/** Copia profunda con todos los números redondeados (las cadenas, como el MJCF, no se tocan). */
export function roundDeep<T>(value: T, digits = RECORDING_SIGNIFICANT_DIGITS): T {
  if (typeof value === "number") return roundSignificant(value, digits) as T;
  if (Array.isArray(value)) return value.map((v) => roundDeep(v, digits)) as T;
  if (typeof value === "object" && value !== null) {
    const out: Record<string, unknown> = {};
    for (const [key, v] of Object.entries(value)) out[key] = roundDeep(v, digits);
    return out as T;
  }
  return value;
}

/** JSON de la grabación (compacto) con los números redondeados. */
export function serializeRecording(recording: Recording): string {
  return JSON.stringify(roundDeep(recording));
}

/** `recording.json.gz`: el JSON anterior comprimido con gzip nivel 9 (sin marca de tiempo: reproducible). */
export function gzipRecording(recording: Recording): Buffer {
  return gzipSync(Buffer.from(serializeRecording(recording), "utf8"), { level: 9 });
}
