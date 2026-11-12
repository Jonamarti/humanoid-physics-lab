/**
 * Galería de grabaciones de ejemplo (índice y descarga). Sin DOM, Three.js ni MuJoCo: la interfaz está en
 * `src/renderer/galleryView.ts` y la generación de los archivos en `src/recording/galleryBuild.ts` (solo Node).
 * Formato y uso: `docs/REPLAY.md`, sección «Galería de grabaciones de ejemplo».
 */

export const GALLERY_INDEX_SCHEMA_ID = "hpl.gallery.v1";

/** Carpeta (relativa a la base de Vite) y nombre del índice. */
export const GALLERY_DIR = "recordings";
export const GALLERY_INDEX_FILE = "index.json";

export const GALLERY_OUTCOMES = ["standing", "tilted", "fall", "stepped", "unresolved"] as const;
export const GALLERY_EXPECTS = ["pass", "fail", "none"] as const;

/** Identificador admitido: el mismo que el de un escenario y seguro dentro de una URL y de un nombre de archivo. */
export const GALLERY_ID_PATTERN = /^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$/;

export interface GalleryEntry {
  /** Id del escenario (y del archivo). */
  id: string;
  title: string;
  description: string;
  /** Archivo dentro de `recordings/`; siempre `<id>.json.gz`. */
  file: string;
  /** Duración del escenario (último instante grabado), en s. */
  duration_s: number;
  record_hz: number;
  samples: number;
  /** Veredicto de `summary.outcome`. */
  outcome: (typeof GALLERY_OUTCOMES)[number];
  /** Dirección de la caída en grados (`summary.direction_deg`), o `null`. */
  direction_deg: number | null;
  /** Resultado de la comparación con `expect` del escenario. */
  expect: (typeof GALLERY_EXPECTS)[number];
  /** `meta.engine_version` con la que se generó. */
  engine_version: string;
  /** Tamaño del `.json.gz` en bytes. */
  bytes: number;
}

export interface GalleryIndex {
  schema: typeof GALLERY_INDEX_SCHEMA_ID;
  recordings: GalleryEntry[];
}

/** Nombre de archivo de la grabación de un id. */
export function galleryFileName(id: string): string {
  return `${id}.json.gz`;
}

function fail(path: string, reason: string): never {
  throw new Error(`Índice de galería inválido en ${path}: ${reason}`);
}

function str(o: Record<string, unknown>, key: string, path: string): string {
  const v = o[key];
  if (typeof v !== "string" || v.trim().length === 0) fail(`${path}.${key}`, "debe ser un texto no vacío");
  return v;
}

function num(o: Record<string, unknown>, key: string, path: string, opts: { integer?: boolean; min: number }): number {
  const v = o[key];
  if (typeof v !== "number" || !Number.isFinite(v)) fail(`${path}.${key}`, "debe ser un número finito");
  if (opts.integer && !Number.isInteger(v)) fail(`${path}.${key}`, "debe ser un entero");
  if (v < opts.min) fail(`${path}.${key}`, `debe ser >= ${opts.min}`);
  return v;
}

function oneOf<T extends string>(o: Record<string, unknown>, key: string, path: string, allowed: readonly T[]): T {
  const v = o[key];
  if (typeof v !== "string" || !(allowed as readonly string[]).includes(v)) {
    fail(`${path}.${key}`, `debe ser uno de ${allowed.join(", ")}`);
  }
  return v as T;
}

/** Valida el índice `recordings/index.json`. Cualquier entrada mal formada lanza un error con su ruta. */
export function validateGalleryIndex(json: unknown): GalleryIndex {
  if (typeof json !== "object" || json === null || Array.isArray(json)) fail("(raíz)", "debe ser un objeto");
  const root = json as Record<string, unknown>;
  if (root.schema !== GALLERY_INDEX_SCHEMA_ID) {
    throw new Error(
      `Versión de índice de galería desconocida: ${JSON.stringify(root.schema)} (esperado "${GALLERY_INDEX_SCHEMA_ID}")`,
    );
  }
  if (!Array.isArray(root.recordings)) fail("recordings", "debe ser una lista");
  const seen = new Set<string>();
  const recordings = root.recordings.map((raw: unknown, i): GalleryEntry => {
    const path = `recordings[${i}]`;
    if (typeof raw !== "object" || raw === null || Array.isArray(raw)) fail(path, "debe ser un objeto");
    const o = raw as Record<string, unknown>;
    const id = str(o, "id", path);
    if (!GALLERY_ID_PATTERN.test(id)) fail(`${path}.id`, `"${id}" solo admite letras, cifras, «_» y «-»`);
    if (seen.has(id)) fail(`${path}.id`, `id repetido: ${id}`);
    seen.add(id);
    const file = str(o, "file", path);
    if (file !== galleryFileName(id)) fail(`${path}.file`, `debe ser "${galleryFileName(id)}", no "${file}"`);
    const direction = o.direction_deg;
    if (direction !== null && (typeof direction !== "number" || !Number.isFinite(direction))) {
      fail(`${path}.direction_deg`, "debe ser un número o null");
    }
    return {
      id,
      title: str(o, "title", path),
      description: str(o, "description", path),
      file,
      duration_s: num(o, "duration_s", path, { min: 0 }),
      record_hz: num(o, "record_hz", path, { min: 1 }),
      samples: num(o, "samples", path, { integer: true, min: 1 }),
      outcome: oneOf(o, "outcome", path, GALLERY_OUTCOMES),
      direction_deg: direction as number | null,
      expect: oneOf(o, "expect", path, GALLERY_EXPECTS),
      engine_version: str(o, "engine_version", path),
      bytes: num(o, "bytes", path, { integer: true, min: 1 }),
    };
  });
  return { schema: GALLERY_INDEX_SCHEMA_ID, recordings };
}

/**
 * URL de un archivo de la galería a partir de la base de Vite (`import.meta.env.BASE_URL`: `/humanoid-physics-lab/` en
 * Pages, `./` con una base relativa). El resultado es una ruta que `fetch` resuelve contra la página, de modo que
 * funciona bajo una subruta. `name` puede ser un archivo o vacío (carpeta).
 */
export function resolveGalleryUrl(base: string, name: string): string {
  if (/[\\]|(^|\/)\.\.(\/|$)/.test(name)) throw new Error(`Nombre de archivo de galería no válido: ${name}`);
  const root = base === "" ? "./" : base.endsWith("/") ? base : `${base}/`;
  return `${root}${GALLERY_DIR}/${name.replace(/^\.?\//, "")}`;
}

/** Id pedido por la URL (`?rec=<id>`), o `undefined`. Se acepta también una cadena con o sin `?`. */
export function recIdFromSearch(search: string): string | undefined {
  const id = new URLSearchParams(search).get("rec");
  return id === null || id.trim() === "" ? undefined : id.trim();
}

/** Enlace compartible: la URL actual con `?rec=<id>` (se conservan los demás parámetros y el hash). */
export function shareUrl(currentHref: string, id: string): string {
  const url = new URL(currentHref);
  url.searchParams.set("rec", id);
  return url.toString();
}

export type FetchLike = (url: string, init?: { cache?: "no-cache" | "default" }) => Promise<{
  ok: boolean; status: number; statusText: string; json(): Promise<unknown>; arrayBuffer(): Promise<ArrayBuffer>;
}>;

/** Mensaje comprensible para el usuario a partir de un fallo de red o de formato. */
function describeFailure(what: string, error: unknown): Error {
  const detail = error instanceof Error ? error.message : String(error);
  return new Error(`${what}: ${detail}`);
}

/** Descarga y valida `recordings/index.json`. Lanza `Error` con un mensaje en español si no existe o está mal. */
export async function fetchGalleryIndex(base: string, fetchFn: FetchLike): Promise<GalleryIndex> {
  const url = resolveGalleryUrl(base, GALLERY_INDEX_FILE);
  let json: unknown;
  try {
    const response = await fetchFn(url, { cache: "no-cache" });
    if (!response.ok) throw new Error(`HTTP ${response.status}${response.statusText ? " " + response.statusText : ""}`);
    try {
      json = await response.json();
    } catch {
      throw new Error("la respuesta no es JSON (¿falta recordings/index.json en esta publicación?)");
    }
  } catch (error) {
    throw describeFailure("No se pudo cargar el índice de grabaciones de ejemplo", error);
  }
  try {
    return validateGalleryIndex(json);
  } catch (error) {
    throw describeFailure("El índice de grabaciones de ejemplo no es válido", error);
  }
}

/** Descarga los bytes (`.json.gz`) de una entrada; se leen después con `parseRecording`. */
export async function fetchGalleryRecording(base: string, entry: GalleryEntry, fetchFn: FetchLike): Promise<ArrayBuffer> {
  const url = resolveGalleryUrl(base, entry.file);
  try {
    const response = await fetchFn(url);
    if (!response.ok) throw new Error(`HTTP ${response.status}${response.statusText ? " " + response.statusText : ""}`);
    return await response.arrayBuffer();
  } catch (error) {
    throw describeFailure(`No se pudo descargar la grabación «${entry.title}»`, error);
  }
}

const OUTCOME_TEXT: Readonly<Record<GalleryEntry["outcome"], string>> = {
  standing: "se mantiene en pie",
  tilted: "queda inclinado",
  fall: "cae",
  stepped: "da un paso y se mantiene estable",
  unresolved: "queda sin resolver",
};

/** Veredicto legible, por ejemplo «cae hacia 0°» o «se mantiene en pie». */
export function describeVerdict(entry: Pick<GalleryEntry, "outcome" | "direction_deg" | "expect">): string {
  const direction = entry.direction_deg === null ? "" : ` hacia ${Math.round(entry.direction_deg) || 0}°`;
  const expect = entry.expect === "pass" ? " · cumple lo esperado" : entry.expect === "fail" ? " · NO cumple lo esperado" : "";
  return `${OUTCOME_TEXT[entry.outcome]}${entry.outcome === "fall" || entry.outcome === "tilted" ? direction : ""}${expect}`;
}

/** Tamaño legible, por ejemplo «64 kB». */
export function formatBytes(bytes: number): string {
  return bytes >= 1_000_000 ? `${(bytes / 1_000_000).toFixed(2).replace(".", ",")} MB` : `${Math.round(bytes / 1000)} kB`;
}
