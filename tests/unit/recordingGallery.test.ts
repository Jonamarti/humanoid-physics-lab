import { describe, expect, it } from "vitest";
import {
  describeVerdict, fetchGalleryIndex, fetchGalleryRecording, formatBytes, recIdFromSearch, resolveGalleryUrl, shareUrl,
  validateGalleryIndex, type FetchLike, type GalleryEntry,
} from "../../src/recording/gallery";
import { parseGalleryList } from "../../src/recording/galleryBuild";

/** Lógica pura de la galería (índice, URL con base, descarga con `fetch` simulado). La interfaz no tiene prueba de navegador. */

function entry(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    id: "cyl_a", title: "Cilindro", description: "Un cilindro.", file: "cyl_a.json.gz", duration_s: 3, record_hz: 125,
    samples: 376, outcome: "fall", direction_deg: 0, expect: "pass", engine_version: "0.1.0", bytes: 33853, ...overrides,
  };
}
const index = (...recordings: unknown[]): unknown => ({ schema: "hpl.gallery.v1", recordings });

describe("validateGalleryIndex", () => {
  it("acepta un índice correcto y una lista vacía", () => {
    expect(validateGalleryIndex(index(entry())).recordings[0]).toMatchObject({ id: "cyl_a", outcome: "fall", bytes: 33853 });
    expect(validateGalleryIndex(index(entry({ direction_deg: null }))).recordings[0]!.direction_deg).toBeNull();
    expect(validateGalleryIndex(index()).recordings).toEqual([]);
  });

  it.each([
    ["raíz no objeto", [], /\(raíz\).*objeto/],
    ["schema desconocido", { schema: "hpl.gallery.v9", recordings: [] }, /desconocida.*hpl\.gallery\.v9/],
    ["recordings no es lista", { schema: "hpl.gallery.v1", recordings: {} }, /recordings.*lista/],
    ["entrada no objeto", index(5), /recordings\[0\].*objeto/],
    ["id vacío", index(entry({ id: "" })), /recordings\[0\]\.id.*no vacío/],
    ["id con barra", index(entry({ id: "a/b", file: "a/b.json.gz" })), /recordings\[0\]\.id.*solo admite/],
    ["id con ..", index(entry({ id: "..", file: "...json.gz" })), /recordings\[0\]\.id/],
    ["id repetido", index(entry(), entry()), /recordings\[1\]\.id.*repetido/],
    ["título ausente", index(entry({ title: undefined })), /title.*no vacío/],
    ["descripción numérica", index(entry({ description: 4 })), /description/],
    ["archivo que no corresponde al id", index(entry({ file: "otro.json.gz" })), /file.*cyl_a\.json\.gz/],
    ["archivo con ruta", index(entry({ file: "../cyl_a.json.gz" })), /file/],
    ["duración negativa", index(entry({ duration_s: -1 })), /duration_s/],
    ["duración NaN", index(entry({ duration_s: Number.NaN })), /duration_s.*finito/],
    ["muestras no enteras", index(entry({ samples: 3.5 })), /samples.*entero/],
    ["veredicto desconocido", index(entry({ outcome: "roto" })), /outcome.*standing, tilted, fall/],
    ["expect desconocido", index(entry({ expect: "tal vez" })), /expect/],
    ["dirección de texto", index(entry({ direction_deg: "0" })), /direction_deg.*null/],
    ["tamaño cero", index(entry({ bytes: 0 })), /bytes/],
    ["versión del motor vacía", index(entry({ engine_version: " " })), /engine_version/],
  ])("rechaza %s con un mensaje", (_name, input, message) => {
    expect(() => validateGalleryIndex(input)).toThrow(message);
  });
});

describe("resolveGalleryUrl", () => {
  it("cuelga recordings/ de la base de Vite, también bajo una subruta", () => {
    expect(resolveGalleryUrl("/humanoid-physics-lab/", "index.json")).toBe("/humanoid-physics-lab/recordings/index.json");
    expect(resolveGalleryUrl("/humanoid-physics-lab", "a.json.gz")).toBe("/humanoid-physics-lab/recordings/a.json.gz");
    expect(resolveGalleryUrl("/", "index.json")).toBe("/recordings/index.json");
    expect(resolveGalleryUrl("./", "index.json")).toBe("./recordings/index.json");
    expect(resolveGalleryUrl("", "index.json")).toBe("./recordings/index.json");
    expect(resolveGalleryUrl("/base/", "./a.json.gz")).toBe("/base/recordings/a.json.gz");
  });

  it("rechaza nombres que salen de la carpeta", () => {
    expect(() => resolveGalleryUrl("/", "../secreto.json")).toThrow(/no válido/);
    expect(() => resolveGalleryUrl("/", "a\\b.json")).toThrow(/no válido/);
  });
});

describe("recIdFromSearch y shareUrl", () => {
  it("lee ?rec=<id>", () => {
    expect(recIdFromSearch("?rec=cylinder_obstacle_low")).toBe("cylinder_obstacle_low");
    expect(recIdFromSearch("rec=a&x=1")).toBe("a");
    expect(recIdFromSearch("?x=1&rec=%20b%20")).toBe("b");
  });
  it("ignora la ausencia o un valor vacío", () => {
    expect(recIdFromSearch("")).toBeUndefined();
    expect(recIdFromSearch("?x=1")).toBeUndefined();
    expect(recIdFromSearch("?rec=")).toBeUndefined();
  });
  it("construye el enlace conservando la subruta y los demás parámetros", () => {
    expect(shareUrl("https://jonamarti.github.io/humanoid-physics-lab/", "a_b"))
      .toBe("https://jonamarti.github.io/humanoid-physics-lab/?rec=a_b");
    expect(shareUrl("https://x.test/base/?rec=viejo&q=1#h", "nuevo")).toBe("https://x.test/base/?rec=nuevo&q=1#h");
  });
});

describe("describeVerdict y formatBytes", () => {
  it("describe el veredicto en español", () => {
    expect(describeVerdict({ outcome: "fall", direction_deg: -5e-7, expect: "pass" })).toBe("cae hacia 0° · cumple lo esperado");
    expect(describeVerdict({ outcome: "standing", direction_deg: null, expect: "none" })).toBe("se mantiene en pie");
    expect(describeVerdict({ outcome: "tilted", direction_deg: 90, expect: "fail" })).toBe("queda inclinado hacia 90° · NO cumple lo esperado");
  });
  it("da el tamaño en kB o MB", () => {
    expect(formatBytes(33853)).toBe("34 kB");
    expect(formatBytes(1_250_000)).toBe("1,25 MB");
  });
});

function response(init: { ok?: boolean; status?: number; json?: unknown; bytes?: ArrayBuffer; jsonError?: boolean }): ReturnType<FetchLike> {
  return Promise.resolve({
    ok: init.ok ?? true, status: init.status ?? 200, statusText: "",
    json: () => (init.jsonError ? Promise.reject(new SyntaxError("Unexpected token <")) : Promise.resolve(init.json)),
    arrayBuffer: () => Promise.resolve(init.bytes ?? new ArrayBuffer(0)),
  });
}

describe("fetchGalleryIndex y fetchGalleryRecording", () => {
  it("pide el índice bajo la base y lo valida", async () => {
    const urls: string[] = [];
    const fetchFn: FetchLike = (url) => { urls.push(url); return response({ json: index(entry()) }); };
    const result = await fetchGalleryIndex("/humanoid-physics-lab/", fetchFn);
    expect(urls).toEqual(["/humanoid-physics-lab/recordings/index.json"]);
    expect(result.recordings.map((r) => r.id)).toEqual(["cyl_a"]);
  });

  it("si el índice no existe (404) lanza un mensaje claro", async () => {
    await expect(fetchGalleryIndex("/", () => response({ ok: false, status: 404 })))
      .rejects.toThrow(/No se pudo cargar el índice de grabaciones de ejemplo: HTTP 404/);
  });

  it("si la red falla o la respuesta no es JSON, lanza un mensaje claro", async () => {
    await expect(fetchGalleryIndex("/", () => Promise.reject(new TypeError("Failed to fetch"))))
      .rejects.toThrow(/No se pudo cargar el índice.*Failed to fetch/);
    await expect(fetchGalleryIndex("/", () => response({ jsonError: true })))
      .rejects.toThrow(/No se pudo cargar el índice.*no es JSON/);
  });

  it("si el índice está mal formado, lo dice", async () => {
    await expect(fetchGalleryIndex("/", () => response({ json: index(entry({ outcome: "roto" })) })))
      .rejects.toThrow(/El índice de grabaciones de ejemplo no es válido.*outcome/);
  });

  it("descarga los bytes de una grabación y avisa si falla", async () => {
    const e = validateGalleryIndex(index(entry())).recordings[0] as GalleryEntry;
    const bytes = new Uint8Array([0x1f, 0x8b, 8]).buffer;
    const urls: string[] = [];
    const ok: FetchLike = (url) => { urls.push(url); return response({ bytes }); };
    expect(await fetchGalleryRecording("/b/", e, ok)).toBe(bytes);
    expect(urls).toEqual(["/b/recordings/cyl_a.json.gz"]);
    await expect(fetchGalleryRecording("/b/", e, () => response({ ok: false, status: 500 })))
      .rejects.toThrow(/No se pudo descargar la grabación «Cilindro»: HTTP 500/);
  });
});

describe("parseGalleryList (scenarios/gallery/gallery.json)", () => {
  const list = (...recordings: unknown[]): unknown => ({ schema: "hpl.gallery-list.v1", recordings });
  const item = (o: Record<string, unknown> = {}): Record<string, unknown> => ({ id: "a", title: "A", description: "Una.", ...o });

  it("acepta una lista correcta, con record_hz opcional", () => {
    expect(parseGalleryList(list(item(), item({ id: "b", record_hz: 50 })))).toEqual([
      { id: "a", title: "A", description: "Una." }, { id: "b", title: "A", description: "Una.", record_hz: 50 },
    ]);
  });
  it.each([
    ["schema distinto", { schema: "x", recordings: [item()] }, /schema/],
    ["lista vacía", list(), /no vacía/],
    ["id repetido", list(item(), item()), /repetido/],
    ["id con barra", list(item({ id: "a/b" })), /id válido/],
    ["sin título", list(item({ title: "" })), /title/],
    ["record_hz no entero", list(item({ record_hz: 62.5 })), /record_hz/],
  ])("rechaza %s", (_name, input, message) => {
    expect(() => parseGalleryList(input)).toThrow(message);
  });
});
