/**
 * Interfaz de la galería de grabaciones de ejemplo: un selector «Grabaciones de ejemplo» que lee `recordings/index.json`
 * (junto al de la web, bajo la base de Vite) y reproduce la elegida con el mismo `ReplayView` que «Cargar grabación».
 * Admite `?rec=<id>` para abrir una directamente. Índice y descarga: `src/recording/gallery.ts` (ver `docs/REPLAY.md`).
 */
import {
  describeVerdict, fetchGalleryIndex, fetchGalleryRecording, formatBytes, recIdFromSearch, shareUrl,
  type FetchLike, type GalleryEntry, type GalleryIndex,
} from "../recording/gallery";

/** Lo mínimo que la galería necesita del reproductor (`ReplayView`). Devuelve `true` si la grabación se cargó. */
export interface GalleryReplayTarget {
  load(file: Blob): Promise<boolean>;
}

export interface GalleryViewOptions {
  replay: GalleryReplayTarget;
  /** Inserta el bloque de la galería en la interfaz (junto a «Cargar grabación»). */
  mount(element: HTMLElement): void;
  /** Base de Vite (`import.meta.env.BASE_URL`). */
  baseUrl: string;
  /** `?rec=...` inicial (`location.search`). */
  search: string;
  fetchFn?: FetchLike;
  /** Se llama cuando cambia el texto de estado de la galería (para repetirlo en el estado general). */
  onStatus?(text: string): void;
  /** Se llama cuando una grabación de la galería queda cargada en el reproductor. */
  onOpened?(entry: GalleryEntry): void;
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, props: Partial<HTMLElementTagNameMap[K]> = {}, text?: string): HTMLElementTagNameMap[K] {
  const node = Object.assign(document.createElement(tag), props);
  if (text !== undefined) node.textContent = text;
  return node;
}

const PLACEHOLDER = "Elige una grabación…";

export class GalleryView {
  private readonly root = el("section", { className: "gallery", ariaLabel: "Grabaciones de ejemplo" });
  private readonly label = el("label", { htmlFor: "gallery-select" }, "Grabaciones de ejemplo");
  private readonly select = el("select", { id: "gallery-select", disabled: true });
  private readonly message = el("p", { className: "muted gallery-message", role: "status" });
  private readonly detail = el("div", { className: "gallery-detail", hidden: true });
  private readonly title = el("strong", { className: "gallery-title" });
  private readonly description = el("p", { className: "muted" });
  private readonly verdict = el("p", { className: "gallery-verdict" });
  private readonly link = el("a", { className: "gallery-link" }, "Enlace para compartir");
  private index: GalleryIndex | undefined;
  /** Evita que una descarga lenta pise a otra más reciente. */
  private token = 0;
  /** `true` mientras el reproductor carga una grabación de la galería (su fallo también avisa de «modo cerrado»). */
  private loading = false;

  constructor(private readonly options: GalleryViewOptions) {
    this.select.append(el("option", { value: "" }, "Cargando lista…"));
    this.select.addEventListener("change", () => {
      const entry = this.index?.recordings.find((r) => r.id === this.select.value);
      if (entry) void this.open(entry);
    });
    this.link.rel = "noopener";
    this.detail.append(this.title, this.description, this.verdict, this.link);
    this.root.append(this.label, this.select, this.message, this.detail);
    options.mount(this.root);
    void this.init();
  }

  private get fetchFn(): FetchLike {
    return this.options.fetchFn ?? ((url, init) => fetch(url, init));
  }

  private say(text: string, isError = false): void {
    this.message.textContent = text;
    this.message.classList.toggle("is-error", isError);
    this.message.hidden = text === "";
    this.options.onStatus?.(text);
  }

  private async init(): Promise<void> {
    const wanted = recIdFromSearch(this.options.search);
    try {
      this.index = await fetchGalleryIndex(this.options.baseUrl, this.fetchFn);
    } catch (error) {
      this.select.replaceChildren(el("option", { value: "" }, "No disponible"));
      this.say(`${error instanceof Error ? error.message : String(error)}. El resto de la web funciona con normalidad.`, true);
      return;
    }
    const { recordings } = this.index;
    if (recordings.length === 0) {
      this.select.replaceChildren(el("option", { value: "" }, "Sin grabaciones"));
      this.say("La galería de grabaciones de ejemplo está vacía.", true);
      return;
    }
    this.select.replaceChildren(el("option", { value: "" }, PLACEHOLDER));
    for (const r of recordings) {
      this.select.append(el("option", { value: r.id }, `${r.title} (${formatBytes(r.bytes)})`));
    }
    this.select.disabled = false;
    this.say("");
    if (wanted !== undefined) {
      const entry = recordings.find((r) => r.id === wanted);
      if (entry) await this.open(entry);
      else this.say(`La grabación «${wanted}» del enlace no está en la galería; elige otra de la lista.`, true);
    }
  }

  /** Descarga y reproduce una entrada. */
  async open(entry: GalleryEntry): Promise<void> {
    const token = ++this.token;
    this.select.value = entry.id;
    this.showDetail(entry);
    this.say(`Descargando «${entry.title}» (${formatBytes(entry.bytes)})…`);
    let bytes: ArrayBuffer;
    try {
      bytes = await fetchGalleryRecording(this.options.baseUrl, entry, this.fetchFn);
    } catch (error) {
      if (token === this.token) this.say(error instanceof Error ? error.message : String(error), true);
      return;
    }
    if (token !== this.token) return;
    this.loading = true;
    let loaded: boolean;
    try {
      loaded = await this.options.replay.load(new Blob([bytes]));
    } finally {
      this.loading = false;
    }
    if (token !== this.token) return;
    if (loaded) {
      this.say("");
      this.updateAddress(entry.id);
      this.options.onOpened?.(entry);
    } else {
      this.say(`No se pudo reproducir «${entry.title}». Mira el mensaje de estado del visor.`, true);
    }
  }

  /** Para que `main.ts` restablezca el selector cuando se sale del modo reproducción (se ignora durante una carga propia). */
  reset(): void {
    if (this.loading) return;
    this.token++;
    this.select.value = "";
    this.detail.hidden = true;
    this.say("");
    try {
      const url = new URL(window.location.href);
      if (url.searchParams.has("rec")) {
        url.searchParams.delete("rec");
        window.history.replaceState(null, "", url);
      }
    } catch { /* sin historial: no importa */ }
  }

  private showDetail(entry: GalleryEntry): void {
    this.title.textContent = entry.title;
    this.description.textContent = entry.description;
    this.verdict.textContent = `Resultado: ${describeVerdict(entry)} · ${entry.duration_s.toFixed(1).replace(".", ",")} s`;
    this.link.href = shareUrl(window.location.href, entry.id);
    this.detail.hidden = false;
  }

  private updateAddress(id: string): void {
    try {
      window.history.replaceState(null, "", shareUrl(window.location.href, id));
    } catch { /* sin historial: el enlace compartible sigue disponible en pantalla */ }
  }
}
