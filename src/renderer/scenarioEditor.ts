/**
 * Editor de escenarios de la web: abre el escenario de una grabación de la galería, deja cambiar rozamientos, cuerpos,
 * obstáculos y la curva de fuerzas de cada mano punto a punto, lo simula en el navegador con el mismo `runScenario` que
 * `npm run sim` y reproduce el resultado. El escenario editado viaja en el enlace (`#scn=...`). Ver `docs/SCENARIO_EDITOR.md`.
 */
import { loadPhysicsModule } from "../physics/mujoco";
import type { Recording } from "../recording/schema";
import { decodeScenarioHash, describeTrial, encodeScenarioHash, forceToPolar, polarToForce } from "../scenario/editing";
import type { MujocoModule } from "../scenario/mujocoModule";
import { runScenario } from "../scenario/run";
import { ScenarioValidationError, validateScenario } from "../scenario/validate";

export interface ScenarioEditorOptions {
  /** Inserta el bloque del editor en la interfaz. */
  mount(element: HTMLElement): void;
  /** Reproduce la grabación simulada. */
  show(recording: Recording): Promise<boolean>;
  /** Base de Vite, para descargar `scenarios/<id>.json`. */
  baseUrl: string;
  engineVersion?: string;
}

type Json = Record<string, unknown>;
type Vec = [number, number, number];

function el<K extends keyof HTMLElementTagNameMap>(tag: K, props: Partial<HTMLElementTagNameMap[K]> = {}, text?: string): HTMLElementTagNameMap[K] {
  const node = Object.assign(document.createElement(tag), props);
  if (text !== undefined) node.textContent = text;
  return node;
}

/** Valores por defecto que se enseñan cuando el escenario no declara el campo (los de los módulos de efectores). */
const DEFAULT_OBSTACLE_FRICTION = { cloth: 0.8, skin: 0.5 } as const;

export class ScenarioEditor {
  private readonly root = el("details", { className: "scenario-editor", hidden: true });
  private readonly form = el("div", { className: "scenario-form" });
  private readonly json = el("textarea", { className: "scenario-json", spellcheck: false, rows: 10 });
  private readonly message = el("p", { className: "muted scenario-message", role: "status" });
  private readonly report = el("ul", { className: "scenario-report" });
  private readonly runButton = el("button", { type: "button", className: "primary" }, "Simular con estos valores");
  private readonly link = el("a", { className: "gallery-link" }, "Enlace a este escenario editado");
  private draft: Json | undefined;
  private original: string | undefined;
  private running = false;

  constructor(private readonly options: ScenarioEditorOptions) {
    const reset = el("button", { type: "button" }, "Restablecer");
    const download = el("button", { type: "button" }, "Descargar JSON");
    const applyJson = el("button", { type: "button" }, "Aplicar el JSON al formulario");
    this.runButton.addEventListener("click", () => void this.simulate());
    reset.addEventListener("click", () => { if (this.original !== undefined) this.setDraft(JSON.parse(this.original) as Json); });
    download.addEventListener("click", () => this.download());
    applyJson.addEventListener("click", () => {
      try {
        this.setDraft(JSON.parse(this.json.value) as Json);
        this.say("JSON aplicado. Pulsa «Simular con estos valores».");
      } catch (error) {
        this.say("El JSON no se puede leer: " + (error instanceof Error ? error.message : String(error)), true);
      }
    });
    this.link.rel = "noopener";
    const buttons = el("div", { className: "button-row" });
    buttons.append(this.runButton, reset, download);
    const advanced = el("details", { className: "scenario-advanced" });
    advanced.append(el("summary", {}, "JSON completo del escenario (avanzado)"), this.json, applyJson);
    this.root.append(
      el("summary", {}, "Editar y simular este escenario"),
      el("p", { className: "muted" }, "Cambia valores y pulsa «Simular»: la física se calcula en tu navegador y el resultado sustituye a la reproducción. Nada se guarda en el servidor."),
      this.form, buttons, this.message, this.report, this.link, advanced,
    );
    options.mount(this.root);
  }

  private say(text: string, isError = false): void {
    this.message.textContent = text;
    this.message.classList.toggle("is-error", isError);
  }

  /** Abre el escenario publicado de una grabación de la galería. Si no existe, el editor se oculta. */
  async openGalleryScenario(id: string): Promise<void> {
    try {
      const root = this.options.baseUrl.endsWith("/") ? this.options.baseUrl : `${this.options.baseUrl}/`;
      const response = await fetch(`${root}scenarios/${id}.json`, { cache: "no-cache" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const scenario = await response.json() as Json;
      this.original = JSON.stringify(scenario);
      this.setDraft(scenario);
      this.report.replaceChildren();
      this.say("");
      this.root.hidden = false;
    } catch {
      this.hide();
    }
  }

  /** Si la URL lleva `#scn=...`, lo abre y lo simula. Devuelve `true` si había escenario en el enlace. */
  async openFromHash(hash: string): Promise<boolean> {
    let scenario: unknown;
    try {
      scenario = decodeScenarioHash(hash);
    } catch (error) {
      this.root.hidden = false;
      this.root.open = true;
      this.say(error instanceof Error ? error.message : String(error), true);
      return true;
    }
    if (scenario === undefined) return false;
    this.original = JSON.stringify(scenario);
    this.setDraft(scenario as Json);
    this.root.hidden = false;
    this.root.open = true;
    await this.simulate();
    return true;
  }

  hide(): void {
    this.root.hidden = true;
    this.draft = undefined;
  }

  private setDraft(scenario: Json): void {
    this.draft = scenario;
    this.render();
    this.sync();
  }

  /** Lleva el borrador al cuadro de JSON y al enlace. */
  private sync(): void {
    if (!this.draft) return;
    this.json.value = JSON.stringify(this.draft, null, 1);
    try {
      const url = new URL(window.location.href);
      url.searchParams.delete("rec");
      url.hash = encodeScenarioHash(this.draft);
      this.link.href = url.toString();
    } catch { /* sin URL: el enlace queda como estaba */ }
  }

  private download(): void {
    if (!this.draft) return;
    const blob = new Blob([JSON.stringify(this.draft, null, 2) + "\n"], { type: "application/json" });
    const a = el("a", { href: URL.createObjectURL(blob), download: `${String(this.draft.id ?? "escenario")}.json` });
    a.click();
    URL.revokeObjectURL(a.href);
  }

  async simulate(): Promise<void> {
    if (this.running) return;
    let scenario;
    try {
      scenario = validateScenario(JSON.parse(this.json.value) as unknown);
    } catch (error) {
      const where = error instanceof ScenarioValidationError ? `${error.path}: ${error.reason}` : error instanceof Error ? error.message : String(error);
      this.say("El escenario no es válido. " + where, true);
      return;
    }
    this.running = true;
    this.runButton.disabled = true;
    this.say("Simulando…");
    try {
      const mujoco = await loadPhysicsModule() as unknown as MujocoModule;
      // Un respiro para que el navegador pinte «Simulando…» antes de bloquear el hilo.
      await new Promise((resolve) => window.setTimeout(resolve, 20));
      const started = performance.now();
      const recording = await runScenario(scenario, { mujoco, engineVersion: this.options.engineVersion ?? "web" });
      const elapsed = (performance.now() - started) / 1000;
      this.report.replaceChildren(...describeTrial(recording).map((line) => el("li", {}, line)));
      await this.options.show(recording);
      this.say(`Simulado en ${elapsed.toFixed(1).replace(".", ",")} s. Abajo, el informe; arriba, la reproducción.`);
      try {
        window.history.replaceState(null, "", this.link.href);
      } catch { /* sin historial */ }
    } catch (error) {
      this.say("La simulación falló: " + (error instanceof Error ? error.message : String(error)), true);
    } finally {
      this.running = false;
      this.runButton.disabled = false;
    }
  }

  // --- Formulario -----------------------------------------------------------------------------------------------------

  private number(label: string, value: number | undefined, set: (v: number) => void, step = "any", placeholder = ""): HTMLLabelElement {
    const input = el("input", { type: "number", step, value: value === undefined ? "" : String(value), placeholder });
    input.addEventListener("change", () => {
      if (input.value.trim() === "") return;
      const v = Number(input.value);
      if (Number.isFinite(v)) { set(v); this.sync(); }
    });
    const wrap = el("label", {}, label);
    wrap.append(input);
    return wrap;
  }

  private group(title: string, hint?: string): HTMLFieldSetElement {
    const set = el("fieldset", {});
    set.append(el("legend", {}, title));
    if (hint) set.append(el("p", { className: "muted" }, hint));
    this.form.append(set);
    return set;
  }

  /** Tabla editable: una fila por elemento, con «+» para añadir al final y «×» para quitar. */
  private table<T>(
    parent: HTMLElement, headers: string[], rows: T[], cells: (row: T) => number[], write: (row: T, values: number[]) => void,
    make: (last: T | undefined) => T,
  ): void {
    const table = el("table", { className: "scenario-table" });
    const head = el("tr");
    for (const h of [...headers, ""]) head.append(el("th", {}, h));
    table.append(head);
    rows.forEach((row, index) => {
      const tr = el("tr");
      const values = cells(row);
      values.forEach((value, k) => {
        const input = el("input", { type: "number", step: "any", value: String(value), ariaLabel: `${headers[k]}, fila ${index + 1}` });
        input.addEventListener("change", () => {
          const v = Number(input.value);
          if (input.value.trim() === "" || !Number.isFinite(v)) return;
          values[k] = v;
          write(row, values);
          this.sync();
        });
        const td = el("td");
        td.append(input);
        tr.append(td);
      });
      const remove = el("button", { type: "button", ariaLabel: `Quitar la fila ${index + 1}` }, "×");
      remove.addEventListener("click", () => { rows.splice(index, 1); this.render(); this.sync(); });
      const td = el("td");
      td.append(remove);
      tr.append(td);
      table.append(tr);
    });
    const add = el("button", { type: "button" }, "+ Añadir fila");
    add.addEventListener("click", () => { rows.push(make(rows[rows.length - 1])); this.render(); this.sync(); });
    parent.append(table, add);
  }

  private phases(parent: HTMLElement, owner: Json, frameText: string): void {
    if (!Array.isArray(owner.phases)) owner.phases = [];
    const rows = owner.phases as Array<{ start_s: number; duration_s: number; offset_m: Vec }>;
    parent.append(el("p", { className: "muted" }, `Desplazamientos acumulados (${frameText}): en cada tramo el objetivo viaja hasta ese desplazamiento.`));
    this.table(parent, ["Inicio (s)", "Dura (s)", "Delante X (m)", "Izq. Y (m)", "Arriba Z (m)"], rows,
      (r) => [r.start_s, r.duration_s, ...r.offset_m],
      (r, v) => { r.start_s = v[0]!; r.duration_s = v[1]!; r.offset_m = [v[2]!, v[3]!, v[4]!]; },
      (last) => last
        ? { start_s: Number((last.start_s + last.duration_s).toFixed(3)), duration_s: last.duration_s, offset_m: [...last.offset_m] as Vec }
        : { start_s: 0.2, duration_s: 0.4, offset_m: [0, 0, 0] as Vec });
  }

  private render(): void {
    const d = this.draft;
    this.form.replaceChildren();
    if (!d) return;

    const general = this.group("General");
    general.append(this.number("Duración (s)", d.duration_s as number, (v) => { d.duration_s = v; }));
    const ground = (d.ground ?? (d.ground = {})) as Json;
    general.append(this.number("Rozamiento del suelo", ground.friction as number, (v) => { ground.friction = v; }, "0.05"));

    for (const actor of (d.actors ?? []) as Json[]) {
      if (actor.kind !== "humanoid") continue;
      const g = this.group(`Uke (${String(actor.id)})`);
      g.append(
        this.number("Masa (kg)", actor.mass_kg as number, (v) => { actor.mass_kg = v; }),
        this.number("Altura (m)", actor.height_m as number, (v) => { actor.height_m = v; }, "0.01"),
        this.number("Fuerza muscular (1 = normal)", actor.strength as number, (v) => { actor.strength = v; }, "0.1"),
      );
      const attitude = el("select");
      for (const a of ["tense", "relaxed"]) attitude.append(el("option", { value: a, selected: actor.attitude === a }, a === "tense" ? "en tensión" : "relajado"));
      attitude.addEventListener("change", () => { actor.attitude = attitude.value; this.sync(); });
      const wrap = el("label", {}, "Actitud");
      wrap.append(attitude);
      g.append(wrap);
    }

    if (d.tori) {
      const tori = d.tori as Json;
      const g = this.group("Tori (presupuesto)", "Tori no tiene cuerpo: es una masa con dos pies que sirve para juzgar si podría hacer estas fuerzas.");
      g.append(
        this.number("Masa (kg)", tori.mass_kg as number, (v) => { tori.mass_kg = v; }),
        this.number("Rozamiento de sus pies", tori.friction as number | undefined, (v) => { tori.friction = v; }, "0.05", `${String(ground.friction)} (el del suelo)`),
      );
    }

    for (const o of (d.obstacles ?? []) as Json[]) {
      const g = this.group(`Obstáculo «${String(o.id)}» (${String(o.shape)})`, "La cadera, la pierna o el pie de Tori. Coordenadas en el mundo: Uke empieza en X = 0 mirando hacia +X.");
      const start = o.start_m as Vec;
      const surface = (o.surface as keyof typeof DEFAULT_OBSTACLE_FRICTION | undefined) ?? "cloth";
      const tori = Number(o.tori_mass_kg);
      g.append(
        this.number("Rozamiento con Uke", o.friction as number | undefined, (v) => { o.friction = v; }, "0.05", String(DEFAULT_OBSTACLE_FRICTION[surface])),
        this.number("Distancia delante de Uke, X (m)", start[0], (v) => { start[0] = v; }, "0.01"),
        this.number("Altura del centro, Z (m)", start[2], (v) => { start[2] = v; }, "0.01"),
        this.number("Radio (m)", o.radius_m as number | undefined, (v) => { o.radius_m = v; }, "0.01", "0.11 (barra)"),
        this.number("Tope de fuerza horizontal (N)", o.max_force_n as number | undefined, (v) => { o.max_force_n = v; }, "10", (0.8 * tori * 9.81).toFixed(0)),
        this.number("Tope de fuerza vertical (N)", o.max_vertical_force_n as number | undefined, (v) => { o.max_vertical_force_n = v; }, "10", (2 * tori * 9.81).toFixed(0)),
      );
      this.phases(g, o, "en el mundo, respecto a la posición inicial");
    }

    for (const e of (d.effectors ?? []) as Json[]) {
      const where = String(e.site ?? e.body ?? "");
      if (e.kind === "pull") {
        const g = this.group(`Tirón «${String(e.id)}» en ${where}`,
          "Curva de la fuerza: en cada instante, módulo y elevación (0° = hacia delante de Uke, 90° = arriba, −90° = abajo, ±180° = hacia atrás). Entre filas se interpola; antes de la primera y desde la última no hay fuerza.");
        g.append(this.number("Tope de fuerza (N)", e.max_force_n as number | undefined, (v) => { e.max_force_n = v; }, "10", "500"));
        g.append(this.number("Tope de potencia (W)", e.max_power_w as number | undefined, (v) => { e.max_power_w = v; }, "10", "600"));
        const keys = e.keys as Array<{ t_s: number; force_n: Vec }>;
        this.table(g, ["Instante (s)", "Fuerza (N)", "Elevación (°)", "Lateral izq. (N)"], keys,
          (k) => { const p = forceToPolar(k.force_n); return [k.t_s, p.magnitude_n, p.elevation_deg, p.lateral_n]; },
          (k, v) => { k.t_s = v[0]!; k.force_n = [...polarToForce({ magnitude_n: v[1]!, elevation_deg: v[2]!, lateral_n: v[3]! })] as Vec; },
          (last) => last ? { t_s: Number((last.t_s + 0.1).toFixed(3)), force_n: [...last.force_n] as Vec } : { t_s: 0.2, force_n: [0, 0, 0] as Vec });
      } else if (e.kind === "hand") {
        const g = this.group(`Mano «${String(e.id)}» en ${where} (${String(e.link)})`, "Mano con muelle: se guía la posición del objetivo; la fuerza sale de la distancia al agarre.");
        g.append(
          this.number("Tope de fuerza (N)", e.max_force_n as number | undefined, (v) => { e.max_force_n = v; }, "10", "500"),
          this.number("Se cierra a (s)", e.close_s as number, (v) => { e.close_s = v; }, "0.05"),
          this.number("Suelta a (s)", e.release_s as number | undefined, (v) => { e.release_s = v; }, "0.05", "no suelta"),
        );
        this.phases(g, e, "en el marco inicial de Uke, respecto al agarre al cerrarse");
      } else if (e.kind === "prescribed_force") {
        const g = this.group(`Fuerza «${String(e.id)}» en ${where}`, `Fuerza fija en el marco ${String(e.frame)}, con forma ${String(e.waveform)}.`);
        const force = e.force_n as Vec;
        g.append(
          this.number("Delante X (N)", force[0], (v) => { force[0] = v; }, "10"),
          this.number("Izquierda Y (N)", force[1], (v) => { force[1] = v; }, "10"),
          this.number("Arriba Z (N)", force[2], (v) => { force[2] = v; }, "10"),
          this.number("Empieza a (s)", e.start_s as number, (v) => { e.start_s = v; }, "0.05"),
          this.number("Duración (s)", e.duration_s as number, (v) => { e.duration_s = v; }, "0.05"),
        );
      }
    }
  }
}
