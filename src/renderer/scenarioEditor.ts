/**
 * Editor de escenarios de la web: abre el escenario de una grabación de la galería, deja cambiar rozamientos, cuerpos,
 * obstáculos y la curva de fuerzas de cada mano punto a punto, lo simula en el navegador con el mismo `runScenario` que
 * `npm run sim` y reproduce el resultado. El escenario editado viaja en el enlace (`#scn=...`). Ver `docs/SCENARIO_EDITOR.md`.
 */
import { defaultToriKeys } from "../effectors/toriBody";
import { loadPhysicsModule } from "../physics/mujoco";
import type { Recording } from "../recording/schema";
import { decodeScenarioHash, describeTrial, encodeScenarioHash, forceToPolar, polarToForce } from "../scenario/editing";
import type { MujocoModule } from "../scenario/mujocoModule";
import { runScenario } from "../scenario/run";
import { ACTOR_BODIES, HUMANOID_SITE_BODY } from "../scenario/schema";
import { ScenarioValidationError, validateScenario } from "../scenario/validate";

export interface ScenarioEditorOptions {
  /** Inserta el bloque del editor en la interfaz. */
  mount(element: HTMLElement): void;
  /** Reproduce la grabación simulada. */
  show(recording: Recording): Promise<boolean>;
  /** Base de Vite, para descargar `scenarios/<id>.json`. */
  baseUrl: string;
  engineVersion?: string;
  /**
   * Pide un clic en el visor 3D sobre un cuerpo de la grabación abierta; llama a `onPicked` con el nombre del cuerpo y el
   * punto en sus ejes locales. Devuelve `false` si ahora no se puede (no hay grabación en pantalla).
   */
  pick?: (onPicked: (body: string, local: [number, number, number]) => void) => boolean;
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

/** Nombres en castellano de los puntos de Uke donde se puede aplicar una fuerza directa. */
const POINT_LABELS: Readonly<Record<string, string>> = {
  sternum: "esternón", belt_front: "cinturón, delante", belt_back: "cinturón, detrás", abdomen_front: "abdomen, delante",
  pelvis_origin: "pelvis", nape: "nuca", head_front: "cabeza, delante", head_side_left: "cabeza, lado izquierdo",
  head_side_right: "cabeza, lado derecho", left_shoulder_impact: "hombro izquierdo", right_shoulder_impact: "hombro derecho",
  left_sleeve_elbow: "manga izquierda (codo)", right_sleeve_elbow: "manga derecha (codo)", left_wrist: "muñeca izquierda",
  right_wrist: "muñeca derecha", left_knee_back: "corva izquierda", right_knee_back: "corva derecha",
  left_ankle: "tobillo izquierdo", right_ankle: "tobillo derecho",
};

/** Un `id` libre con el prefijo dado (`fuerza_1`, `fuerza_2`, ...). */
function freeId(taken: readonly unknown[], prefix: string): string {
  for (let n = 1; ; n++) {
    const id = `${prefix}_${n}`;
    if (!taken.includes(id)) return id;
  }
}

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

  /** Botón que quita `item` de `list` y vuelve a pintar el formulario. */
  private removeButton(list: unknown[], item: unknown, text: string): HTMLButtonElement {
    const button = el("button", { type: "button" }, text);
    button.addEventListener("click", () => {
      const at = list.indexOf(item);
      if (at >= 0) list.splice(at, 1);
      this.render();
      this.sync();
    });
    return button;
  }

  private select(label: string, options: ReadonlyArray<readonly [string, string]>, value: string, set: (v: string) => void): HTMLLabelElement {
    const select = el("select");
    for (const [v, text] of options) select.append(el("option", { value: v, selected: v === value }, text));
    select.addEventListener("change", () => { set(select.value); this.sync(); });
    const wrap = el("label", {}, label);
    wrap.append(select);
    return wrap;
  }

  /** Uke del borrador (el primer humanoide), para dar tamaño por defecto a Tori. */
  private ukeHeight(): number {
    const uke = ((this.draft?.actors ?? []) as Json[]).find((a) => a.kind === "humanoid");
    return typeof uke?.height_m === "number" ? uke.height_m : 1.75;
  }

  /** Cuerpo de Tori de partida: la secuencia de `defaultToriKeys`, con la manga derecha y la solapa de Uke agarradas. */
  private defaultToriBody(): Json {
    return {
      height_m: this.ukeHeight(),
      grips: { left: { site: "right_sleeve_elbow", max_force_n: 300 }, right: { site: "sternum", max_force_n: 300 } },
      keys: defaultToriKeys(this.ukeHeight()),
    };
  }

  private renderTori(d: Json, ground: Json): void {
    if (!d.tori) {
      const g = this.group("Tori", "Sin Tori: sobre Uke solo actúan las fuerzas directas y los obstáculos de abajo.");
      const enable = el("button", { type: "button" }, "Activar a Tori");
      enable.addEventListener("click", () => { d.tori = { mass_kg: 75, body: this.defaultToriBody() }; this.render(); this.sync(); });
      g.append(enable);
      return;
    }
    const tori = d.tori as Json;
    const g = this.group("Tori", "Con cuerpo, Tori ejecuta la secuencia de posturas, sus segmentos chocan con Uke con fuerza limitada y sus manos tiran de lo que agarran. El presupuesto juzga si podría sostener esas fuerzas con sus pies.");
    g.append(
      this.number("Masa (kg)", tori.mass_kg as number, (v) => { tori.mass_kg = v; }),
      this.number("Rozamiento de sus pies", tori.friction as number | undefined, (v) => { tori.friction = v; }, "0.05", `${String(ground.friction)} (el del suelo)`),
    );
    const body = tori.body as Json | undefined;
    const check = el("input", { type: "checkbox", checked: Boolean(body) && body?.enabled !== false });
    check.addEventListener("change", () => {
      if (!tori.body) tori.body = this.defaultToriBody();
      const b = tori.body as Json;
      if (check.checked) delete b.enabled; else b.enabled = false;
      this.render();
      this.sync();
    });
    const checkLabel = el("label", { className: "scenario-check" });
    checkLabel.append(check, " Cuerpo de Tori (se ve y hace de obstáculo)");
    g.append(checkLabel);
    const remove = el("button", { type: "button" }, "Quitar a Tori");
    remove.addEventListener("click", () => { delete d.tori; this.render(); this.sync(); });
    if (!body || body.enabled === false) { g.append(remove); return; }

    g.append(
      this.number("Altura (m)", body.height_m as number, (v) => { body.height_m = v; }, "0.01"),
      this.number("Rozamiento de su cuerpo con Uke", body.friction as number | undefined, (v) => { body.friction = v; }, "0.05", "0.8"),
    );
    const grips = (body.grips ?? (body.grips = {})) as Json;
    const sites: Array<readonly [string, string]> = [["", "no agarra"],
      ...Object.keys(HUMANOID_SITE_BODY).filter((site) => POINT_LABELS[site]).map((site) => [site, POINT_LABELS[site]!] as const)];
    for (const [side, text] of [["left", "Mano izquierda agarra"], ["right", "Mano derecha agarra"]] as const) {
      const grip = grips[side] as Json | undefined;
      g.append(this.select(text, sites, String(grip?.site ?? ""), (v) => {
        if (v === "") delete grips[side]; else grips[side] = { ...(grip ?? {}), site: v };
        this.render();
      }));
      if (!grip) continue;
      g.append(
        this.number("  Tope de fuerza (N)", grip.max_force_n as number | undefined, (v) => { grip.max_force_n = v; }, "10", "500"),
        this.number("  Suelta a (s)", grip.release_s as number | undefined, (v) => { grip.release_s = v; }, "0.05", "no suelta"),
      );
    }
    type Key = {
      t_s: number; pelvis_m: Vec; yaw_deg: number; lean_deg?: number; twist_deg?: number; left_foot_m: Vec; right_foot_m: Vec;
      left_hand_m?: Vec; right_hand_m?: Vec;
    };
    const REST: Record<"left" | "right", Vec> = { left: [0.25, 0.15, -0.35], right: [0.25, -0.15, -0.35] };
    const keys = body.keys as Key[];
    const make = (last: Key | undefined): Key => last
      ? JSON.parse(JSON.stringify({ ...last, t_s: Number((last.t_s + 0.2).toFixed(3)) })) as Key
      : (defaultToriKeys(this.ukeHeight())[0] as unknown as Key);
    g.append(el("p", { className: "muted" }, "Posturas (mundo: Uke empieza en X = 0 mirando a +X; Y es su izquierda). Entre filas se interpola. Giro: 180° = de frente a Uke, 360° = de espaldas tras girar a su izquierda."));
    this.table(g, ["Instante (s)", "Pelvis X (m)", "Pelvis Z (m)", "Giro (°)", "Tronco adelante (°)", "Hombros (°)"], keys,
      (k) => [k.t_s, k.pelvis_m[0], k.pelvis_m[2], k.yaw_deg, k.lean_deg ?? 0, k.twist_deg ?? 0],
      (k, v) => { k.t_s = v[0]!; k.pelvis_m = [v[1]!, k.pelvis_m[1], v[2]!]; k.yaw_deg = v[3]!; k.lean_deg = v[4]!; k.twist_deg = v[5]!; },
      make);
    g.append(el("p", { className: "muted" }, "Pies de cada postura (mismo orden de filas): dónde apoya cada uno."));
    this.table(g, ["Instante (s)", "Izq. X (m)", "Izq. Y (m)", "Der. X (m)", "Der. Y (m)"], keys,
      (k) => [k.t_s, k.left_foot_m[0], k.left_foot_m[1], k.right_foot_m[0], k.right_foot_m[1]],
      (k, v) => { k.t_s = v[0]!; k.left_foot_m = [v[1]!, v[2]!, k.left_foot_m[2]]; k.right_foot_m = [v[3]!, v[4]!, k.right_foot_m[2]]; },
      make);
    for (const [side, text] of [["left", "izquierda"], ["right", "derecha"]] as const) {
      const field = `${side}_hand_m` as const;
      g.append(el("p", { className: "muted" }, `Mano ${text} en cada postura, desde el punto medio entre sus hombros: delante del pecho, a su izquierda y hacia la cabeza. El agarre tira de Uke hacia ese punto.`));
      this.table(g, ["Instante (s)", "Delante (m)", "A su izq. (m)", "Hacia la cabeza (m)"], keys,
        (k) => [k.t_s, ...(k[field] ?? REST[side])],
        (k, v) => { k.t_s = v[0]!; k[field] = [v[1]!, v[2]!, v[3]!]; },
        make);
    }
    g.append(remove);
  }

  /** Fuerzas directas sobre Uke: lista de `prescribed_force` y botones para añadir un impacto o una fuerza constante. */
  private renderDirectForces(d: Json): void {
    const effectors = (d.effectors ?? (d.effectors = [])) as Json[];
    const uke = ((d.actors ?? []) as Json[]).find((a) => a.kind === "humanoid");
    const g = this.group("Fuerzas directas sobre Uke", "Impactos, choques o empujes aplicados en un punto de Uke, sin Tori. Ejes: X delante de Uke, Y su izquierda, Z arriba.");
    if (!uke) { g.append(el("p", { className: "muted" }, "Este escenario no tiene un Uke humanoide.")); return; }
    const add = (waveform: string, duration: number, force: Vec): void => {
      effectors.push({
        id: freeId(effectors.map((e) => e.id), "fuerza"), kind: "prescribed_force", actor: uke.id, site: "sternum",
        frame: "actor_initial", force_n: force, start_s: 0.3, duration_s: duration, waveform,
      });
      this.render();
      this.sync();
    };
    const impact = el("button", { type: "button" }, "+ Impacto (0,1 s)");
    impact.addEventListener("click", () => add("half_sine", 0.1, [-900, 0, 0]));
    const constant = el("button", { type: "button" }, "+ Fuerza constante");
    constant.addEventListener("click", () => add("constant", 1, [-200, 0, 0]));
    const row = el("div", { className: "button-row" });
    row.append(impact, constant);
    g.append(row);
  }

  private renderForce(e: Json, effectors: Json[]): void {
    const where = String(e.site ?? e.body ?? "");
    const g = this.group(`Fuerza «${String(e.id)}» en ${POINT_LABELS[where] ?? where}`);
    const force = e.force_n as Vec;
    const points: Array<readonly [string, string]> = [
      ...Object.keys(HUMANOID_SITE_BODY).filter((site) => POINT_LABELS[site]).map((site) => [`site:${site}`, POINT_LABELS[site]!] as const),
      ...ACTOR_BODIES.humanoid.map((b) => [`body:${b}`, `centro de ${b}`] as const),
    ];
    const local = e.local_point_m as Vec | undefined;
    const custom = e.site === undefined && local !== undefined && local.some((x) => x !== 0);
    if (custom) points.unshift([`body:${String(e.body)}`, `punto elegido en ${String(e.body)}`]);
    g.append(this.select("Punto de aplicación", points, e.site !== undefined ? `site:${String(e.site)}` : `body:${String(e.body)}`, (v) => {
      const [kind, name] = v.split(":") as [string, string];
      delete e.site; delete e.body; delete e.local_point_m;
      if (kind === "site") e.site = name; else { e.body = name; e.local_point_m = [0, 0, 0]; }
      this.render();
    }));
    if (this.options.pick) {
      const pick = el("button", { type: "button" }, "Elegir el punto en el visor 3D");
      pick.addEventListener("click", () => {
        const armed = this.options.pick!((body, point) => {
          if (!(ACTOR_BODIES.humanoid as readonly string[]).includes(body)) {
            this.say("Ese cuerpo no es de Uke: pulsa otra vez el botón y toca a Uke.", true);
            return;
          }
          delete e.site;
          e.body = body;
          e.local_point_m = point.map((x) => Number(x.toFixed(3)));
          this.render();
          this.sync();
          this.say(`Punto fijado en ${body}. Pulsa «Simular con estos valores».`);
        });
        this.say(armed ? "Toca a Uke en el visor 3D, en el punto donde quieres aplicar la fuerza." : "Abre antes una grabación para ver a Uke en el visor.", !armed);
      });
      g.append(pick);
    }
    g.append(
      this.select("Tipo", [["half_sine", "impacto (sube y baja)"], ["constant", "constante"]], String(e.waveform), (v) => { e.waveform = v; }),
      this.select("Ejes de la fuerza", [["actor_initial", "los de Uke al empezar"], ["world", "los del mundo"], ["actor_current", "los del cuerpo en cada instante"]],
        String(e.frame), (v) => { e.frame = v; }),
      this.number("Delante X (N)", force[0], (v) => { force[0] = v; }, "10"),
      this.number("Izquierda Y (N)", force[1], (v) => { force[1] = v; }, "10"),
      this.number("Arriba Z (N)", force[2], (v) => { force[2] = v; }, "10"),
      this.number("Empieza a (s)", e.start_s as number, (v) => { e.start_s = v; }, "0.05"),
      this.number("Duración (s)", e.duration_s as number, (v) => { e.duration_s = v; }, "0.05"),
      this.removeButton(effectors, e, "Quitar esta fuerza"),
    );
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

    this.renderTori(d, ground);
    this.renderDirectForces(d);
    const effectors = (d.effectors ?? []) as Json[];
    for (const e of effectors) if (e.kind === "prescribed_force") this.renderForce(e, effectors);

    for (const o of (d.obstacles ?? []) as Json[]) {
      const g = this.group(`Obstáculo «${String(o.id)}» (${String(o.shape)})`, "La cadera, la pierna o el pie de Tori. Coordenadas en el mundo: Uke empieza en X = 0 mirando hacia +X.");
      const start = o.start_m as Vec;
      const surface = (o.surface as keyof typeof DEFAULT_OBSTACLE_FRICTION | undefined) ?? "cloth";
      const tori = Number(o.tori_mass_kg);
      g.append(
        this.number("Rozamiento con Uke", o.friction as number | undefined, (v) => { o.friction = v; }, "0.05", String(DEFAULT_OBSTACLE_FRICTION[surface])),
        this.number("Distancia delante de Uke, X (m)", start[0], (v) => { start[0] = v; }, "0.01"),
        this.number("A su izquierda, Y (m)", start[1], (v) => { start[1] = v; }, "0.01"),
        this.number("Altura del centro, Z (m)", start[2], (v) => { start[2] = v; }, "0.01"),
        this.number("Radio (m)", o.radius_m as number | undefined, (v) => { o.radius_m = v; }, "0.01", "0.11 (barra)"),
        this.number("Tope de fuerza horizontal (N)", o.max_force_n as number | undefined, (v) => { o.max_force_n = v; }, "10", (0.8 * tori * 9.81).toFixed(0)),
        this.number("Tope de fuerza vertical (N)", o.max_vertical_force_n as number | undefined, (v) => { o.max_vertical_force_n = v; }, "10", (2 * tori * 9.81).toFixed(0)),
      );
      this.phases(g, o, "en el mundo, respecto a la posición inicial");
      g.append(this.removeButton(d.obstacles as unknown[], o, "Quitar este obstáculo"));
    }
    {
      const g = this.group("Añadir obstáculo", "Un cuerpo con fuerza limitada contra el que Uke choca: barra a lo ancho, poste vertical o almohadilla.");
      const row = el("div", { className: "button-row" });
      for (const [shape, text] of [["bar", "+ Barra"], ["post", "+ Poste"], ["pad", "+ Almohadilla"]] as const) {
        const button = el("button", { type: "button" }, text);
        button.addEventListener("click", () => {
          const list = (d.obstacles ?? (d.obstacles = [])) as Json[];
          const toriMass = typeof (d.tori as Json | undefined)?.mass_kg === "number" ? (d.tori as Json).mass_kg : 75;
          list.push({ id: freeId(list.map((o) => o.id), "obstaculo"), shape, tori_mass_kg: toriMass, start_m: [0.3, 0, 0.9], phases: [] });
          this.render();
          this.sync();
        });
        row.append(button);
      }
      g.append(row);
    }

    for (const e of effectors) {
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
        g.append(this.removeButton(effectors, e, "Quitar este tirón"));
      } else if (e.kind === "hand") {
        const g = this.group(`Mano «${String(e.id)}» en ${where} (${String(e.link)})`, "Mano con muelle: se guía la posición del objetivo; la fuerza sale de la distancia al agarre.");
        g.append(
          this.number("Tope de fuerza (N)", e.max_force_n as number | undefined, (v) => { e.max_force_n = v; }, "10", "500"),
          this.number("Se cierra a (s)", e.close_s as number, (v) => { e.close_s = v; }, "0.05"),
          this.number("Suelta a (s)", e.release_s as number | undefined, (v) => { e.release_s = v; }, "0.05", "no suelta"),
        );
        this.phases(g, e, "en el marco inicial de Uke, respecto al agarre al cerrarse");
        g.append(this.removeButton(effectors, e, "Quitar esta mano"));
      }
    }
  }
}
