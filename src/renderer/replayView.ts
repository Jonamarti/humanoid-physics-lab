/**
 * Modo reproducción del visor web: carga una grabación `hpl.recording.v1` y la reproduce con
 * `RecordingPlayer` + cinemática directa de MuJoCo (`mj_forward`, nunca `mj_step`). Construye su propia
 * interfaz y deja el laboratorio intacto; `main.ts` solo lo instancia (ver `docs/REPLAY.md`).
 */
import { loadPhysicsModule } from "../physics/mujoco";
import { PLAYBACK_SPEEDS, RecordingPlayer } from "../recording/player";
import { parseRecording } from "../recording/read";
import { ReplayKinematics } from "../recording/replayKinematics";
import type { Recording } from "../recording/schema";
import type { BallViewer } from "./ballViewer";

export interface ReplayViewOptions {
  viewer: BallViewer | undefined;
  /** Contenedor donde se inserta el botón y los controles. */
  host: HTMLElement;
  /** Contenedor del visor 3D (con `position: relative`) donde se superpone el rótulo de fuerzas. */
  overlayHost: HTMLElement;
  /** Se llama al entrar y al salir del modo reproducción (para pausar o reactivar el laboratorio). */
  onModeChange(active: boolean): void;
  onStatus(text: string): void;
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, props: Partial<HTMLElementTagNameMap[K]> = {}, text?: string): HTMLElementTagNameMap[K] {
  const node = Object.assign(document.createElement(tag), props);
  if (text !== undefined) node.textContent = text;
  return node;
}

const fmtN = (n: number): string => (Math.abs(n) >= 100 ? n.toFixed(0) : n.toFixed(1)) + " N";

export class ReplayView {
  private kin: ReplayKinematics | undefined;
  private player: RecordingPlayer | undefined;
  private recording: Recording | undefined;
  private frameHandle: number | undefined;
  private lastMs: number | undefined;
  private readonly fileInput = el("input", { type: "file", accept: ".json,.gz,application/json,application/gzip" });
  private readonly panel = el("div", { className: "replay-panel", hidden: true });
  private readonly info = el("p", { className: "result" });
  private readonly timeBar = el("input", { type: "range", min: "0", max: "0", step: "1", value: "0" });
  private readonly timeText = el("span", { className: "muted" });
  private readonly playButton = el("button", { type: "button" }, "Reproducir");
  private readonly speedButtons = new Map<number, HTMLButtonElement>();
  private readonly actorSelect = el("select", { ariaLabel: "Actor medido" });
  private readonly diagnosticText = el("pre", { className: "replay-diagnostics" });
  private readonly showContacts = el("input", { type: "checkbox", checked: false });
  private readonly showReactions = el("input", { type: "checkbox", checked: false });
  private readonly overlay = el("div", { className: "replay-forces", hidden: true });

  constructor(private readonly options: ReplayViewOptions) {
    const loadButton = el("button", { type: "button" }, "Cargar grabación");
    this.fileInput.hidden = true;
    loadButton.addEventListener("click", () => this.fileInput.click());
    this.fileInput.addEventListener("change", () => {
      const file = this.fileInput.files?.[0];
      this.fileInput.value = "";
      if (file) void this.load(file);
    });

    const prev = el("button", { type: "button", ariaLabel: "Muestra anterior" }, "◀ Anterior");
    const next = el("button", { type: "button", ariaLabel: "Muestra siguiente" }, "Siguiente ▶");
    const exit = el("button", { type: "button" }, "Salir del modo reproducción");
    this.playButton.addEventListener("click", () => { this.player?.toggle(); this.lastMs = undefined; this.refresh(); });
    prev.addEventListener("click", () => { this.player?.stepPrev(); this.refresh(); });
    next.addEventListener("click", () => { this.player?.stepNext(); this.refresh(); });
    exit.addEventListener("click", () => this.exit());
    this.timeBar.setAttribute("aria-label", "Barra de tiempo");
    this.timeBar.addEventListener("input", () => {
      this.player?.pause();
      this.player?.seek(Number(this.timeBar.value) / 1000);
      this.refresh();
    });
    const speeds = el("div", { className: "button-row" });
    for (const speed of PLAYBACK_SPEEDS) {
      const button = el("button", { type: "button" }, "×" + String(speed).replace(".", ","));
      button.addEventListener("click", () => { this.player?.setSpeed(speed); this.refresh(); });
      this.speedButtons.set(speed, button);
      speeds.append(button);
    }
    const transport = el("div", { className: "button-row" });
    transport.append(prev, this.playButton, next);
    this.panel.append(this.info, transport, speeds, this.timeBar, this.timeText, exit);
    const diagnostics = el("details");
    diagnostics.append(el("summary", {}, "Fuerzas, apoyos y estado de los agarres"), this.diagnosticText);
    for (const [input, text] of [[this.showContacts, "Mostrar contactos"], [this.showReactions, "Mostrar reacción sobre Tori"]] as const) {
      const label = el("label"); label.append(input, document.createTextNode(text)); diagnostics.append(label);
      input.addEventListener("change", () => this.refresh());
    }
    this.actorSelect.hidden = true;
    this.actorSelect.addEventListener("change", () => this.refresh());
    this.panel.append(this.actorSelect, diagnostics);

    const row = el("div", { className: "button-row" });
    row.append(loadButton, this.fileInput);
    options.host.append(row, this.panel);
    options.overlayHost.append(this.overlay);
  }

  get active(): boolean { return this.player !== undefined; }

  /** Inserta un bloque de interfaz entre «Cargar grabación» y los controles de reproducción (lo usa la galería). */
  insertBeforePanel(node: HTMLElement): void { this.panel.before(node); }

  /** Inserta un bloque de interfaz después de los controles de reproducción (lo usa el editor de escenarios). */
  insertAfterPanel(node: HTMLElement): void { this.panel.after(node); }

  /** Carga y reproduce una grabación. Devuelve `true` si se pudo cargar (el motivo del fallo va a `onStatus`). */
  async load(file: Blob): Promise<boolean> {
    const { viewer, onStatus } = this.options;
    if (!viewer) { onStatus("Se necesita WebGL para reproducir grabaciones."); return false; }
    onStatus("Leyendo grabación…");
    try {
      return await this.show(await parseRecording(await file.arrayBuffer()));
    } catch (error) {
      this.release();
      this.options.onModeChange(false);
      onStatus("No se pudo cargar la grabación: " + (error instanceof Error ? error.message : String(error)));
      return false;
    }
  }

  /** Reproduce una grabación ya en memoria (la usa el editor de escenarios tras simular en el navegador). */
  async show(recording: Recording): Promise<boolean> {
    const { viewer, onStatus } = this.options;
    if (!viewer) { onStatus("Se necesita WebGL para reproducir grabaciones."); return false; }
    try {
      const mj = await loadPhysicsModule();
      this.release();
      this.options.onModeChange(true);
      this.kin = new ReplayKinematics(mj, recording.model.mjcf);
      this.player = new RecordingPlayer(recording, this.kin.jointTypes());
      this.recording = recording;
      viewer.showArticulated(this.kin.geomSpecs());
      this.actorSelect.replaceChildren();
      for (const actor of recording.meta.humanoid_actors ?? []) this.actorSelect.append(el("option", {value:actor.id}, `Medir ${actor.id}`));
      this.actorSelect.hidden = !recording.meta.humanoid_actors;
      const s = recording.summary;
      this.info.textContent = `Escenario ${recording.meta.scenario_id} · resultado ${s.outcome}` +
        (s.actors ? ` · ${Object.entries(s.actors).map(([id,a])=>`${id}: ${a.outcome}`).join(" · ")}` : "") +
        (s.direction_deg === null ? "" : ` hacia ${Math.round(s.direction_deg) || 0}°`) + ` · ${recording.samples.length} muestras`;
      this.timeBar.max = String(Math.round(this.player.duration * 1000));
      this.panel.hidden = false;
      this.player.play();
      this.refresh();
      this.lastMs = undefined;
      this.frameHandle = window.requestAnimationFrame(this.tick);
      onStatus("Modo reproducción: la física no se ejecuta, solo se muestran las muestras grabadas.");
      return true;
    } catch (error) {
      this.release();
      this.options.onModeChange(false);
      onStatus("No se pudo cargar la grabación: " + (error instanceof Error ? error.message : String(error)));
      return false;
    }
  }

  /**
   * Pide un clic sobre un cuerpo de la grabación abierta (la reproducción se pausa) y devuelve su nombre y el punto en sus
   * ejes locales. Devuelve `false` si no hay grabación abierta.
   */
  pickBodyPoint(onPicked: (body: string, local: [number, number, number]) => void): boolean {
    const { kin, recording, player } = this;
    const viewer = this.options.viewer;
    if (!kin || !recording || !player || !viewer) return false;
    player.pause();
    this.refresh();
    viewer.pickArticulatedOnce((geomId, point) => {
      if (this.kin !== kin) return;
      const hit = kin.locatePoint(geomId, point);
      onPicked(recording.model.bodies[hit.bodyId] ?? "", hit.local);
    });
    return true;
  }

  exit(): void {
    if (!this.player) return;
    this.release();
    this.options.onModeChange(false);
    this.options.onStatus("Modo reproducción cerrado. Vuelves al laboratorio.");
  }

  private readonly tick = (ms: number): void => {
    this.frameHandle = window.requestAnimationFrame(this.tick);
    const dt = this.lastMs === undefined ? 0 : Math.min((ms - this.lastMs) / 1000, 0.1);
    this.lastMs = ms;
    if (this.player?.playing) {
      this.player.advance(dt);
      this.refresh();
    }
  };

  /** Dibuja el instante actual: `qpos` interpolado → `mj_forward` → poses y flechas de los efectores. */
  private refresh(): void {
    const { player, kin, recording } = this;
    const viewer = this.options.viewer;
    if (!player || !kin || !recording || !viewer) return;
    const frame = player.current();
    kin.setQpos(frame.qpos);
    viewer.setArticulatedPoses(kin.geomPoses());
    const forces = frame.effectors.filter((e) => Math.hypot(...e.force_n) > 0);
    const recordedSample = recording.samples[frame.index]!;
    const observation = recordedSample.actors?.[this.actorSelect.value] ?? recordedSample;
    if (observation.com) viewer.setBalanceProjection(observation.com, observation.support_polygon ?? []);
    else viewer.clearBalanceProjection();
    const reactions = frame.effectors.filter(e => this.showReactions.checked && e.coupled_grip && Math.hypot(...e.coupled_grip.reaction_n) > 0);
    viewer.setMultiForces([
      ...forces.map(e => ({ id: e.id, point: e.point_m, force: e.force_n, active: true })),
      ...reactions.map(e => ({ id: e.id + "_reaction", point: e.point_m, force: e.coupled_grip!.reaction_n, active: true })),
      ...frame.contacts.filter(c => this.showContacts.checked && Math.hypot(...c.force_n) > 20).map((c, i) => ({ id: "contact_" + i,
        point: c.point_m, force: c.force_n, active: true })),
    ]);
    this.overlay.hidden = false;
    const lines = this.actorSelect.hidden ? [] : [`Actor medido: ${this.actorSelect.value} · control articular · pelvis libre`];
    lines.push(...forces.map(e => `${e.id}: ${fmtN(Math.hypot(...e.force_n))}`));
    for (const e of frame.effectors) if (e.coupled_grip) lines.push(`${e.id}: ${e.coupled_grip.status}, separación ${(100*e.coupled_grip.distance_m).toFixed(1)} cm`);
    if (observation.capture_margin_m != null) lines.push(`Margen dinámico: ${(100*observation.capture_margin_m).toFixed(1)} cm (no decide la caída)`);
    if (observation.com_velocity_mps) lines.push(`Velocidad COM: ${Math.hypot(...observation.com_velocity_mps).toFixed(2)} m/s`);
    if (observation.ground_reaction_n) lines.push(`Apoyo vertical izq/der: ${observation.ground_reaction_n.left[2].toFixed(0)} / ${observation.ground_reaction_n.right[2].toFixed(0)} N`);
    if (recording.meta.tori_assist) lines.push("Tori: seguimiento asistido de segmentos; no tiene equilibrio propio");
    if (recording.summary.artifacts?.length) lines.push(`Artefactos pendientes: ${recording.summary.artifacts.map(a => a.type).join(", ")}`);
    this.diagnosticText.textContent = lines.join("\n");
    this.overlay.textContent = [
      ...forces.map(e => `${e.id}: ${fmtN(Math.hypot(...e.force_n))}`),
      ...(observation.capture_margin_m != null ? [`Margen dinámico: ${(100*observation.capture_margin_m).toFixed(1)} cm`] : []),
    ].join("\n");
    viewer.setBalanceDynamics(observation.com, observation.com_velocity_mps, observation.xcom);
    this.timeBar.value = String(Math.round(frame.t * 1000));
    this.timeText.textContent = `t = ${frame.t.toFixed(3)} s / ${player.endTime.toFixed(3)} s · ×${String(player.speed).replace(".", ",")}`;
    this.playButton.textContent = player.playing ? "Pausa" : "Reproducir";
    for (const [speed, button] of this.speedButtons) button.setAttribute("aria-pressed", String(speed === player.speed));
  }

  private release(): void {
    if (this.frameHandle !== undefined) window.cancelAnimationFrame(this.frameHandle);
    this.frameHandle = undefined;
    this.options.viewer?.clearMultiForces();
    this.options.viewer?.clearBalanceProjection();
    this.options.viewer?.pickArticulatedOnce(undefined);
    this.kin?.dispose();
    this.kin = undefined;
    this.player = undefined;
    this.recording = undefined;
    this.panel.hidden = true;
    this.overlay.hidden = true;
  }
}
