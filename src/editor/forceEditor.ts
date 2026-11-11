import type { MujocoSimulation } from "../physics/mujoco";
import type { BallViewer } from "../renderer/ballViewer";
import {
  FORCE_SITES, editedForceAtTime, editedForceImpulseNs, editedSequenceHorizonS,
  validateForceSequence, type EditedForce, type ForceSite, type ForceVector,
  type ForceWaveform,
} from "../physics/forceSequence";

export interface ForceEditor {
  readonly enabled: boolean;
  readonly running: boolean;
  readonly horizonS: number;
  refresh(): void;
  reset(): void;
  /** Collapse editor when a standing scenario is unloaded; keep force settings. */
  disable(): void;
  /** Clear body-index attachments when switching MuJoCo model layouts. */
  clearCustomAttachments(): number;
  finish(): void;
  beginFrame(): void;
  applyStep(sim: MujocoSimulation, timeS: number): void;
  present(sim: MujocoSimulation | undefined, viewer: BallViewer | undefined, timeS: number): void;
}

interface EditorCallbacks {
  getSimulation(): MujocoSimulation | undefined;
  getViewer(): BallViewer | undefined;
  canEdit(): boolean;
  onToggle(): void;
  onRun(): void;
}

/** UI in a separate module: the integrator only sees validated snapshots. */
export function installForceEditor(host: HTMLElement, callbacks: EditorCallbacks): ForceEditor {
  let enabled = false;
  let snapshot: readonly EditedForce[] | undefined;
  let horizonS = 6;
  let nextId = 2;
  let forces: EditedForce[] = [{
    id: "f1", site: "sternum_impact", forceN: [-240, 0, 0],
    startS: 1, durationS: 0.12, waveform: "half-sine",
  }];
  const applied = new Map<string, ForceVector>();
  let selectedId = "f1";
  function configureViewer(): void {
    callbacks.getViewer()?.configureForceEditing(
      enabled && !snapshot && callbacks.canEdit() ? selectedId : undefined,
      enabled && !snapshot && callbacks.canEdit() ? (id, geomId, point) => {
        const sim = callbacks.getSimulation();
        if (!sim) return;
        try {
          const attachment = sim.bodyAttachmentAtGeomPoint(geomId, point);
          replace(id, { attachment });
          status.textContent = "Punto fijado en el cuerpo " + attachment.bodyIndex +
            " (coordenadas locales, sigue al segmento).";
        } catch (error) {
          status.textContent = error instanceof Error ? error.message : String(error);
        }
      } : undefined,
      enabled && !snapshot && callbacks.canEdit() ? (id, direction) => {
        const current = forces.find(f => f.id === id);
        if (!current) return;
        const magnitude = Math.hypot(...current.forceN);
        if (!magnitude) return;
        replace(id, {
          forceN: direction.map(component => Number((component * magnitude).toFixed(3))) as unknown as ForceVector,
        });
      } : undefined,
    );
  }
  const heading = document.createElement("label");
  heading.className = "force-editor-toggle";
  const check = document.createElement("input");
  check.type = "checkbox";
  heading.append(check, document.createTextNode(" Editor avanzado: varias fuerzas 3D"));
  const pane = document.createElement("div");
  pane.hidden = true;
  pane.className = "advanced-forces";
  const help = document.createElement("p");
  help.className = "muted";
  help.textContent = "Ejes del mundo: X adelante, Y izquierda, Z arriba. " +
    "⊗ entra en la pantalla; ⊙ sale de la pantalla. El símbolo depende de la cámara. " +
    "El impulso es vectorial (N·s) y cada punto pertenece al humanoide.";
  const rows = document.createElement("div");
  rows.className = "force-editor-rows";
  const actions = document.createElement("div");
  actions.className = "button-row";
  const add = document.createElement("button");
  add.type = "button";
  add.textContent = "+ Añadir fuerza";
  const run = document.createElement("button");
  run.type = "button";
  run.textContent = "Ejecutar secuencia";
  const status = document.createElement("p");
  status.className = "muted";
  status.setAttribute("role", "status");
  actions.append(add, run);
  pane.append(help, rows, actions, status);
  host.append(heading, pane);

  function checked(): boolean {
    try { validateForceSequence(forces); return true; }
    catch { return false; }
  }

  function refresh(): void {
    run.disabled = !enabled || !!snapshot || !callbacks.canEdit() || !callbacks.getSimulation() || !checked();
    add.disabled = !!snapshot || !callbacks.canEdit() || forces.length >= 12;
    configureViewer();
    if (!snapshot) status.textContent = checked()
      ? forces.length + " fuerza(s) configurada(s). Pulsa ejecutar para aplicar física real."
      : "Parámetros inválidos: limita la magnitud total a 10.000 N por fuerza.";
  }

  function replace(id: string, update: Partial<EditedForce>): void {
    if (snapshot || !callbacks.canEdit()) return;
    const i = forces.findIndex(f => f.id === id);
    if (i < 0) return;
    forces[i] = { ...forces[i]!, ...update };
    draw();
    preview();
  }

  function field(group: HTMLElement, title: string, input: HTMLElement): void {
    const label = document.createElement("label");
    label.textContent = title;
    label.append(input);
    group.append(label);
  }

  function draw(): void {
    rows.replaceChildren();
    for (const f of forces) {
      const group = document.createElement("fieldset");
      group.className = "edited-force-row";
      const legend = document.createElement("legend");
      legend.textContent = "Fuerza " + f.id + (f.id === selectedId ? " · seleccionada" : "");
      group.append(legend);
      const select = document.createElement("button");
      select.type = "button"; select.textContent = "Editar esta flecha";
      select.disabled = !!snapshot || !callbacks.canEdit();
      select.addEventListener("click", () => {
        selectedId = f.id; draw(); preview();
      });
      group.append(select);
      const site = document.createElement("select");
      for (const item of FORCE_SITES) {
        const option = document.createElement("option");
        option.value = item.id;
        option.textContent = item.label;
        option.selected = item.id === f.site;
        site.append(option);
      }
      site.disabled = !!snapshot || !callbacks.canEdit();
      site.addEventListener("change", () => replace(f.id, {
        site: site.value as ForceSite, attachment: undefined,
      }));
      field(group, "Punto de aplicación", site);
      const pick = document.createElement("button");
      pick.type = "button";
      pick.textContent = f.attachment ? "Recolocar sobre el cuerpo" : "Elegir punto sobre el cuerpo";
      pick.disabled = !!snapshot || !callbacks.canEdit() || !callbacks.getSimulation();
      pick.addEventListener("click", () => {
        selectedId = f.id;
        configureViewer();
        callbacks.getViewer()?.armForcePointPick(f.id);
        status.textContent = "Toca o haz clic sobre el segmento del humanoide donde aplicar la fuerza.";
      });
      group.append(pick);
      if (f.attachment) {
        const anchor = document.createElement("small");
        anchor.textContent = "Anclaje físico: cuerpo " + f.attachment.bodyIndex +
          ", offset COM " + f.attachment.localComOffsetM.map(v => v.toFixed(3)).join(", ") + " m";
        group.append(anchor);
      }
      for (let axis = 0; axis < 3; axis++) {
        const input = document.createElement("input");
        input.type = "number";
        input.step = "10";
        input.value = String(f.forceN[axis]);
        input.disabled = !!snapshot || !callbacks.canEdit();
        input.addEventListener("change", () => {
          const components = [...f.forceN];
          components[axis] = input.valueAsNumber;
          replace(f.id, { forceN: components as unknown as ForceVector });
        });
        field(group, ["Fx (N)", "Fy (N)", "Fz (N)"][axis]!, input);
      }
      for (const [key, title] of [["startS", "Inicio (s)"], ["durationS", "Duración (s)"]] as const) {
        const input = document.createElement("input");
        input.type = "number";
        input.min = key === "startS" ? "0" : "0.002";
        input.max = "20";
        input.step = "0.01";
        input.value = String(f[key]);
        input.disabled = !!snapshot;
        input.addEventListener("change", () => replace(f.id, key === "startS" ? { startS: input.valueAsNumber } : { durationS: input.valueAsNumber }));
        field(group, title, input);
      }
      const shape = document.createElement("select");
      for (const [value, title] of [
        ["half-sine", "Pulso (medio seno)"], ["constant", "Fuerza sostenida"],
      ] as const) {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = title;
        option.selected = value === f.waveform;
        shape.append(option);
      }
      shape.disabled = !!snapshot || !callbacks.canEdit();
      shape.addEventListener("change", () => replace(f.id, { waveform: shape.value as ForceWaveform }));
      field(group, "Aplicación temporal", shape);
      const impulse = document.createElement("p");
      impulse.className = "muted";
      try {
        impulse.textContent = "J(X,Y,Z) = " +
          editedForceImpulseNs(f).map(v => v.toFixed(2)).join(", ") + " N·s";
      } catch {
        impulse.textContent = "Parámetros inválidos";
      }
      group.append(impulse);
      const remove = document.createElement("button");
      remove.type = "button";
      remove.textContent = "Eliminar";
      remove.disabled = !!snapshot || !callbacks.canEdit() || forces.length === 1;
      remove.addEventListener("click", () => {
        if (snapshot) return;
        forces = forces.filter(other => other.id !== f.id);
        if (selectedId === f.id) selectedId = forces[0]!.id;
        draw(); preview();
      });
      group.append(remove);
      rows.append(group);
    }
    refresh();
  }
  function preview(): void {
    refresh();
    const sim = callbacks.getSimulation();
    if (sim && enabled) editor.present(sim, callbacks.getViewer(), sim.state().time);
  }
  const editor: ForceEditor = {
    get enabled() { return enabled; },
    get running() { return !!snapshot; },
    get horizonS() { return horizonS; },
    refresh,
    finish() {
      snapshot = undefined;
      applied.clear();
      draw();
      configureViewer();
      status.textContent = "Ensayo terminado. Puedes editar las fuerzas y volver a ejecutar.";
    },
    reset() {
      snapshot = undefined;
      applied.clear();
      draw();
      if (!enabled) callbacks.getViewer()?.clearMultiForces();
      configureViewer();
    },
    disable() {
      enabled=false;
      check.checked=false;
      pane.hidden=true;
      snapshot=undefined;
      applied.clear();
      configureViewer();
      callbacks.getViewer()?.clearMultiForces();
      draw();
    },
    clearCustomAttachments() {
      const count=forces.filter(f=>f.attachment!==undefined).length;
      if(count>0){
        // The same body index may identify another limb in v1 versus v3.
        // Preserve vectors and timings but require user to pick physical
        // attachment points anew on the newly loaded model.
        forces=forces.map(f=>({...f,attachment:undefined}));
        snapshot=undefined;
        applied.clear();
        draw();
        status.textContent=count+" anclaje(s) manual(es) invalidado(s) al cambiar de modelo. "+
          "Selecciona de nuevo los puntos del cuerpo para aplicar esas fuerzas.";
      }
      return count;
    },
    beginFrame() { applied.clear(); },
    applyStep(sim, timeS) {
      if (!snapshot) return;
      for (const f of snapshot) {
        const vec = editedForceAtTime(timeS, f);
        if (Math.hypot(...vec) <= 0) continue;
        if (f.attachment) {
          sim.applyForceAtBodyLocalPoint(
            f.attachment.bodyIndex, f.attachment.localComOffsetM, vec);
        } else {
          sim.applyWorldForceAtSite(f.site, vec);
        }
        const previous = applied.get(f.id);
        if (!previous || Math.hypot(...vec) > Math.hypot(...previous)) applied.set(f.id, vec);
      }
    },
    present(sim, viewer, timeS) {
      if (!enabled || !sim) {
        viewer?.clearMultiForces();
        return;
      }
      if (!snapshot && !checked()) {
        viewer?.clearMultiForces();
        return;
      }
      viewer?.setMultiForces((snapshot ?? forces).map(f => {
        const instant = applied.get(f.id);
        return {
          id: f.id,
          point: f.attachment
            ? sim.bodyAttachmentWorldPosition(f.attachment.bodyIndex, f.attachment.localComOffsetM)
            : sim.siteWorldPosition(f.site),
          force: instant ?? f.forceN,
          active: !!instant,
        };
      }));
      if (snapshot) status.textContent = "Simulación t = " + timeS.toFixed(2) +
        " s · " + applied.size + " fuerza(s) activas en el último fotograma.";
    },
  };
  check.addEventListener("change", () => {
    if (snapshot || !callbacks.canEdit()) {
      check.checked = enabled;
      return;
    }
    enabled = check.checked;
    pane.hidden = !enabled;
    configureViewer();
    preview();
    callbacks.onToggle();
  });
  add.addEventListener("click", () => {
    if (snapshot || !callbacks.canEdit() || forces.length >= 12) return;
    forces.push({
      id: "f" + nextId++, site: "sternum_impact", forceN: [0, 120, 0],
      startS: 1.2, durationS: 0.2, waveform: "half-sine",
    });
    draw(); preview();
  });
  run.addEventListener("click", () => {
    if (!enabled || snapshot || !callbacks.canEdit() || !callbacks.getSimulation()) return;
    try {
      validateForceSequence(forces);
      snapshot = forces.map(f => ({
        ...f,
        forceN: [...f.forceN] as unknown as ForceVector,
        attachment: f.attachment ? {
          bodyIndex: f.attachment.bodyIndex,
          localComOffsetM: [...f.attachment.localComOffsetM] as unknown as ForceVector,
        } : undefined,
      }));
      horizonS = editedSequenceHorizonS(snapshot);
      applied.clear();
      draw();
      callbacks.onRun();
    } catch (error) {
      // Rejecting a run must not retain a frozen snapshot or leave
      // the editor disabled. No force has started yet.
      snapshot=undefined;
      applied.clear();
      draw();
      status.textContent = error instanceof Error ? error.message : String(error);
    }
  });
  draw();
  return editor;
}
