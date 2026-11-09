/** Editable external forces, expressed in MuJoCo WORLD coordinates (Z up).
 * A configuration is snapshotted before integration and never edited mid-step.
 * Several forces on the same site add via MuJoCo's external wrench accumulator.
 */
export const FORCE_SITES = [
  { id: "sternum_impact", label: "Esternón" },
  { id: "pelvis_origin", label: "Pelvis" },
  { id: "left_forearm_impact", label: "Antebrazo izquierdo" },
  { id: "right_forearm_impact", label: "Antebrazo derecho" },
  { id: "left_shoulder_impact", label: "Hombro izquierdo" },
  { id: "right_shoulder_impact", label: "Hombro derecho" },
] as const;

export type ForceSite = typeof FORCE_SITES[number]["id"];
export type ForceWaveform = "half-sine" | "constant";
export type ForceVector = readonly [number, number, number];

/** Body-relative COM offset, captured from a pick on the physical mesh.
 * The body moves after the pick, but the force stays fixed on that body.
 */
export interface ForceBodyAttachment {
  readonly bodyIndex: number;
  readonly localComOffsetM: ForceVector;
}

export interface EditedForce {
  /** Override the named site with an arbitrary, body-attached point. */
  readonly attachment?: ForceBodyAttachment;
  readonly id: string;
  readonly site: ForceSite;
  /** Peak vector in newtons; a negative component reverses that axis. */
  readonly forceN: ForceVector;
  readonly startS: number;
  readonly durationS: number;
  readonly waveform: ForceWaveform;
}

export function validateEditedForce(force: EditedForce): void {
  const a = force.attachment;
  if (a && (!Number.isSafeInteger(a.bodyIndex) || a.bodyIndex < 1 ||
    a.bodyIndex > 128 || a.localComOffsetM.length !== 3 ||
    !a.localComOffsetM.every(Number.isFinite) ||
    Math.hypot(...a.localComOffsetM) > 5)) {
    throw new RangeError("Punto de aplicación local inválido");
  }
  if (!force.id.trim() || force.id.length > 64 ||
      !FORCE_SITES.some(site => site.id === force.site) ||
      !["half-sine", "constant"].includes(force.waveform) ||
      force.forceN.length !== 3 ||
      ![...force.forceN, force.startS, force.durationS].every(Number.isFinite) ||
      Math.hypot(...force.forceN) > 10_000 ||
      force.startS < 0 || force.startS > 20 ||
      force.durationS <= 0 || force.durationS > 20 ||
      force.startS + force.durationS > 30) {
    throw new RangeError("Fuerza inválida: comprueba cuerpo, vector, inicio y duración (máx. 10.000 N)");
  }
}

export function validateForceSequence(forces: readonly EditedForce[]): void {
  if (forces.length < 1 || forces.length > 12) {
    throw new RangeError("El editor admite entre 1 y 12 fuerzas");
  }
  const ids = new Set<string>();
  for (const force of forces) {
    validateEditedForce(force);
    if (ids.has(force.id)) throw new RangeError("Hay identificadores de fuerza duplicados");
    ids.add(force.id);
  }
}

/** The pulse is half a sine wave; sustained forces hold their vector constant.
 * Endpoints are zero, avoiding residual force from floating-point rounding.
 */
export function editedForceAtTime(timeS: number, force: EditedForce): ForceVector {
  validateEditedForce(force);
  if (!Number.isFinite(timeS)) throw new RangeError("Tiempo de simulación inválido");
  const phase = (timeS - force.startS) / force.durationS;
  if (phase < 0 || phase >= 1 - 1e-12) return [0, 0, 0];
  const fraction = force.waveform === "constant" ? 1 :
    (phase <= 1e-12 ? 0 : Math.sin(Math.PI * phase));
  return force.forceN.map(component => component * fraction) as unknown as ForceVector;
}

/** Component-wise exact theoretical impulse; N·s, NOT a prediction of motion. */
export function editedForceImpulseNs(force: EditedForce): ForceVector {
  validateEditedForce(force);
  const integral = force.durationS * (force.waveform === "constant" ? 1 : 2 / Math.PI);
  return force.forceN.map(component => component * integral) as unknown as ForceVector;
}

/** Allow two seconds after the latest force to observe balance/recovery. */
export function editedSequenceHorizonS(forces: readonly EditedForce[]): number {
  validateForceSequence(forces);
  return Math.max(6, ...forces.map(f => f.startS + f.durationS + 2));
}
