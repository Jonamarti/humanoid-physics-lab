/** Visual cue only. This never changes the applied physical force. */
export function forceArrowLengthM(peakOrInstantaneousN: number): number {
  if (!Number.isFinite(peakOrInstantaneousN) || peakOrInstantaneousN < 0) {
    throw new RangeError("Magnitud de flecha inválida");
  }
  // Nonlinear display scale: shows large forces without covering the model.
  return peakOrInstantaneousN === 0 ? 0 : Math.min(1.35, 0.25 + 0.2 * Math.log1p(peakOrInstantaneousN / 50));
}
