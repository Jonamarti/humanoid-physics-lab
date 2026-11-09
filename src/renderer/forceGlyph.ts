import type { ForceVector } from "../physics/forceSequence";

export type ForceGlyph = "arrow" | "into-screen" | "out-of-screen" | "none";

/** View-relative textbook notation: ⊗ points AWAY from the viewer, ⊙ toward.
 * Works for any orbit angle; NEVER changes the physical world-space force.
 */
export function classifyForceGlyph(
  forceWorldN: ForceVector,
  towardCameraWorld: ForceVector,
  endOnThreshold = 0.86,
): ForceGlyph {
  if (![...forceWorldN, ...towardCameraWorld, endOnThreshold].every(Number.isFinite) ||
      endOnThreshold < 0 || endOnThreshold > 1) {
    throw new RangeError("Vector de visualización inválido");
  }
  const forceMagnitude = Math.hypot(...forceWorldN);
  if (forceMagnitude <= 1e-12) return "none";
  const cameraMagnitude = Math.hypot(...towardCameraWorld);
  if (cameraMagnitude <= 1e-12) throw new RangeError("Cámara sin orientación");
  const dot = forceWorldN.reduce(
    (sum, v, i) => sum + v * towardCameraWorld[i]!, 0,
  ) / (forceMagnitude * cameraMagnitude);
  return dot >= endOnThreshold ? "out-of-screen" :
    dot <= -endOnThreshold ? "into-screen" : "arrow";
}
