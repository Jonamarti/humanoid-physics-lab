import { angularDifferenceDeg } from "../metrics/outcome";
import type { ExpectSpec, Outcome } from "./schema";

/** Lo que `compareExpect` necesita del resumen de una grabación. */
export interface ExpectInput {
  outcome: Outcome;
  direction_deg: number | null;
}

/**
 * Compara el resumen con `expect`. Devuelve `"pass"` o la lista de diferencias (en español).
 * La dirección solo se comprueba si `expect.direction_deg` está presente.
 */
export function compareExpect(summary: ExpectInput, expect: ExpectSpec): "pass" | string[] {
  const diffs: string[] = [];
  if (summary.outcome !== expect.outcome) {
    diffs.push(`resultado esperado "${expect.outcome}", obtenido "${summary.outcome}"`);
  }
  if (expect.direction_deg !== undefined) {
    const tol = expect.tolerance_deg ?? 0;
    if (summary.direction_deg === null) {
      diffs.push(`dirección esperada ${expect.direction_deg} ± ${tol} grados, pero no hay dirección de caída`);
    } else if (angularDifferenceDeg(summary.direction_deg, expect.direction_deg) > tol) {
      diffs.push(
        `dirección esperada ${expect.direction_deg} ± ${tol} grados, obtenida ${summary.direction_deg.toFixed(1)}`,
      );
    }
  }
  return diffs.length === 0 ? "pass" : diffs;
}
