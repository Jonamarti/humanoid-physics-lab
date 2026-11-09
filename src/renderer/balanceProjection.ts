import { insideConvexHullXY, type XY, type XYZ } from "../control/balanceSensors";

/** Loaded foot hull is a geometric envelope, NOT a measured CoP or proof
 * of dynamic stability. If no feet have verified load, return "unknown".
 */
export type ProjectionStatus = "inside" | "outside" | "unknown";

export interface BalanceProjection {
  readonly groundPoint: XYZ;
  readonly status: ProjectionStatus;
}

export function projectBalanceToGround(
  comWorldM: XYZ,
  loadedSupportPolygonXY: readonly XY[],
): BalanceProjection {
  if (!comWorldM.every(Number.isFinite) ||
      loadedSupportPolygonXY.some(p => p.length !== 2 || !p.every(Number.isFinite))) {
    throw new RangeError("Centro de masas o base de apoyo no finitos");
  }
  const status = loadedSupportPolygonXY.length < 3 ? "unknown" :
    insideConvexHullXY([comWorldM[0], comWorldM[1]], loadedSupportPolygonXY)
      ? "inside" : "outside";
  return {
    groundPoint: [comWorldM[0], comWorldM[1], 0],
    status,
  };
}
