/** Physics-derived, deterministic geometric observables for biped balance.
 * This module does not generate torques or pretend a projected foot is
 * necessarily bearing load. Actual contact forces are a later milestone.
 */
export type XY = readonly [number, number];
export type XYZ = readonly [number, number, number];
export type Matrix3 = readonly [number, number, number, number, number, number, number, number, number];

export interface FootGeom {
  readonly centerM: XYZ;
  readonly rotationWorld: Matrix3; // MuJoCo row-major 3x3
  readonly halfSizeM: XYZ;
}

export interface BalanceObservation {
  readonly timeS: number;
  readonly totalMassKg: number;
  readonly comWorldM: XYZ;
  readonly comVelocityWorldMps: XYZ;
  readonly feetWorldM: Readonly<{ left: XYZ; right: XYZ }>;
  /** 2D convex hull from geometrically near-ground feet; not a COP measurement. */
  readonly candidateSupportPolygonXY: readonly XY[];
  readonly contactCount: number; // total contacts, not individual-foot force
  readonly capturePointXY: XY;
  readonly comProjectionInsideCandidateSupport: boolean;
  readonly capturePointInsideCandidateSupport: boolean;
}

function finitePoint(p: readonly number[]): boolean {
  return p.length > 0 && p.every(Number.isFinite);
}

export function capturePointXY(com: XY, velocity: XY, heightM: number, gravity = 9.81): XY {
  if (!finitePoint(com) || !finitePoint(velocity) || !Number.isFinite(heightM) ||
      !Number.isFinite(gravity) || heightM <= 0 || gravity <= 0) {
    throw new RangeError("Entradas inválidas para capture point");
  }
  const t = Math.sqrt(heightM / gravity);
  return [com[0] + velocity[0] * t, com[1] + velocity[1] * t];
}

function cross(o: XY, a: XY, b: XY): number {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

/** Monotone-chain convex hull with counterclockwise vertices. */
export function convexHullXY(points: readonly XY[]): XY[] {
  if (points.some(p => !finitePoint(p))) throw new RangeError("Coordenadas no finitas");
  const sorted = [...points].sort((a, b) => (a[0] - b[0]) || (a[1] - b[1]));
  const unique = sorted.filter((p, i) => i === 0 || p[0] !== sorted[i - 1]![0] || p[1] !== sorted[i - 1]![1]);
  if (unique.length < 3) return unique;
  const lower: XY[] = [];
  for (const p of unique) {
    while (lower.length >= 2 && cross(lower[lower.length - 2]!, lower[lower.length - 1]!, p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper: XY[] = [];
  for (let i = unique.length - 1; i >= 0; i--) {
    const p = unique[i]!;
    while (upper.length >= 2 && cross(upper[upper.length - 2]!, upper[upper.length - 1]!, p) <= 0) upper.pop();
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  return [...lower, ...upper];
}

export function insideConvexHullXY(p: XY, hull: readonly XY[], epsilon = 1e-8): boolean {
  if (!finitePoint(p) || !Number.isFinite(epsilon) || epsilon < 0) throw new RangeError("Entrada no finita");
  if (hull.length < 3) return false;
  for (let i = 0; i < hull.length; i++) {
    if (cross(hull[i]!, hull[(i + 1) % hull.length]!, p) < -epsilon) return false;
  }
  return true;
}

/** Project the floor-facing rectangle of a foot box on XY, provided its
 * lowest corner is close enough to z=0 to count as a support CANDIDATE.
 * A candidate is not evidence of nonzero normal load.
 */
export function candidateFootCornersXY(foot: FootGeom, maxBottomGapM = 0.015): XY[] {
  const { centerM: p, rotationWorld: m, halfSizeM: s } = foot;
  if (![...p, ...m, ...s, maxBottomGapM].every(Number.isFinite) ||
      s.some(v => v <= 0) || maxBottomGapM < 0) {
    throw new RangeError("Pie geométrico inválido");
  }
  const bottomZ = p[2] - Math.abs(m[6]) * s[0] - Math.abs(m[7]) * s[1] - Math.abs(m[8]) * s[2];
  if (bottomZ > maxBottomGapM) return [];
  const pts: XY[] = [];
  for (const x of [-s[0], s[0]]) {
    for (const y of [-s[1], s[1]]) {
      pts.push([p[0] + m[0] * x + m[1] * y, p[1] + m[3] * x + m[4] * y]);
    }
  }
  return pts;
}

export function buildBalanceObservation(input: {
  timeS: number;
  totalMassKg: number;
  comWorldM: XYZ;
  comVelocityWorldMps: XYZ;
  left: FootGeom;
  right: FootGeom;
  contactCount: number;
}): BalanceObservation {
  const { comWorldM: com, comVelocityWorldMps: vel, left, right } = input;
  if (![input.timeS, input.totalMassKg, input.contactCount, ...com, ...vel].every(Number.isFinite) ||
      input.totalMassKg <= 0 || input.contactCount < 0 || com[2] <= 0) {
    throw new RangeError("Medidas de equilibrio no finitas");
  }
  const polygon = convexHullXY([
    ...candidateFootCornersXY(left), ...candidateFootCornersXY(right),
  ]);
  const capture = capturePointXY([com[0], com[1]], [vel[0], vel[1]], com[2]);
  return {
    timeS: input.timeS,
    totalMassKg: input.totalMassKg,
    comWorldM: [...com] as XYZ,
    comVelocityWorldMps: [...vel] as XYZ,
    feetWorldM: { left: [...left.centerM] as XYZ, right: [...right.centerM] as XYZ },
    candidateSupportPolygonXY: polygon,
    contactCount: input.contactCount,
    capturePointXY: capture,
    comProjectionInsideCandidateSupport: insideConvexHullXY([com[0], com[1]], polygon),
    capturePointInsideCandidateSupport: insideConvexHullXY(capture, polygon),
  };
}
