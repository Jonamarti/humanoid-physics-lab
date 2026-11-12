/**
 * Métricas de equilibrio del humanoide: funciones puras (sin MuJoCo), issue #75 PR 4. El cálculo de referencia es el de
 * `src/control/balanceSensors.ts` (que no se modifica): punto de captura `xi = c + v / w0` con `w0 = sqrt(g / z_c)`,
 * envolvente convexa de los puntos de apoyo y distancia con signo. Aquí el polígono sale de los contactos de los pies
 * con carga > 20 N y el margen del punto de captura tiene signo (positivo dentro, negativo fuera).
 */

export type XY = readonly [number, number];
export type XYZ = readonly [number, number, number];

/** Aceleración de la gravedad, en m/s². */
export const GRAVITY_MPS2 = 9.81;
/** Un contacto de un pie cuenta para el polígono de apoyo si su carga normal supera este valor (N). */
export const SUPPORT_MIN_LOAD_N = 20;
/** Altura mínima del centro de masas (m) que se usa en `w0`, para no dividir por casi cero con el cuerpo en el suelo. */
export const MIN_COM_HEIGHT_M = 0.1;

/** Centro de masas y su velocidad de un conjunto de cuerpos (masa, posición del centro de masas del cuerpo y su velocidad). */
export function centerOfMass(
  bodies: ReadonlyArray<{ mass: number; pos: XYZ; vel?: XYZ }>,
): { com: XYZ; velocity: XYZ; mass: number } {
  let m = 0;
  const c = [0, 0, 0];
  const v = [0, 0, 0];
  for (const b of bodies) {
    m += b.mass;
    for (let k = 0; k < 3; k++) {
      c[k]! += b.mass * b.pos[k]!;
      v[k]! += b.mass * (b.vel?.[k] ?? 0);
    }
  }
  if (!(m > 0)) throw new RangeError("la masa total debe ser positiva");
  return { com: [c[0]! / m, c[1]! / m, c[2]! / m], velocity: [v[0]! / m, v[1]! / m, v[2]! / m], mass: m };
}

/** `w0 = sqrt(g / z_c)` del péndulo invertido lineal, en 1/s. */
export function captureOmega(comHeightM: number, gravity = GRAVITY_MPS2): number {
  return Math.sqrt(gravity / Math.max(comHeightM, MIN_COM_HEIGHT_M));
}

/** Punto de captura `xi = c + v / w0` en el plano horizontal (`z_c` es la altura del centro de masas sobre el suelo). */
export function capturePoint(com: XYZ, velocity: XYZ, gravity = GRAVITY_MPS2): XY {
  const w0 = captureOmega(com[2], gravity);
  return [com[0] + velocity[0] / w0, com[1] + velocity[1] / w0];
}

/** Un vértice cuyo producto vectorial con la arista vecina no supera esto (m², del orden de 3 nm de desvío en un lado de 30 cm) se considera alineado y se descarta. */
const HULL_COLLINEAR_EPS = 1e-9;

function cross(o: XY, a: XY, b: XY): number {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

/** Envolvente convexa (cadena monótona) en sentido antihorario, sin puntos repetidos. Con 0, 1 o 2 puntos distintos los devuelve tal cual. */
export function convexHull(points: readonly XY[]): XY[] {
  const sorted = [...points].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const unique: XY[] = [];
  for (const p of sorted) {
    const last = unique[unique.length - 1];
    if (!last || last[0] !== p[0] || last[1] !== p[1]) unique.push(p);
  }
  if (unique.length <= 2) return unique;
  const build = (pts: readonly XY[]): XY[] => {
    const out: XY[] = [];
    for (const p of pts) {
      while (out.length >= 2 && cross(out[out.length - 2]!, out[out.length - 1]!, p) <= HULL_COLLINEAR_EPS) out.pop();
      out.push(p);
    }
    out.pop();
    return out;
  };
  const lower = build(unique);
  const upper = build([...unique].reverse());
  return [...lower, ...upper];
}

/** Un contacto: cuerpos que se tocan, punto y fuerza (en el mundo) del contacto. */
export interface ContactLike {
  body_a: string;
  body_b: string;
  point_m: XYZ;
  force_n: XYZ;
}

/**
 * Polígono de apoyo: envolvente convexa de los puntos de contacto entre el suelo (`world`) y un pie (`isFoot`) cuya carga
 * normal (componente vertical de la fuerza) supera `minLoadN`. Vacío si ningún pie está cargado.
 */
export function supportPolygon(
  contacts: readonly ContactLike[],
  isFoot: (body: string) => boolean,
  minLoadN = SUPPORT_MIN_LOAD_N,
): XY[] {
  const points: XY[] = [];
  for (const c of contacts) {
    const foot = c.body_a === "world" ? c.body_b : c.body_b === "world" ? c.body_a : null;
    if (foot === null || !isFoot(foot)) continue;
    if (Math.abs(c.force_n[2]) <= minLoadN) continue;
    points.push([c.point_m[0], c.point_m[1]]);
  }
  return convexHull(points);
}

function distanceToSegment(p: XY, a: XY, b: XY): number {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const len2 = dx * dx + dy * dy;
  const u = len2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2)) : 0;
  return Math.hypot(p[0] - (a[0] + u * dx), p[1] - (a[1] + u * dy));
}

/**
 * Margen con signo de un punto al polígono (envolvente convexa antihoraria): distancia al borde más cercano, positiva si
 * el punto está dentro y negativa si está fuera. Con un polígono degenerado (un punto o un segmento) el punto nunca está
 * «dentro»: el margen es 0 o negativo. `null` si el polígono está vacío (ningún pie cargado).
 */
export function signedMargin(p: XY, polygon: readonly XY[]): number | null {
  const n = polygon.length;
  if (n === 0) return null;
  if (n === 1) return -Math.hypot(p[0] - polygon[0]![0], p[1] - polygon[0]![1]);
  if (n === 2) return -distanceToSegment(p, polygon[0]!, polygon[1]!);
  let nearest = Infinity;
  let inside = true;
  for (let i = 0; i < n; i++) {
    const a = polygon[i]!, b = polygon[(i + 1) % n]!;
    if (cross(a, b, p) < 0) inside = false;
    nearest = Math.min(nearest, distanceToSegment(p, a, b));
  }
  return inside ? nearest : -nearest;
}

/** Las ocho direcciones, de 45 en 45 grados desde el frente (0) hacia la izquierda (+90). */
export const OCTANT_NAMES = [
  "front", "front_left", "left", "back_left", "back", "back_right", "right", "front_right",
] as const;
export type Octant = (typeof OCTANT_NAMES)[number];

/** Ángulo en grados normalizado a (-180, 180]. */
export function wrapDeg(deg: number): number {
  let d = ((deg + 180) % 360 + 360) % 360 - 180;
  if (d === -180) d = 180;
  return d;
}

/** Cuantiza un ángulo (grados, 0 = frente) a una de las 8 direcciones: devuelve el ángulo múltiplo de 45 en (-180, 180] y su nombre. */
export function octantOf(angleDeg: number): { direction_deg: number; name: Octant } {
  const index = ((Math.round(angleDeg / 45) % 8) + 8) % 8;
  return { direction_deg: wrapDeg(index * 45), name: OCTANT_NAMES[index]! };
}

/** Centroide (de área) de un polígono convexo; con menos de 3 vértices (o área nula), la media de los puntos. `null` si está vacío. */
export function polygonCentroid(polygon: readonly XY[]): XY | null {
  const n = polygon.length;
  if (n === 0) return null;
  const mean: XY = [polygon.reduce((s, p) => s + p[0], 0) / n, polygon.reduce((s, p) => s + p[1], 0) / n];
  if (n < 3) return mean;
  let area2 = 0, cx = 0, cy = 0;
  for (let i = 0; i < n; i++) {
    const a = polygon[i]!, b = polygon[(i + 1) % n]!;
    const w = a[0] * b[1] - b[0] * a[1];
    area2 += w;
    cx += (a[0] + b[0]) * w;
    cy += (a[1] + b[1]) * w;
  }
  return Math.abs(area2) < 1e-12 ? mean : [cx / (3 * area2), cy / (3 * area2)];
}

/**
 * Dirección (una de 8, relativa al frente inicial `headingRad`) en que el punto de captura sale del apoyo: la del vector
 * desde el centroide de `polygon` hasta el punto. Con el polígono vacío (ningún pie cargado), desde el centro de masas.
 * Si el punto coincide con el origen del vector, la dirección es 0 (frente). Conviene pasar el último polígono en cuyo interior
 * estuvo el punto y no el actual: al levantarse un talón el polígono se reduce a un segmento y su centroide salta de sitio.
 */
export function unbalanceDirection(
  xcom: XY, polygon: readonly XY[], com: XYZ, headingRad: number,
): { direction_deg: number; name: Octant } {
  const origin = polygonCentroid(polygon) ?? [com[0], com[1]];
  const angle = (Math.atan2(xcom[1] - origin[1], xcom[0] - origin[0]) - headingRad) * (180 / Math.PI);
  return octantOf(wrapDeg(angle));
}
