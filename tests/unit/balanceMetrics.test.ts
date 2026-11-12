import { describe, expect, it } from "vitest";
import {
  GRAVITY_MPS2, SUPPORT_MIN_LOAD_N, capturePoint, captureOmega, centerOfMass, convexHull, octantOf, polygonCentroid,
  signedMargin, supportPolygon, unbalanceDirection, wrapDeg, type ContactLike, type XY,
} from "../../src/metrics/balance";

const SQUARE: XY[] = [[0, 0], [1, 0], [1, 1], [0, 1]]; // antihorario

describe("centro de masas", () => {
  it("promedia posiciones y velocidades por masa", () => {
    const r = centerOfMass([
      { mass: 1, pos: [0, 0, 0], vel: [1, 0, 0] },
      { mass: 3, pos: [4, 0, 2], vel: [0, 2, 0] },
    ]);
    expect(r.mass).toBe(4);
    expect(r.com).toEqual([3, 0, 1.5]);
    expect(r.velocity).toEqual([0.25, 1.5, 0]);
  });

  it("sin velocidades la velocidad es cero; masa total no positiva es un error", () => {
    expect(centerOfMass([{ mass: 2, pos: [1, 2, 3] }]).velocity).toEqual([0, 0, 0]);
    expect(() => centerOfMass([])).toThrow(RangeError);
  });
});

describe("punto de captura", () => {
  it("xi = c + v / w0 con w0 = sqrt(g / z_c)", () => {
    const zc = 1.0;
    const w0 = Math.sqrt(GRAVITY_MPS2 / zc);
    expect(captureOmega(zc)).toBeCloseTo(w0, 12);
    const xi = capturePoint([0.1, -0.2, zc], [0.5, 0.25, 9], GRAVITY_MPS2);
    expect(xi[0]).toBeCloseTo(0.1 + 0.5 / w0, 12);
    expect(xi[1]).toBeCloseTo(-0.2 + 0.25 / w0, 12);
  });

  it("en reposo coincide con la proyección del centro de masas; la velocidad vertical no cuenta", () => {
    expect(capturePoint([0.3, 0.4, 1.1], [0, 0, 3])).toEqual([0.3, 0.4]);
  });

  it("con el centro de masas casi en el suelo no divide por cero", () => {
    const xi = capturePoint([0, 0, 0], [1, 0, 0]);
    expect(Number.isFinite(xi[0])).toBe(true);
  });
});

describe("envolvente convexa", () => {
  it("devuelve los vértices en sentido antihorario y descarta los interiores y los alineados", () => {
    const hull = convexHull([[0, 0], [1, 1], [1, 0], [0, 1], [0.5, 0.5], [0.5, 0], [0.5, 1]]);
    expect(hull).toHaveLength(4);
    expect(hull).toEqual(expect.arrayContaining(SQUARE));
    // Antihorario: área con signo positiva.
    let a2 = 0;
    hull.forEach((p, i) => { const q = hull[(i + 1) % hull.length]!; a2 += p[0] * q[1] - q[0] * p[1]; });
    expect(a2).toBeGreaterThan(0);
  });

  it("casos degenerados: vacío, un punto, repetidos, alineados", () => {
    expect(convexHull([])).toEqual([]);
    expect(convexHull([[1, 1]])).toEqual([[1, 1]]);
    expect(convexHull([[1, 1], [1, 1], [1, 1]])).toEqual([[1, 1]]);
    expect(convexHull([[0, 0], [1, 0], [2, 0], [0.3, 0]])).toEqual([[0, 0], [2, 0]]);
  });

  it("trata como alineados los puntos que se desvían nanómetros", () => {
    expect(convexHull([[0, 0], [1, 1e-12], [2, 0]])).toEqual([[0, 0], [2, 0]]);
  });
});

describe("polígono de apoyo", () => {
  const contact = (a: string, b: string, p: [number, number], fz: number): ContactLike =>
    ({ body_a: a, body_b: b, point_m: [p[0], p[1], 0], force_n: [0, 0, fz] });
  const isFoot = (b: string): boolean => b.endsWith("_foot");

  it("usa solo contactos suelo-pie con carga > 20 N, en cualquiera de los dos órdenes de cuerpos", () => {
    const poly = supportPolygon([
      contact("world", "left_foot", [0, 0], 100),
      contact("right_foot", "world", [1, 0], -100), // el pie es body_a: la fuerza sobre el suelo es negativa
      contact("world", "left_foot", [1, 1], 50),
      contact("world", "left_foot", [0, 1], 20), // = 20 N: no cuenta (hace falta > 20)
      contact("world", "left_foot", [-2, 3], SUPPORT_MIN_LOAD_N + 0.001),
      contact("world", "chest", [-3, -3], 500), // no es un pie
      contact("left_foot", "right_foot", [-4, -4], 500), // no toca el suelo
    ], isFoot);
    expect(poly).toEqual(expect.arrayContaining([[0, 0], [1, 0], [1, 1], [-2, 3]]));
    expect(poly).toHaveLength(4);
  });

  it("ningún pie cargado: polígono vacío", () => {
    expect(supportPolygon([contact("world", "left_foot", [0, 0], 5)], isFoot)).toEqual([]);
    expect(supportPolygon([], isFoot)).toEqual([]);
  });
});

describe("margen con signo", () => {
  it("positivo dentro (distancia al borde más cercano), negativo fuera, cero en el borde", () => {
    expect(signedMargin([0.5, 0.5], SQUARE)).toBeCloseTo(0.5, 12);
    expect(signedMargin([0.9, 0.5], SQUARE)).toBeCloseTo(0.1, 12);
    expect(signedMargin([1.3, 0.5], SQUARE)).toBeCloseTo(-0.3, 12);
    expect(signedMargin([2, 2], SQUARE)).toBeCloseTo(-Math.SQRT2, 12); // fuera por una esquina: distancia al vértice
    expect(signedMargin([1, 0.5], SQUARE)).toBeCloseTo(0, 12);
  });

  it("polígono degenerado: nunca «dentro» (margen <= 0); vacío: null", () => {
    expect(signedMargin([0.5, 0.5], [[0, 0], [1, 0]])).toBeCloseTo(-0.5, 12);
    expect(signedMargin([0.5, 0], [[0, 0], [1, 0]])).toBeCloseTo(0, 12);
    expect(signedMargin([3, 4], [[0, 0]])).toBeCloseTo(-5, 12);
    expect(signedMargin([0, 0], [])).toBeNull();
  });
});

describe("direcciones", () => {
  it("wrapDeg normaliza a (-180, 180]", () => {
    expect(wrapDeg(190)).toBeCloseTo(-170, 12);
    expect(wrapDeg(-190)).toBeCloseTo(170, 12);
    expect(wrapDeg(180)).toBe(180);
    expect(wrapDeg(-180)).toBe(180);
    expect(wrapDeg(720)).toBeCloseTo(0, 12);
  });

  it("octantOf cuantiza a múltiplos de 45 grados con nombre (0 = delante, +90 = izquierda)", () => {
    expect(octantOf(0)).toEqual({ direction_deg: 0, name: "front" });
    expect(octantOf(40)).toEqual({ direction_deg: 45, name: "front_left" });
    expect(octantOf(90)).toEqual({ direction_deg: 90, name: "left" });
    expect(octantOf(-170)).toEqual({ direction_deg: 180, name: "back" });
    expect(octantOf(179)).toEqual({ direction_deg: 180, name: "back" });
    expect(octantOf(-135)).toEqual({ direction_deg: -135, name: "back_right" });
    expect(octantOf(-90).name).toBe("right");
    expect(octantOf(-44).name).toBe("front_right");
  });

  it("el centroide de un polígono convexo es el de área, no la media de los vértices", () => {
    // Rectángulo con un vértice extra sobre un lado largo: la media de vértices se desplaza, el centroide no.
    const poly: XY[] = [[0, 0], [2, 0], [4, 0], [4, 1], [0, 1]];
    const c = polygonCentroid(poly)!;
    expect(c[0]).toBeCloseTo(2, 12);
    expect(c[1]).toBeCloseTo(0.5, 12);
    expect(polygonCentroid([])).toBeNull();
    expect(polygonCentroid([[1, 2], [3, 4]])).toEqual([2, 3]);
  });

  it("dirección de desequilibrio: desde el centroide del apoyo, relativa al frente de Uke", () => {
    const poly: XY[] = [[-1, -1], [1, -1], [1, 1], [-1, 1]]; // centroide (0, 0)
    const com = [0, 0, 1] as const;
    expect(unbalanceDirection([2, 0], poly, com, 0)).toEqual({ direction_deg: 0, name: "front" });
    expect(unbalanceDirection([-2, 0.1], poly, com, 0).name).toBe("back");
    expect(unbalanceDirection([0, 2], poly, com, 0).name).toBe("left");
    expect(unbalanceDirection([2, -2], poly, com, 0).name).toBe("front_right");
    // Con el frente de Uke mirando a +Y (90°), un punto en +Y está delante y uno en +X a su derecha.
    expect(unbalanceDirection([0, 2], poly, com, Math.PI / 2).name).toBe("front");
    expect(unbalanceDirection([2, 0], poly, com, Math.PI / 2).name).toBe("right");
    // Sin polígono, desde el centro de masas.
    expect(unbalanceDirection([0.5, 3], [], [0.5, 1, 1], 0).name).toBe("left");
  });
});
