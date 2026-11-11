/**
 * Rangos articulares del humanoide generado (`generate.ts`), en radianes.
 *
 * Valores de goniometría clínica de adultos (orden de magnitud de las tablas de la AAOS y de Norkin y White).
 * PENDIENTE DE VERIFICAR CON FUENTE: no se han consultado las tablas originales al escribir este fichero; ver
 * `docs/UKE_MODEL.md`. Convención de signos: la del modelo v2 (hinge con el eje de `generate.ts`).
 *
 *  - Eje +Y (pitch): positivo = cabeza/tronco/cuello hacia delante (flexión), miembro colgante hacia atrás (extensión
 *    de cadera y hombro), planta del pie hacia abajo (flexión plantar). La rodilla usa el eje -Y.
 *  - Eje +X (roll): positivo = el miembro del lado izquierdo se aleja de la línea media. Por eso los rangos de roll
 *    y yaw de las articulaciones pares son espejo entre izquierda y derecha.
 */

const DEG = Math.PI / 180;

export type Range = readonly [number, number];

interface RomEntry {
  /** Rango en grados del lado izquierdo (o de la articulación impar). */
  readonly deg: Range;
  /** Si es verdadero, el lado derecho es el espejo `[-max, -min]`. */
  readonly mirrored: boolean;
}

const e = (min: number, max: number, mirrored = false): RomEntry => ({ deg: [min, max], mirrored });

/** Rangos del lado izquierdo (o impares), en grados. */
const ROM_DEG: Readonly<Record<string, RomEntry>> = {
  lumbar_pitch: e(-25, 60),
  lumbar_roll: e(-25, 25),
  thorax_pitch: e(-20, 35),
  thorax_roll: e(-20, 20),
  thorax_yaw: e(-40, 40),
  neck_pitch: e(-45, 45),
  neck_roll: e(-40, 40),
  neck_yaw: e(-80, 80),
  shoulder_pitch: e(-178, 60), // flexión 180°, extensión 60°
  shoulder_roll: e(-45, 178, true), // aducción 45°, abducción 180° (izquierdo: + = abducción)
  shoulder_yaw: e(-70, 90, true), // rotación interna 70°, externa 90° (izquierdo: + = externa)
  elbow: e(-150, 0), // flexión 150°, sin hiperextensión
  hip_pitch: e(-120, 30), // flexión 120°, extensión 30°
  hip_roll: e(-30, 45, true), // aducción 30°, abducción 45°
  hip_yaw: e(-45, 45, true),
  knee: e(-140, 0), // flexión 140°
  ankle_pitch: e(-30, 50), // dorsiflexión 30° (en carga), flexión plantar 50°
  ankle_roll: e(-35, 15, true), // izquierdo: + = eversión; inversión 35°, eversión 15°
};

const SIDED = new Set([
  "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "hip_pitch", "hip_roll", "hip_yaw", "knee",
  "ankle_pitch", "ankle_roll",
]);

/** Nombres de articulación del modelo generado -> clave de la tabla. */
function keyOf(joint: string): { key: string; right: boolean } {
  const m = /^(left|right)_(.+)$/.exec(joint);
  if (m && SIDED.has(m[2]!)) return { key: m[2]!, right: m[1] === "right" };
  return { key: joint, right: false };
}

/** Rango [min, max] en radianes de una articulación del modelo generado. Lanza si el nombre no existe. */
export function jointRangeRad(joint: string): Range {
  const { key, right } = keyOf(joint);
  const entry = ROM_DEG[key];
  if (!entry) throw new RangeError(`Articulación sin rango: ${joint}`);
  const [lo, hi] = entry.deg;
  return right && entry.mirrored ? [-hi * DEG, -lo * DEG] : [lo * DEG, hi * DEG];
}

/** Atributo `range="a b"` de MJCF. */
export function rangeAttr(joint: string): string {
  const [lo, hi] = jointRangeRad(joint);
  return `${Number(lo.toFixed(4))} ${Number(hi.toFixed(4))}`;
}
