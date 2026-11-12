/**
 * Cuerpo de Tori (`docs/TORI_BODY.md`). Módulo puro: no importa MuJoCo ni el ejecutor.
 *
 * Tori es una marioneta de segmentos SUELTOS (cada uno un cuerpo libre con una cápsula o una esfera) que siguen una secuencia de
 * posturas con fuerza y par LIMITADOS, igual que el obstáculo de `obstacle.ts`: si Uke empuja más que el tope, el segmento
 * cede. No hay articulaciones entre segmentos: se mantienen unidos porque sus objetivos salen del mismo esqueleto. Tampoco
 * hay equilibrio propio: que Tori pudiera sostener esas fuerzas lo juzga el presupuesto (`toriBudget.ts`).
 *
 * La postura se describe con pocos números por instante (pelvis, orientación, inclinación del tronco y pies); las piernas y los
 * brazos salen por cinemática inversa de dos tramos. Las manos van a los puntos de agarre que se le indiquen.
 *
 * Unidades SI; vectores en el mundo (+X delante de Uke, +Y izquierda, +Z arriba); ángulos de la postura en grados.
 */
import { minimumJerk, type Vec3 } from "./hand";

export type { Vec3 };

export const TORI_BODY_PREFIX = "tori_";
export const GRAVITY_MPS2 = 9.81;
const REFERENCE_HEIGHT_M = 1.76;
const DEG = Math.PI / 180;

export const TORI_BODY_DEFAULTS = Object.freeze({
  /** Rozamiento de la tela de Tori contra Uke. Sin calibrar (el mismo punto de partida que el obstáculo). */
  friction: 0.8,
  /** Frecuencia natural del seguimiento de cada segmento, rad/s (amortiguación crítica). */
  tracking_rad_per_s: 40,
  /** Brazo de palanca con el que el tope de par sale del tope de fuerza horizontal, m. */
  torque_arm_m: 0.25,
});

/** Segmentos, en el orden en que se añaden al modelo. */
export const TORI_SEGMENTS = [
  "pelvis", "trunk", "shoulders", "head",
  "left_upperarm", "left_forearm", "right_upperarm", "right_forearm",
  "left_thigh", "left_shin", "left_foot", "right_thigh", "right_shin", "right_foot",
] as const;
export type ToriSegment = (typeof TORI_SEGMENTS)[number];

interface SegmentDef {
  /** Fracción de la masa de Tori. */
  mass: number;
  /** Radio a la altura de referencia, m. */
  radius: number;
  /** Topes en pesos de Tori: horizontal (módulo en XY) y vertical. */
  cap_h: number;
  cap_v: number;
  /** Los brazos no chocan: la fuerza de las manos ya la ponen los efectores `pull` / `hand`. */
  collides: boolean;
}

const SEGMENT_DEFS: Readonly<Record<ToriSegment, SegmentDef>> = {
  pelvis: { mass: 0.14, radius: 0.11, cap_h: 0.8, cap_v: 2.0, collides: true },
  trunk: { mass: 0.3, radius: 0.13, cap_h: 0.8, cap_v: 2.0, collides: true },
  shoulders: { mass: 0.08, radius: 0.075, cap_h: 0.6, cap_v: 1.5, collides: true },
  head: { mass: 0.07, radius: 0.095, cap_h: 0.2, cap_v: 0.3, collides: true },
  left_upperarm: { mass: 0.03, radius: 0.045, cap_h: 0.3, cap_v: 0.4, collides: false },
  left_forearm: { mass: 0.025, radius: 0.04, cap_h: 0.3, cap_v: 0.4, collides: false },
  right_upperarm: { mass: 0.03, radius: 0.045, cap_h: 0.3, cap_v: 0.4, collides: false },
  right_forearm: { mass: 0.025, radius: 0.04, cap_h: 0.3, cap_v: 0.4, collides: false },
  left_thigh: { mass: 0.1, radius: 0.07, cap_h: 0.6, cap_v: 1.5, collides: true },
  left_shin: { mass: 0.05, radius: 0.05, cap_h: 0.5, cap_v: 1.5, collides: true },
  left_foot: { mass: 0.015, radius: 0.04, cap_h: 0.5, cap_v: 2.0, collides: true },
  right_thigh: { mass: 0.1, radius: 0.07, cap_h: 0.6, cap_v: 1.5, collides: true },
  right_shin: { mass: 0.05, radius: 0.05, cap_h: 0.5, cap_v: 1.5, collides: true },
  right_foot: { mass: 0.015, radius: 0.04, cap_h: 0.5, cap_v: 2.0, collides: true },
};

/** Postura de Tori en un instante. Entre posturas se interpola con mínimo tirón; fuera, se mantiene la más cercana. */
export interface ToriPoseKey {
  readonly t_s: number;
  /** Punto medio entre las dos caderas, en el mundo. */
  readonly pelvis_m: Vec3;
  /** Hacia dónde mira la pelvis: 0° = +X, 90° = +Y. No se reduce a ±180°: 180 → 360 es media vuelta a su izquierda. */
  readonly yaw_deg: number;
  /** Inclinación del tronco hacia delante (0° = vertical, 90° = horizontal). */
  readonly lean_deg?: number;
  /** Inclinación del tronco hacia su izquierda. */
  readonly side_lean_deg?: number;
  /** Giro de los hombros respecto a la pelvis (positivo = a su izquierda). */
  readonly twist_deg?: number;
  /** Apoyo de cada pie (bajo el tobillo): `[x, y, z]` con z = altura de la planta sobre el suelo. */
  readonly left_foot_m: Vec3;
  readonly right_foot_m: Vec3;
}

export interface ToriBodyConfig {
  mass_kg: number;
  height_m: number;
  friction?: number;
  keys: readonly ToriPoseKey[];
}

export interface ToriSegmentParams {
  readonly segment: ToriSegment;
  /** Nombre del cuerpo en el modelo: `tori_<segmento>`. */
  readonly name: string;
  readonly mass_kg: number;
  readonly radius_m: number;
  readonly max_horizontal_force_n: number;
  readonly max_vertical_force_n: number;
  readonly max_torque_nm: number;
  readonly collides: boolean;
}

export interface ToriBodyParams {
  readonly mass_kg: number;
  readonly height_m: number;
  readonly friction: number;
  readonly keys: readonly ToriPoseKey[];
  readonly segments: readonly ToriSegmentParams[];
  /** Longitudes del esqueleto, m. */
  readonly dims: Readonly<{
    thigh: number; shin: number; ankle: number; upper_arm: number; forearm: number; trunk: number; neck: number;
    hip_half: number; shoulder_half: number; head_radius: number; foot_back: number; foot_front: number;
    /** Altura de la pelvis de pie. */
    standing_pelvis: number;
  }>;
}

function add(a: Vec3, b: Vec3): Vec3 { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function sub(a: Vec3, b: Vec3): Vec3 { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function scale(a: Vec3, s: number): Vec3 { return [a[0] * s, a[1] * s, a[2] * s]; }
function dot(a: Vec3, b: Vec3): number { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a: Vec3, b: Vec3): Vec3 { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function norm(a: Vec3): number { return Math.sqrt(dot(a, a)); }
function unit(a: Vec3, fallback: Vec3): Vec3 {
  const n = norm(a);
  return n > 1e-9 ? scale(a, 1 / n) : fallback;
}
function lerp(a: number, b: number, u: number): number { return a + (b - a) * u; }
function lerp3(a: Vec3, b: Vec3, u: number): Vec3 { return [lerp(a[0], b[0], u), lerp(a[1], b[1], u), lerp(a[2], b[2], u)]; }
/** Gira `v` un ángulo `angle` (rad) alrededor del eje unitario `axis` (Rodrigues). */
function rotateAbout(v: Vec3, axis: Vec3, angle: number): Vec3 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return add(add(scale(v, c), scale(cross(axis, v), s)), scale(axis, dot(axis, v) * (1 - c)));
}

function finiteVec(v: unknown): v is Vec3 {
  return Array.isArray(v) && v.length === 3 && v.every((x) => typeof x === "number" && Number.isFinite(x));
}

/** Valida la configuración y deriva masas, radios y topes. Lanza `RangeError` con datos fuera de rango. */
export function resolveToriBodyParams(config: ToriBodyConfig): ToriBodyParams {
  const { mass_kg: m, height_m: h } = config;
  if (!(Number.isFinite(m) && m >= 40 && m <= 200)) throw new RangeError(`mass_kg: debe estar entre 40 y 200 kg (recibido ${m})`);
  if (!(Number.isFinite(h) && h >= 1.4 && h <= 2.2)) throw new RangeError(`height_m: debe estar entre 1,40 y 2,20 m (recibido ${h})`);
  const friction = config.friction ?? TORI_BODY_DEFAULTS.friction;
  if (!(Number.isFinite(friction) && friction >= 0)) throw new RangeError(`friction: debe ser >= 0 (recibido ${friction})`);
  if (!Array.isArray(config.keys) || config.keys.length === 0) throw new RangeError("keys: hace falta al menos una postura");
  config.keys.forEach((k, i) => {
    if (!(Number.isFinite(k.t_s) && k.t_s >= 0)) throw new RangeError(`keys[${i}].t_s: debe ser >= 0`);
    if (i > 0 && !(k.t_s > config.keys[i - 1]!.t_s)) throw new RangeError(`keys[${i}].t_s: debe ser mayor que el de la postura anterior`);
    for (const key of ["pelvis_m", "left_foot_m", "right_foot_m"] as const) {
      if (!finiteVec(k[key])) throw new RangeError(`keys[${i}].${key}: debe ser [x, y, z] con números finitos`);
    }
    if (!(k.pelvis_m[2] > 0.15)) throw new RangeError(`keys[${i}].pelvis_m: la pelvis debe quedar a más de 0,15 m del suelo`);
    if (k.left_foot_m[2] < 0 || k.right_foot_m[2] < 0) throw new RangeError(`keys[${i}]: los pies no pueden quedar bajo el suelo`);
    for (const key of ["yaw_deg", "lean_deg", "side_lean_deg", "twist_deg"] as const) {
      const v = k[key];
      if (v !== undefined && !Number.isFinite(v)) throw new RangeError(`keys[${i}].${key}: debe ser un número finito`);
    }
  });
  const hs = h / REFERENCE_HEIGHT_M;
  // Grosor: como el de Uke, crece con la raíz de la masa por unidad de altura.
  const thick = Math.sqrt((m / h) / (75 / REFERENCE_HEIGHT_M));
  const total = TORI_SEGMENTS.reduce((s, name) => s + SEGMENT_DEFS[name].mass, 0);
  const weight = m * GRAVITY_MPS2;
  const segments = TORI_SEGMENTS.map((segment): ToriSegmentParams => {
    const def = SEGMENT_DEFS[segment];
    return {
      segment,
      name: TORI_BODY_PREFIX + segment,
      mass_kg: (def.mass / total) * m,
      radius_m: def.radius * hs * (segment === "head" ? 1 : thick),
      max_horizontal_force_n: def.cap_h * weight,
      max_vertical_force_n: def.cap_v * weight,
      max_torque_nm: def.cap_h * weight * TORI_BODY_DEFAULTS.torque_arm_m,
      collides: def.collides,
    };
  });
  const dims = {
    thigh: 0.42 * hs, shin: 0.42 * hs, ankle: 0.07 * hs, upper_arm: 0.29 * hs, forearm: 0.3 * hs, trunk: 0.6 * hs, neck: 0.06 * hs,
    hip_half: 0.09 * hs, shoulder_half: 0.19 * hs, head_radius: 0.095 * hs, foot_back: 0.07 * hs, foot_front: 0.17 * hs,
    standing_pelvis: 0.91 * hs,
  };
  return { mass_kg: m, height_m: h, friction, keys: config.keys.map((k) => ({ ...k })), segments, dims };
}

/** Postura interpolada en `t` (mínimo tirón entre cada par de posturas). */
export function toriPoseAt(keys: readonly ToriPoseKey[], t: number): Required<ToriPoseKey> {
  const full = (k: ToriPoseKey): Required<ToriPoseKey> => ({
    ...k, lean_deg: k.lean_deg ?? 0, side_lean_deg: k.side_lean_deg ?? 0, twist_deg: k.twist_deg ?? 0,
  });
  const first = keys[0]!;
  if (t <= first.t_s) return full(first);
  for (let i = 1; i < keys.length; i++) {
    const b = keys[i]!;
    if (t > b.t_s) continue;
    const a = full(keys[i - 1]!);
    const fb = full(b);
    const u = minimumJerk((t - a.t_s) / (b.t_s - a.t_s));
    return {
      t_s: t,
      pelvis_m: lerp3(a.pelvis_m, fb.pelvis_m, u),
      yaw_deg: lerp(a.yaw_deg, fb.yaw_deg, u),
      lean_deg: lerp(a.lean_deg, fb.lean_deg, u),
      side_lean_deg: lerp(a.side_lean_deg, fb.side_lean_deg, u),
      twist_deg: lerp(a.twist_deg, fb.twist_deg, u),
      left_foot_m: lerp3(a.left_foot_m, fb.left_foot_m, u),
      right_foot_m: lerp3(a.right_foot_m, fb.right_foot_m, u),
    };
  }
  return full(keys[keys.length - 1]!);
}

/** Extremos de un segmento (cápsula de `a` a `b`; en la cabeza, `a` = `b` = centro). */
export interface SegmentEnds { readonly a: Vec3; readonly b: Vec3 }

/** Rodilla o codo: articulación intermedia de dos tramos `l1`, `l2` entre `root` y `end`, doblada hacia `bend`. */
function twoLink(root: Vec3, end: Vec3, l1: number, l2: number, bend: Vec3): { mid: Vec3; end: Vec3 } {
  const reach = l1 + l2 - 1e-4;
  let d = sub(end, root);
  let dist = norm(d);
  if (dist < 0.05) { d = [0, 0, -0.05]; dist = 0.05; }
  const dir = scale(d, 1 / dist);
  const D = Math.min(dist, reach);
  const along = (l1 * l1 - l2 * l2 + D * D) / (2 * D);
  const off = Math.sqrt(Math.max(0, l1 * l1 - along * along));
  const side = unit(sub(bend, scale(dir, dot(bend, dir))), unit(cross(dir, [0, 1, 0]), [1, 0, 0]));
  return { mid: add(root, add(scale(dir, along), scale(side, off))), end: add(root, scale(dir, D)) };
}

/** Puntos de agarre a los que van las muñecas (mundo). Sin punto, el brazo cuelga delante del cuerpo. */
export interface ToriHandTargets { left?: Vec3; right?: Vec3 }

/** Esqueleto de Tori en `t`: extremos de cada segmento en el mundo. */
export function toriSkeleton(params: ToriBodyParams, t: number, hands: ToriHandTargets = {}): Record<ToriSegment, SegmentEnds> {
  const d = params.dims;
  const pose = toriPoseAt(params.keys, t);
  const yaw = pose.yaw_deg * DEG;
  const fwd: Vec3 = [Math.cos(yaw), Math.sin(yaw), 0];
  const left: Vec3 = [-Math.sin(yaw), Math.cos(yaw), 0];
  const P = pose.pelvis_m;
  // Tronco: la vertical, inclinada hacia delante (giro alrededor de `left`) y hacia su izquierda (alrededor de `-fwd`).
  let up: Vec3 = rotateAbout([0, 0, 1], left, pose.lean_deg * DEG);
  up = rotateAbout(up, fwd, -pose.side_lean_deg * DEG);
  const trunkFwd = unit(cross(left, up), fwd);
  const shoulderLeft = rotateAbout(unit(cross(up, trunkFwd), left), up, pose.twist_deg * DEG);
  const chestFwd = unit(cross(shoulderLeft, up), trunkFwd);
  const neck = add(P, scale(up, d.trunk));
  const shoulderMid = add(P, scale(up, d.trunk * 0.9));
  const out = {} as Record<ToriSegment, SegmentEnds>;
  out.pelvis = { a: add(P, scale(left, d.hip_half * 1.3)), b: add(P, scale(left, -d.hip_half * 1.3)) };
  out.trunk = { a: add(P, scale(up, d.trunk * 0.2)), b: add(P, scale(up, d.trunk * 0.82)) };
  out.shoulders = { a: add(shoulderMid, scale(shoulderLeft, d.shoulder_half)), b: add(shoulderMid, scale(shoulderLeft, -d.shoulder_half)) };
  const headCenter = add(neck, scale(up, d.neck + d.head_radius));
  out.head = { a: headCenter, b: headCenter };
  for (const side of ["left", "right"] as const) {
    const sign = side === "left" ? 1 : -1;
    const hip = add(P, scale(left, sign * d.hip_half));
    const foot = side === "left" ? pose.left_foot_m : pose.right_foot_m;
    const leg = twoLink(hip, [foot[0], foot[1], foot[2] + d.ankle], d.thigh, d.shin, fwd);
    out[`${side}_thigh`] = { a: hip, b: leg.mid };
    out[`${side}_shin`] = { a: leg.mid, b: leg.end };
    // El pie: cápsula tumbada bajo el tobillo, apuntando hacia donde mira la pelvis.
    const radius = params.segments.find((s) => s.segment === `${side}_foot`)!.radius_m;
    const sole: Vec3 = [leg.end[0], leg.end[1], Math.max(leg.end[2] - d.ankle, 0) + radius + 0.005];
    out[`${side}_foot`] = { a: add(sole, scale(fwd, -d.foot_back)), b: add(sole, scale(fwd, d.foot_front)) };

    const shoulder = sign > 0 ? out.shoulders.a : out.shoulders.b;
    const rest = add(shoulder, add(scale(chestFwd, 0.3 * d.upper_arm), add(scale(up, -0.9 * d.upper_arm), scale(chestFwd, 0.6 * d.forearm))));
    const target = hands[side] ?? rest;
    // Codo hacia abajo y hacia fuera.
    const arm = twoLink(shoulder, target, d.upper_arm, d.forearm, add([0, 0, -1], scale(shoulderLeft, 0.6 * sign)));
    out[`${side}_upperarm`] = { a: shoulder, b: arm.mid };
    out[`${side}_forearm`] = { a: arm.mid, b: arm.end };
  }
  return out;
}

/** Longitud del tramo recto de la cápsula de cada segmento (constante: no depende de la postura). */
export function toriSegmentLength(params: ToriBodyParams, segment: ToriSegment): number {
  const d = params.dims;
  if (segment === "pelvis") return 2 * d.hip_half * 1.3;
  if (segment === "trunk") return d.trunk * 0.62;
  if (segment === "shoulders") return 2 * d.shoulder_half;
  if (segment === "head") return 0;
  if (segment.endsWith("thigh")) return d.thigh;
  if (segment.endsWith("shin")) return d.shin;
  if (segment.endsWith("foot")) return d.foot_back + d.foot_front;
  return segment.endsWith("upperarm") ? d.upper_arm : d.forearm;
}

/** Cuaternión `[w, x, y, z]` que lleva el eje Z al vector unitario `axis` (giro más corto). */
export function quatFromZ(axis: Vec3): [number, number, number, number] {
  const c = axis[2];
  if (c < -1 + 1e-9) return [0, 1, 0, 0];
  const w = Math.sqrt((1 + c) / 2);
  const k = 1 / (2 * w);
  return [w, -axis[1] * k, axis[0] * k, 0];
}

function f(x: number): string { return String(Number(x.toFixed(6))); }

/** Nombres de los cuerpos y de sus articulaciones libres, en el orden del modelo. */
export function toriBodyNames(params: ToriBodyParams): { bodies: string[]; joints: string[] } {
  return { bodies: params.segments.map((s) => s.name), joints: params.segments.map((s) => `${s.name}_free`) };
}

/**
 * MJCF de los cuerpos de Tori en la postura de t = 0 (para añadir al final de `<worldbody>` con `appendToWorldbody`).
 * Colisión: `contype="2" conaffinity="1"`, como los obstáculos (chocan con Uke, el suelo y los props; no entre sí ni con los
 * obstáculos). Los brazos no chocan con nada.
 */
export function buildToriBodyMjcf(params: ToriBodyParams, hands: ToriHandTargets = {}): string {
  const skeleton = toriSkeleton(params, 0, hands);
  const contact = `solref="0.015 1" solimp="0.9 0.95 0.001"`;
  return params.segments.map((s) => {
    const ends = skeleton[s.segment];
    const mid = scale(add(ends.a, ends.b), 0.5);
    const half = toriSegmentLength(params, s.segment) / 2;
    const quat = quatFromZ(unit(sub(ends.b, ends.a), [0, 0, 1]));
    const shape = half === 0 ? `type="sphere" size="${f(s.radius_m)}"` : `type="capsule" size="${f(s.radius_m)} ${f(half)}"`;
    const collide = s.collides ? `contype="2" conaffinity="1"` : `contype="0" conaffinity="0"`;
    const rgba = s.segment === "head" ? "0.93 0.8 0.7 1" : s.collides ? "0.95 0.93 0.86 1" : "0.95 0.93 0.86 0.55";
    return `    <body name="${s.name}" pos="${mid.map(f).join(" ")}" quat="${quat.map(f).join(" ")}">
      <freejoint name="${s.name}_free"/>
      <geom name="${s.name}_geom" ${shape} mass="${f(s.mass_kg)}" rgba="${rgba}" ${collide}
            friction="${f(params.friction)} 0.005 0.0001" ${contact}/>
    </body>
`;
  }).join("");
}

/** Estado de un segmento leído del motor. */
export interface ToriSegmentState {
  /** Centro del cuerpo (su centro de masas), mundo. */
  position_m: Vec3;
  velocity_mps: Vec3;
  /** Eje Z del cuerpo en el mundo (el de la cápsula). */
  axis: Vec3;
  angular_velocity_rps: Vec3;
  /** Inercia transversal y axial, kg·m². */
  inertia_transverse: number;
  inertia_axial: number;
}

export interface ToriSegmentCommand {
  force_n: Vec3;
  torque_nm: Vec3;
  /** Distancia al objetivo, m. */
  deviation_m: number;
  saturated: boolean;
}

/**
 * Orden de un segmento para el paso `[t, t + dt]`: muelle y amortiguador hacia el objetivo (con la velocidad del objetivo como
 * anticipo), más su peso, recortado a los topes. El par alinea el eje de la cápsula con el del objetivo.
 */
export function toriSegmentCommand(
  s: ToriSegmentParams, state: ToriSegmentState, now: SegmentEnds, next: SegmentEnds, dt: number,
): ToriSegmentCommand {
  const w = TORI_BODY_DEFAULTS.tracking_rad_per_s;
  const target = scale(add(now.a, now.b), 0.5);
  const targetNext = scale(add(next.a, next.b), 0.5);
  const targetVelocity = scale(sub(targetNext, target), 1 / dt);
  const error = sub(target, state.position_m);
  let force = add(scale(error, s.mass_kg * w * w), scale(sub(targetVelocity, state.velocity_mps), 2 * s.mass_kg * w));
  force = [force[0], force[1], force[2] + s.mass_kg * GRAVITY_MPS2];
  let saturated = false;
  const horizontal = Math.hypot(force[0], force[1]);
  if (horizontal > s.max_horizontal_force_n) {
    const k = s.max_horizontal_force_n / horizontal;
    force = [force[0] * k, force[1] * k, force[2]];
    saturated = true;
  }
  if (Math.abs(force[2]) > s.max_vertical_force_n) {
    force = [force[0], force[1], Math.sign(force[2]) * s.max_vertical_force_n];
    saturated = true;
  }

  const axisNow = unit(sub(now.b, now.a), state.axis);
  const axisNext = unit(sub(next.b, next.a), axisNow);
  const pointLike = norm(sub(now.b, now.a)) < 1e-9;
  const targetOmega = pointLike ? [0, 0, 0] as Vec3 : scale(cross(axisNow, axisNext), 1 / dt);
  const omegaError = sub(targetOmega, state.angular_velocity_rps);
  const axial = scale(state.axis, dot(omegaError, state.axis));
  const transverse = sub(omegaError, axial);
  // La amortiguación va por separado en cada inercia: con la transversal, el giro sobre el propio eje sería inestable.
  let torque = add(scale(transverse, 2 * state.inertia_transverse * w), scale(axial, 2 * state.inertia_axial * w));
  if (!pointLike) torque = add(torque, scale(cross(state.axis, axisNow), state.inertia_transverse * w * w));
  const torqueNorm = norm(torque);
  if (torqueNorm > s.max_torque_nm) {
    torque = scale(torque, s.max_torque_nm / torqueNorm);
    saturated = true;
  }
  return { force_n: force, torque_nm: torque, deviation_m: norm(error), saturated };
}

/**
 * Secuencia de posturas de partida: Tori empieza de frente a Uke (que está en el origen mirando a +X), entra girando media
 * vuelta a su izquierda hasta darle la espalda con las rodillas flexionadas, carga, y estira las piernas inclinando el tronco.
 * Los tiempos salen de una demostración a velocidad real grabada de perfil a 25 fps (ver `docs/TORI_BODY.md`): entrada 0,55 s,
 * giro 0,25 s, carga 0,2 s y proyección 0,35 s. Es un punto de partida editable, no una técnica validada.
 */
export function defaultToriKeys(toriHeightM: number, startS = 0.2): ToriPoseKey[] {
  const hs = toriHeightM / REFERENCE_HEIGHT_M;
  const stand = 0.91 * hs;
  const r = (x: number): number => Number(x.toFixed(3));
  const key = (
    dt: number, x: number, z: number, yaw: number, lean: number, lf: [number, number], rf: [number, number], twist = 0,
  ): ToriPoseKey => ({
    t_s: r(startS + dt), pelvis_m: [r(x), 0, r(z * stand)], yaw_deg: yaw, lean_deg: lean, twist_deg: twist,
    left_foot_m: [r(lf[0]), r(lf[1]), 0], right_foot_m: [r(rf[0]), r(rf[1]), 0],
  });
  return [
    key(0, 0.62, 0.98, 180, 5, [0.64, -0.13], [0.64, 0.13]),
    key(0.25, 0.56, 0.93, 200, 8, [0.64, -0.13], [0.5, 0.08]),
    key(0.45, 0.5, 0.78, 360, 20, [0.5, 0.14], [0.5, -0.14]),
    key(0.65, 0.4, 0.66, 360, 40, [0.42, 0.14], [0.42, -0.14]),
    key(1.0, 0.32, 0.95, 360, 95, [0.42, 0.14], [0.42, -0.14], 15),
    key(1.6, 0.36, 0.95, 360, 60, [0.42, 0.14], [0.42, -0.14]),
  ];
}
