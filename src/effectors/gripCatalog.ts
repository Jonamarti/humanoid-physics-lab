/**
 * Catálogo de agarres (issue #76, PR 2). Cada agarre semántico dice el tipo de unión de la mano virtual por defecto y
 * cómo se reparte la fuerza entre puntos del cuerpo de Uke. Documentación y razón de cada reparto: `docs/GRIP_CATALOG.md`.
 *
 * Módulo autónomo: no depende del esquema de escenarios, del ejecutor ni de MuJoCo. Solo importa tipos y
 * `applyForceAtPoint` de `./hand`. «Izquierda» y «derecha» son el lado de Uke (los nombres de sitio del humanoide), no
 * el de la mano de Tori.
 *
 * Uso previsto desde un ejecutor, una vez por paso físico y antes de `mj_step` (tras `mj_kinematics`, `mj_comPos` y
 * `mj_comVel`):
 *
 *   const grip = locateGrip(model, resolveGrip("lapel_left"));
 *   const params = resolveHandParams(handConfigForGrip(grip, { shoulder_m, phases }));
 *   // en cada paso:
 *   const { points_m, velocities_mps } = gripPointStates(model, data, grip);
 *   const eq = equivalentGripPoint(grip.shares, points_m, velocities_mps);
 *   const r = stepHand(params, state, { t, dt, grip_m: eq.point_m, grip_velocity_mps: eq.velocity_mps });
 *   applyGripForce(data, grip, r.force_n);   // reparte r.force_n; la suma de lo aplicado es r.force_n
 */
import { applyForceAtPoint, type HandConfig, type HandLink, type Vec3, type WrenchData } from "./hand";

export type { Vec3 } from "./hand";

/** Nombres de los agarres del catálogo (issue #76, PR 2). */
export const GRIP_NAMES = [
  "sleeve_left", "sleeve_right", "lapel_left", "lapel_right", "collar_back", "belt_front", "belt_back",
  "wrist_left", "wrist_right", "triceps_left", "triceps_right", "collar_tie", "underhook_left", "underhook_right",
  "waist_lock", "knee_back_left", "knee_back_right", "ankle_left", "ankle_right",
] as const;
export type GripName = (typeof GRIP_NAMES)[number];

/** Lado de Uke. +Y es la izquierda (`docs/COORDINATES.md`). */
export type GripSide = "left" | "right";

/**
 * Dónde está el punto, sin necesidad de un sitio nuevo en el modelo. `S` es el lado: en el catálogo puede ser
 * `contrary_to_grip_side` (se resuelve con `grip_side`); resuelto es siempre `left` o `right`.
 *
 * - `site`: un sitio que existe en el MJCF generado, sin desplazamiento.
 * - `child_origin`: el origen del cuerpo hijo `child` expresado en el sistema del cuerpo `body` (el padre directo). Con
 *   el brazo es la articulación del hombro, en el sistema de `chest`.
 * - `geom_semiaxis`: el extremo `side` del semieje Y de una geometría alineada con su cuerpo (elipsoide de la pelvis).
 * - `site_lateral`: el sitio de línea media `site` girado 90° alrededor del eje Z de su cuerpo hasta el lado `side`
 *   (mismo radio y misma altura): `(0, ±hypot(x, y), z)`.
 */
export type GripAnchor<S extends string = GripSide | "contrary_to_grip_side"> =
  | { readonly kind: "site"; readonly site: string }
  | { readonly kind: "child_origin"; readonly body: string; readonly child: string }
  | { readonly kind: "geom_semiaxis"; readonly geom: string; readonly side: S }
  | { readonly kind: "site_lateral"; readonly site: string; readonly side: S };

export interface GripShareSpec {
  readonly anchor: GripAnchor;
  /** Fracción (> 0) de la fuerza de la mano que va a este punto. Las de un agarre suman 1. */
  readonly fraction: number;
}

export interface GripDefinition {
  readonly name: GripName;
  /** Tipo de unión por defecto de la mano virtual. */
  readonly link: HandLink;
  readonly shares: readonly GripShareSpec[];
  /** `true` en los cinturones: hay que indicar `grip_side` (lado de Uke por el que la mano toma el cinturón). */
  readonly needs_grip_side: boolean;
  /** `true` en `underhook_*`: hay que dar `toward_tori_m` (hacia dónde está Tori) para fijar el eje de empuje. */
  readonly needs_toward_tori: boolean;
  /** Agarre espejo (cambiar Y por −Y). Es el propio agarre en los que no tienen lado. */
  readonly mirror: GripName;
  /** Razón del reparto (resumen; el detalle está en `docs/GRIP_CATALOG.md`). */
  readonly rationale: string;
}

const opposite = (s: GripSide): GripSide => (s === "left" ? "right" : "left");

function sided(
  base: string, link: HandLink, build: (s: GripSide) => readonly GripShareSpec[], rationale: string,
  extra: { needs_toward_tori?: boolean } = {},
): GripDefinition[] {
  return (["left", "right"] as const).map((s) => ({
    name: `${base}_${s}` as GripName, link, shares: build(s), needs_grip_side: false,
    needs_toward_tori: extra.needs_toward_tori ?? false, mirror: `${base}_${opposite(s)}` as GripName, rationale,
  }));
}

function single(
  name: GripName, link: HandLink, shares: readonly GripShareSpec[], rationale: string,
  extra: { needs_grip_side?: boolean } = {},
): GripDefinition {
  return { name, link, shares, needs_grip_side: extra.needs_grip_side ?? false, needs_toward_tori: false, mirror: name, rationale };
}

const site = (name: string): GripAnchor => ({ kind: "site", site: name });
const whole = (anchor: GripAnchor): readonly GripShareSpec[] => [{ anchor, fraction: 1 }];

const DEFINITIONS: readonly GripDefinition[] = [
  ...sided("sleeve", "rope", (s) => whole(site(`${s}_sleeve_elbow`)),
    "La manga es tela: solo tracción. Se agarra a la altura del codo y la tela transmite todo al antebrazo."),
  ...sided("lapel", "rope", (s) => [
    { anchor: site("nape"), fraction: 0.7 },
    { anchor: { kind: "child_origin", body: "chest", child: `${opposite(s)}_upperarm` }, fraction: 0.3 },
  ], "La solapa carga la nuca y la espalda alta, no el pecho (plan 12.1, punto 4): 70 % en la nuca y 30 % en el hombro contrario."),
  single("collar_back", "rope", whole(site("nape")),
    "El cuello trasero de la ropa tira de la nuca."),
  single("belt_front", "rope", whole({ kind: "site_lateral", site: "belt_front", side: "contrary_to_grip_side" }),
    "El cinturón carga el lado contrario de la pelvis (plan 12.1, punto 4). Requiere `grip_side`.", { needs_grip_side: true }),
  single("belt_back", "rope", whole({ kind: "site_lateral", site: "belt_back", side: "contrary_to_grip_side" }),
    "Igual que `belt_front`, tomado por detrás. Requiere `grip_side`.", { needs_grip_side: true }),
  ...sided("wrist", "clamp", (s) => whole(site(`${s}_wrist`)),
    "La mano rodea la muñeca: tracción y empuje, todo en el extremo del antebrazo."),
  ...sided("triceps", "clamp", (s) => whole(site(`${s}_triceps`)),
    "La mano rodea la parte trasera del brazo: tracción y empuje, todo en el brazo."),
  single("collar_tie", "rope", whole(site("nape")),
    "Mano en la nuca, como `collar_back`. Unión `rope` según la tabla de la issue (la PR 1 lo cita también como `clamp`)."),
  ...sided("underhook", "push", (s) => whole(site(`${s}_armpit`)),
    "Empuje por contacto bajo la axila: solo hacia arriba y hacia Tori (eje `axis` de `push`).", { needs_toward_tori: true }),
  single("waist_lock", "clamp", [
    { anchor: { kind: "geom_semiaxis", geom: "pelvis_geom", side: "left" }, fraction: 0.5 },
    { anchor: { kind: "geom_semiaxis", geom: "pelvis_geom", side: "right" }, fraction: 0.5 },
  ], "Los brazos rodean la cintura: la fuerza se reparte a partes iguales entre los dos lados de la pelvis."),
  ...sided("knee_back", "clamp", (s) => whole(site(`${s}_knee_back`)),
    "La mano o el brazo rodea la corva: tracción y empuje, todo en la pierna."),
  ...sided("ankle", "clamp", (s) => whole(site(`${s}_ankle`)),
    "La mano rodea el tobillo: tracción y empuje, todo en el pie."),
];

/** El catálogo. Los repartos son de partida, sin calibrar y sin fuente biomecánica (ver `docs/GRIP_CATALOG.md`). */
export const GRIP_CATALOG: Readonly<Record<GripName, GripDefinition>> = Object.freeze(
  Object.fromEntries(DEFINITIONS.map((d) => [d.name, Object.freeze(d)])) as Record<GripName, GripDefinition>,
);

/** Elevación por defecto del eje de `underhook_*` sobre el plano horizontal. Punto de partida sin calibrar. */
export const UNDERHOOK_DEFAULT_ELEVATION_DEG = 45;

// --------------------------------------------------------------------------------------------------------------------
// Álgebra mínima
// --------------------------------------------------------------------------------------------------------------------

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0],
];
const norm = (a: Vec3): number => Math.sqrt(dot(a, a));
const finite = (v: unknown): v is Vec3 =>
  Array.isArray(v) && v.length === 3 && v.every((x) => typeof x === "number" && Number.isFinite(x));

function requireVec(name: string, v: unknown): asserts v is Vec3 {
  if (!finite(v)) throw new RangeError(`${name}: debe ser [x, y, z] con números finitos`);
}

// --------------------------------------------------------------------------------------------------------------------
// Resolución de un agarre (puro)
// --------------------------------------------------------------------------------------------------------------------

/** Datos que da quien llama y que el catálogo no puede conocer. */
export interface ResolveGripOptions {
  /** Solo `belt_*`: lado de Uke por el que la mano toma el cinturón. La fuerza va al lado contrario de la pelvis. */
  grip_side?: GripSide;
  /**
   * Solo `underhook_*`: vector de Uke hacia Tori en el marco de la mano (normalmente el mundo). Solo cuenta su parte
   * horizontal (se descarta Z); no puede ser vertical ni nulo. Tori no tiene cuerpo en HPL: lo da quien llama (por
   * ejemplo, el vector del centro de Uke a la posición de los pies de Tori del escenario).
   */
  toward_tori_m?: Vec3;
  /** Solo `underhook_*`: elevación del eje sobre el plano horizontal, de 0° (solo hacia Tori) a 90° (solo arriba). */
  underhook_elevation_deg?: number;
}

/** Un reparto con el lado ya fijado. */
export interface ResolvedGripShare {
  readonly anchor: GripAnchor<GripSide>;
  readonly fraction: number;
}

export interface ResolvedGrip {
  readonly name: GripName;
  readonly link: HandLink;
  readonly shares: readonly ResolvedGripShare[];
  /** Eje de empuje de `push` (unitario, hacia arriba y hacia Tori) o `null` si la unión no lo necesita. */
  readonly axis: Vec3 | null;
}

export function isGripName(name: string): name is GripName {
  return Object.prototype.hasOwnProperty.call(GRIP_CATALOG, name);
}

/**
 * Eje de empuje de `underhook_*`: `cos(e) · ĥ + sin(e) · ẑ`, con `ĥ` la parte horizontal de `toward_tori_m` normalizada y
 * `e` la elevación. Solo tiene componentes no negativas hacia arriba y hacia Tori y ninguna lateral: es la restricción
 * «solo hacia arriba y hacia Tori», y `push` nunca tira, así que la fuerza no puede salirse de ese cuarto de plano.
 */
export function underhookAxis(toward_tori_m: Vec3, elevation_deg: number = UNDERHOOK_DEFAULT_ELEVATION_DEG): Vec3 {
  requireVec("toward_tori_m", toward_tori_m);
  if (!(Number.isFinite(elevation_deg) && elevation_deg >= 0 && elevation_deg <= 90)) {
    throw new RangeError(`underhook_elevation_deg: debe estar entre 0 y 90 (recibido ${elevation_deg})`);
  }
  const horizontal = Math.hypot(toward_tori_m[0], toward_tori_m[1]);
  if (!(horizontal > 1e-9)) {
    throw new RangeError("toward_tori_m: su parte horizontal no puede ser nula (Tori no puede estar justo encima o debajo)");
  }
  const e = (elevation_deg * Math.PI) / 180;
  const c = Math.cos(e) / horizontal;
  return [toward_tori_m[0] * c, toward_tori_m[1] * c, Math.sin(e)];
}

/** `true` si `axis` cumple «solo hacia arriba y hacia Tori»: unitario, sin componente lateral y con las dos no negativas. */
export function isUnderhookAxis(axis: Vec3, toward_tori_m: Vec3, tolerance = 1e-9): boolean {
  if (!finite(axis) || !finite(toward_tori_m)) return false;
  const horizontal = Math.hypot(toward_tori_m[0], toward_tori_m[1]);
  if (!(horizontal > 1e-9)) return false;
  const toward: Vec3 = [toward_tori_m[0] / horizontal, toward_tori_m[1] / horizontal, 0];
  const lateral: Vec3 = [-toward[1], toward[0], 0];
  return Math.abs(norm(axis) - 1) <= tolerance && Math.abs(dot(axis, lateral)) <= tolerance &&
    dot(axis, toward) >= -tolerance && axis[2] >= -tolerance;
}

/** Resuelve un agarre a su definición con el lado fijado. Lanza `RangeError` con el motivo si falta o sobra un dato. */
export function resolveGrip(name: string, options: ResolveGripOptions = {}): ResolvedGrip {
  if (!isGripName(name)) {
    throw new RangeError(`Agarre desconocido "${name}". Agarres válidos: ${GRIP_NAMES.join(", ")}`);
  }
  const def = GRIP_CATALOG[name];
  if (def.needs_grip_side) {
    if (options.grip_side !== "left" && options.grip_side !== "right") {
      throw new RangeError(`${name}: falta grip_side ("left" o "right"), el lado de Uke por el que la mano toma el cinturón`);
    }
  } else if (options.grip_side !== undefined) {
    throw new RangeError(`${name}: no admite grip_side`);
  }
  if (!def.needs_toward_tori && (options.toward_tori_m !== undefined || options.underhook_elevation_deg !== undefined)) {
    throw new RangeError(`${name}: no admite toward_tori_m ni underhook_elevation_deg`);
  }
  let axis: Vec3 | null = null;
  if (def.needs_toward_tori) {
    if (options.toward_tori_m === undefined) {
      throw new RangeError(`${name}: falta toward_tori_m (vector de Uke hacia Tori; Tori no tiene cuerpo en HPL)`);
    }
    axis = underhookAxis(options.toward_tori_m, options.underhook_elevation_deg);
  }
  const shares = def.shares.map((share): ResolvedGripShare => {
    const a = share.anchor;
    if (a.kind === "geom_semiaxis" || a.kind === "site_lateral") {
      const s: GripSide = a.side === "contrary_to_grip_side" ? opposite(options.grip_side!) : a.side;
      return { anchor: { ...a, side: s }, fraction: share.fraction };
    }
    return { anchor: a, fraction: share.fraction };
  });
  return { name, link: def.link, shares, axis };
}

/** Agarre espejo (Y → −Y) con los datos de quien llama también reflejados. */
export function mirrorGripRequest(name: GripName, options: ResolveGripOptions = {}): { name: GripName; options: ResolveGripOptions } {
  const mirrored: ResolveGripOptions = { ...options };
  if (options.grip_side !== undefined) mirrored.grip_side = opposite(options.grip_side);
  if (options.toward_tori_m !== undefined) {
    mirrored.toward_tori_m = [options.toward_tori_m[0], -options.toward_tori_m[1], options.toward_tori_m[2]];
  }
  return { name: GRIP_CATALOG[name].mirror, options: mirrored };
}

/** Configuración de la mano para un agarre: pone el tipo de unión y, en `push`, el eje. `base` no puede traer `link` ni `axis`. */
export function handConfigForGrip(grip: ResolvedGrip, base: Omit<HandConfig, "link" | "axis">): HandConfig {
  return grip.axis === null ? { ...base, link: grip.link } : { ...base, link: grip.link, axis: grip.axis };
}

// --------------------------------------------------------------------------------------------------------------------
// Reparto de una fuerza y punto equivalente (puro)
// --------------------------------------------------------------------------------------------------------------------

/** Suma de las fracciones de un reparto (debe ser 1). */
export function fractionSum(shares: readonly { readonly fraction: number }[]): number {
  return shares.reduce((a, s) => a + s.fraction, 0);
}

function requireShares(shares: readonly { readonly fraction: number }[]): void {
  if (shares.length === 0) throw new RangeError("El reparto no tiene puntos");
  for (const s of shares) {
    if (!(Number.isFinite(s.fraction) && s.fraction > 0)) throw new RangeError(`fracción inválida: ${s.fraction}`);
  }
  if (Math.abs(fractionSum(shares) - 1) > 1e-9) throw new RangeError(`las fracciones suman ${fractionSum(shares)}, no 1`);
}

/** Fuerza de cada punto: `fracción · F`. La suma es `F` (salvo redondeo de coma flotante). */
export function splitGripForce(shares: readonly { readonly fraction: number }[], force_n: Vec3): Vec3[] {
  requireShares(shares);
  requireVec("force_n", force_n);
  return shares.map((s) => scale(force_n, s.fraction));
}

/**
 * Punto de agarre equivalente: la posición y la velocidad medias ponderadas por las fracciones. Es lo que necesita
 * `stepHand` (`grip_m`, `grip_velocity_mps`): una mano tiene un solo punto de agarre aunque la fuerza se reparta.
 */
export function equivalentGripPoint(
  shares: readonly { readonly fraction: number }[], positions_m: readonly Vec3[], velocities_mps: readonly Vec3[],
): { point_m: Vec3; velocity_mps: Vec3 } {
  requireShares(shares);
  if (positions_m.length !== shares.length || velocities_mps.length !== shares.length) {
    throw new RangeError("hacen falta una posición y una velocidad por punto del reparto");
  }
  let p: Vec3 = [0, 0, 0];
  let v: Vec3 = [0, 0, 0];
  shares.forEach((s, i) => {
    requireVec(`positions_m[${i}]`, positions_m[i]);
    requireVec(`velocities_mps[${i}]`, velocities_mps[i]);
    p = add(p, scale(positions_m[i]!, s.fraction));
    v = add(v, scale(velocities_mps[i]!, s.fraction));
  });
  return { point_m: p, velocity_mps: v };
}

/** Par `Σ (p_i − c) × F_i` respecto al punto `about_m` (por ejemplo, el centro de masas de Uke), en N·m. */
export function gripTorqueAbout(positions_m: readonly Vec3[], forces_n: readonly Vec3[], about_m: Vec3): Vec3 {
  if (positions_m.length !== forces_n.length) throw new RangeError("hace falta una fuerza por posición");
  requireVec("about_m", about_m);
  let torque: Vec3 = [0, 0, 0];
  positions_m.forEach((p, i) => {
    requireVec(`positions_m[${i}]`, p);
    requireVec(`forces_n[${i}]`, forces_n[i]);
    torque = add(torque, cross(sub(p, about_m), forces_n[i]!));
  });
  return torque;
}

// --------------------------------------------------------------------------------------------------------------------
// Con un modelo MuJoCo (estructural: no se importa MuJoCo)
// --------------------------------------------------------------------------------------------------------------------

/** Lo que usa `locateGrip` de un `MjModel`. */
export interface GripModel {
  site(name: string): { readonly id: number };
  body(name: string): { readonly id: number };
  geom(name: string): { readonly id: number };
  readonly site_bodyid: ArrayLike<number>;
  readonly site_pos: ArrayLike<number>;
  readonly body_pos: ArrayLike<number>;
  readonly body_parentid: ArrayLike<number>;
  readonly body_rootid: ArrayLike<number>;
  readonly geom_bodyid: ArrayLike<number>;
  readonly geom_pos: ArrayLike<number>;
  readonly geom_quat: ArrayLike<number>;
  readonly geom_size: ArrayLike<number>;
}

/** Datos para colocar el reparto: posición y orientación de los cuerpos (válidas tras `mj_kinematics`). */
export type GripData = WrenchData & { readonly xpos: ArrayLike<number>; readonly xmat: ArrayLike<number> };
/** Además, lo necesario para la velocidad de un punto (válido tras `mj_comVel`). */
export type GripVelocityData = GripData & { readonly cvel: ArrayLike<number>; readonly subtree_com: ArrayLike<number> };

export interface LocatedGripShare extends ResolvedGripShare {
  readonly bodyId: number;
  /** Punto en el sistema local del cuerpo, en m. */
  readonly local_m: Vec3;
}

export interface LocatedGrip extends Omit<ResolvedGrip, "shares"> {
  readonly shares: readonly LocatedGripShare[];
}

function lookup(model: GripModel, kind: "site" | "body" | "geom", name: string, grip: GripName): number {
  let id = -1;
  try {
    id = model[kind](name).id;
  } catch {
    id = -1;
  }
  if (!(Number.isInteger(id) && id >= 0)) {
    throw new RangeError(`${grip}: el modelo no tiene ${kind === "site" ? "el sitio" : kind === "body" ? "el cuerpo" : "la geometría"} "${name}"`);
  }
  return id;
}

const triple = (a: ArrayLike<number>, i: number): Vec3 => [a[i]!, a[i + 1]!, a[i + 2]!];
const SIDE_SIGN: Readonly<Record<GripSide, number>> = { left: 1, right: -1 };

/** Cuerpo y punto local de cada reparto en un modelo concreto. Lanza `RangeError` si falta un sitio, cuerpo o geometría. */
export function locateGrip(model: GripModel, grip: ResolvedGrip): LocatedGrip {
  const shares = grip.shares.map((share): LocatedGripShare => {
    const a = share.anchor;
    if (a.kind === "site") {
      const id = lookup(model, "site", a.site, grip.name);
      return { ...share, bodyId: model.site_bodyid[id]!, local_m: triple(model.site_pos, 3 * id) };
    }
    if (a.kind === "child_origin") {
      const parent = lookup(model, "body", a.body, grip.name);
      const child = lookup(model, "body", a.child, grip.name);
      if (model.body_parentid[child] !== parent) {
        throw new RangeError(`${grip.name}: el cuerpo "${a.child}" no es hijo directo de "${a.body}"`);
      }
      return { ...share, bodyId: parent, local_m: triple(model.body_pos, 3 * child) };
    }
    if (a.kind === "site_lateral") {
      const id = lookup(model, "site", a.site, grip.name);
      const p = triple(model.site_pos, 3 * id);
      return {
        ...share, bodyId: model.site_bodyid[id]!, local_m: [0, SIDE_SIGN[a.side] * Math.hypot(p[0], p[1]), p[2]],
      };
    }
    const g = lookup(model, "geom", a.geom, grip.name);
    const q = triple(model.geom_quat, 4 * g + 1); // x, y, z del cuaternión (w primero): debe ser la identidad
    if (Math.abs(model.geom_quat[4 * g]! - 1) > 1e-9 || norm(q) > 1e-9) {
      throw new RangeError(`${grip.name}: la geometría "${a.geom}" no está alineada con su cuerpo`);
    }
    const centre = triple(model.geom_pos, 3 * g);
    return {
      ...share, bodyId: model.geom_bodyid[g]!,
      local_m: [centre[0], centre[1] + SIDE_SIGN[a.side] * model.geom_size[3 * g + 1]!, centre[2]],
    };
  });
  return { ...grip, shares };
}

/** Punto del mundo de un punto local de un cuerpo: `xpos + R · local`. */
function worldPoint(data: GripData, bodyId: number, local: Vec3): Vec3 {
  const o = 9 * bodyId;
  const m = data.xmat;
  return add(triple(data.xpos, 3 * bodyId), [
    m[o]! * local[0] + m[o + 1]! * local[1] + m[o + 2]! * local[2],
    m[o + 3]! * local[0] + m[o + 4]! * local[1] + m[o + 5]! * local[2],
    m[o + 6]! * local[0] + m[o + 7]! * local[1] + m[o + 8]! * local[2],
  ]);
}

/** Posición en el mundo de cada punto del reparto (hace falta `mj_kinematics`). */
export function gripPointPositions(data: GripData, grip: LocatedGrip): Vec3[] {
  return grip.shares.map((s) => worldPoint(data, s.bodyId, s.local_m));
}

/**
 * Posición y velocidad en el mundo de cada punto del reparto (hace falta `mj_kinematics`, `mj_comPos` y `mj_comVel`).
 * La velocidad es la de `src/scenario/run.ts`: `cvel` está referido al centro de masas del subárbol de la raíz.
 */
export function gripPointStates(
  model: Pick<GripModel, "body_rootid">, data: GripVelocityData, grip: LocatedGrip,
): { points_m: Vec3[]; velocities_mps: Vec3[] } {
  const points_m = gripPointPositions(data, grip);
  const velocities_mps = grip.shares.map((s, i) => {
    const ref = triple(data.subtree_com, 3 * model.body_rootid[s.bodyId]!);
    const angular = triple(data.cvel, 6 * s.bodyId);
    const linear = triple(data.cvel, 6 * s.bodyId + 3);
    return add(linear, cross(angular, sub(points_m[i]!, ref)));
  });
  return { points_m, velocities_mps };
}

/**
 * Aplica `force_n` repartida según el agarre: suma a `xfrc_applied` la parte de cada punto (con su par `r × F` respecto
 * al centro de masas de su cuerpo, por `applyForceAtPoint`). Acumula: quien llama pone `xfrc_applied` a cero al empezar el
 * paso. Hace falta `mj_kinematics` y `mj_comPos`. Devuelve la fuerza aplicada en cada punto, en el orden del reparto.
 */
export function applyGripForce(data: GripData, grip: LocatedGrip, force_n: Vec3): Vec3[] {
  const forces = splitGripForce(grip.shares, force_n);
  grip.shares.forEach((s, i) => applyForceAtPoint(data, s.bodyId, worldPoint(data, s.bodyId, s.local_m), forces[i]!));
  return forces;
}
