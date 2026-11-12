import { HEIGHT_RANGE_M, MASS_RANGE_KG, V2_SEGMENT_LENGTHS_M } from "../uke/generate";
import {
  ACTOR_BODIES,
  HUMANOID_SITE_BODY,
  EFFECTOR_FRAMES,
  EFFECTOR_WAVEFORMS,
  OUTCOMES,
  SEGMENT_LENGTH_KEYS,
  UKE_ATTITUDES,
  UKE_STANCES,
  SCENARIO_MAX_DURATION_S,
  SCENARIO_PHYSICS_HZ,
  SCENARIO_RECORD_HZ_MAX,
  SCENARIO_RECORD_HZ_MIN,
  SCENARIO_SCHEMA_ID,
  type ActorSpec,
  type HumanoidActor,
  type SegmentLengthKey,
  type EffectorSpec,
  type ExpectSpec,
  type PropSpec,
  type Scenario,
  type Vec3,
} from "./schema";

/** Error de validación: `path` es la ruta del campo (`effectors[1].force_n[0]`) y `reason` el motivo. */
export class ScenarioValidationError extends Error {
  readonly path: string;
  readonly reason: string;

  constructor(path: string, reason: string) {
    super(`${path}: ${reason}`);
    this.name = "ScenarioValidationError";
    this.path = path;
    this.reason = reason;
  }
}

export interface ValidateOptions {
  /** Nombre del fichero (con o sin directorio y extensión). Si se da, `id` debe coincidir con él. */
  fileName?: string;
}

const ID_PATTERN = /^[a-z0-9_]+$/;
type Obj = Record<string, unknown>;

function fail(path: string, reason: string): never {
  throw new ScenarioValidationError(path, reason);
}

function join(parent: string, key: string): string {
  return parent === "" ? key : `${parent}.${key}`;
}

function describe(value: unknown): string {
  if (value === null) return "null";
  if (Array.isArray(value)) return "una lista";
  return typeof value === "object" ? "un objeto" : `${typeof value} (${String(value)})`;
}

function asObject(value: unknown, path: string): Obj {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    fail(path === "" ? "(raíz)" : path, `debe ser un objeto, no ${describe(value)}`);
  }
  return value as Obj;
}

/** Comprueba que no haya claves desconocidas ni falten las obligatorias. */
function checkKeys(obj: Obj, path: string, required: readonly string[], optional: readonly string[] = []): void {
  for (const key of Object.keys(obj)) {
    if (!required.includes(key) && !optional.includes(key)) fail(join(path, key), "campo desconocido");
  }
  for (const key of required) {
    if (!(key in obj)) fail(join(path, key), "campo obligatorio ausente");
  }
}

function asFinite(value: unknown, path: string): number {
  if (typeof value !== "number") fail(path, `debe ser un número, no ${describe(value)}`);
  if (!Number.isFinite(value)) fail(path, `debe ser un número finito, no ${String(value)}`);
  return value;
}

function asPositive(value: unknown, path: string): number {
  const n = asFinite(value, path);
  if (!(n > 0)) fail(path, `debe ser mayor que 0, no ${n}`);
  return n;
}

function asNonNegative(value: unknown, path: string): number {
  const n = asFinite(value, path);
  if (n < 0) fail(path, `no puede ser negativo, no ${n}`);
  return n;
}

function asDuration(value: unknown, path: string): number {
  const n = asPositive(value, path);
  if (n > SCENARIO_MAX_DURATION_S) fail(path, `no puede superar ${SCENARIO_MAX_DURATION_S} s, no ${n}`);
  return n;
}

function asString(value: unknown, path: string): string {
  if (typeof value !== "string") fail(path, `debe ser una cadena, no ${describe(value)}`);
  return value;
}

function asId(value: unknown, path: string): string {
  const s = asString(value, path);
  if (!ID_PATTERN.test(s)) fail(path, `solo admite [a-z0-9_] y no puede estar vacío, no "${s}"`);
  return s;
}

function asEnum<T extends string>(value: unknown, path: string, allowed: readonly T[]): T {
  const s = asString(value, path);
  if (!(allowed as readonly string[]).includes(s)) fail(path, `debe ser uno de ${allowed.join(", ")}, no "${s}"`);
  return s as T;
}

function asArray(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) fail(path, `debe ser una lista, no ${describe(value)}`);
  return value;
}

function asVec3(value: unknown, path: string, element: (v: unknown, p: string) => number = asFinite): Vec3 {
  const arr = asArray(value, path);
  if (arr.length !== 3) fail(path, `debe tener exactamente 3 números, tiene ${arr.length}`);
  return [element(arr[0], `${path}[0]`), element(arr[1], `${path}[1]`), element(arr[2], `${path}[2]`)];
}

function asRange(value: unknown, path: string, [lo, hi]: readonly [number, number], unit: string): number {
  const n = asFinite(value, path);
  if (n < lo || n > hi) fail(path, `debe estar entre ${lo} y ${hi} ${unit}, no ${n}`);
  return n;
}

function validateHumanoid(obj: Obj, path: string, id: string): HumanoidActor {
  checkKeys(obj, path, ["id", "kind", "mass_kg", "height_m"], [
    "strength", "build", "segment_lengths_m", "attitude", "stance",
  ]);
  const actor: HumanoidActor = {
    id,
    kind: "humanoid",
    mass_kg: asRange(obj.mass_kg, join(path, "mass_kg"), MASS_RANGE_KG, "kg"),
    height_m: asRange(obj.height_m, join(path, "height_m"), HEIGHT_RANGE_M, "m"),
    strength: "strength" in obj ? asRange(obj.strength, join(path, "strength"), [0.5, 1.5], "") : 1,
    attitude: "attitude" in obj ? asEnum(obj.attitude, join(path, "attitude"), UKE_ATTITUDES) : "tense",
    stance: "stance" in obj ? asEnum(obj.stance, join(path, "stance"), UKE_STANCES) : "neutral",
  };
  if ("build" in obj) actor.build = asRange(obj.build, join(path, "build"), [0, 1], "");
  if ("segment_lengths_m" in obj) {
    const p = join(path, "segment_lengths_m");
    const lengths = asObject(obj.segment_lengths_m, p);
    checkKeys(lengths, p, [], SEGMENT_LENGTH_KEYS);
    const result: Partial<Record<SegmentLengthKey, number>> = {};
    for (const key of SEGMENT_LENGTH_KEYS) {
      if (!(key in lengths)) continue;
      const base = V2_SEGMENT_LENGTHS_M[key];
      result[key] = asRange(lengths[key], join(p, key), [0.5 * base, 2 * base], "m");
    }
    actor.segment_lengths_m = result;
  }
  return actor;
}

function validateActor(value: unknown, path: string): ActorSpec {
  const obj = asObject(value, path);
  if (obj.kind === "humanoid") return validateHumanoid(obj, path, asId(obj.id, join(path, "id")));
  checkKeys(obj, path, ["id", "kind", "mass_kg", "height_m", "radius_m"]);
  const id = asId(obj.id, join(path, "id"));
  asEnum(obj.kind, join(path, "kind"), ["rigid_cylinder", "humanoid"]);
  return {
    id,
    kind: "rigid_cylinder",
    mass_kg: asPositive(obj.mass_kg, join(path, "mass_kg")),
    height_m: asPositive(obj.height_m, join(path, "height_m")),
    radius_m: asPositive(obj.radius_m, join(path, "radius_m")),
  };
}

function validateProp(value: unknown, path: string): PropSpec {
  const obj = asObject(value, path);
  checkKeys(obj, path, ["id", "kind", "size_m", "pos_m"]);
  const id = asId(obj.id, join(path, "id"));
  asEnum(obj.kind, join(path, "kind"), ["fixed_box"]);
  return {
    id,
    kind: "fixed_box",
    size_m: asVec3(obj.size_m, join(path, "size_m"), asPositive),
    pos_m: asVec3(obj.pos_m, join(path, "pos_m")),
  };
}

function validateEffector(value: unknown, path: string, actors: readonly ActorSpec[]): EffectorSpec {
  const obj = asObject(value, path);
  checkKeys(obj, path, [
    "id", "kind", "actor", "frame", "force_n", "start_s", "duration_s", "waveform",
  ], ["body", "site", "local_point_m"]);
  const id = asId(obj.id, join(path, "id"));
  asEnum(obj.kind, join(path, "kind"), ["prescribed_force"]);
  const actorId = asString(obj.actor, join(path, "actor"));
  const actor = actors.find((a) => a.id === actorId);
  if (!actor) {
    const known = actors.map((a) => a.id).join(", ") || "ninguno";
    fail(join(path, "actor"), `referencia a un actor inexistente "${actorId}" (actores: ${known})`);
  }
  const bodies = ACTOR_BODIES[actor.kind];
  let body: string;
  let site: string | undefined;
  let local: Vec3 = [0, 0, 0];
  if ("site" in obj) {
    if ("body" in obj) fail(join(path, "body"), "no puede combinarse con site: usa body + local_point_m o solo site");
    site = asString(obj.site, join(path, "site"));
    const siteBody = actor.kind === "humanoid" ? HUMANOID_SITE_BODY[site] : undefined;
    if (siteBody === undefined) {
      const known = actor.kind === "humanoid" ? Object.keys(HUMANOID_SITE_BODY).join(", ") : "ninguno";
      fail(join(path, "site"), `el actor "${actor.id}" no tiene el sitio "${site}" (sitios: ${known})`);
    }
    body = siteBody;
    if ("local_point_m" in obj) local = asVec3(obj.local_point_m, join(path, "local_point_m"));
  } else {
    if (!("body" in obj)) fail(join(path, "body"), "campo obligatorio ausente (o indica site)");
    body = asString(obj.body, join(path, "body"));
    if (!bodies.includes(body)) {
      fail(join(path, "body"), `el actor "${actor.id}" no tiene el cuerpo "${body}" (cuerpos: ${bodies.join(", ")})`);
    }
    if (!("local_point_m" in obj)) fail(join(path, "local_point_m"), "campo obligatorio ausente");
    local = asVec3(obj.local_point_m, join(path, "local_point_m"));
  }
  const effector: EffectorSpec = {
    id,
    kind: "prescribed_force",
    actor: actor.id,
    body,
    local_point_m: local,
    frame: asEnum(obj.frame, join(path, "frame"), EFFECTOR_FRAMES),
    force_n: asVec3(obj.force_n, join(path, "force_n")),
    start_s: asNonNegative(obj.start_s, join(path, "start_s")),
    duration_s: asDuration(obj.duration_s, join(path, "duration_s")),
    waveform: asEnum(obj.waveform, join(path, "waveform"), EFFECTOR_WAVEFORMS),
  };
  if (site !== undefined) effector.site = site;
  return effector;
}

function validateExpect(value: unknown, path: string): ExpectSpec {
  const obj = asObject(value, path);
  checkKeys(obj, path, ["outcome"], ["direction_deg", "tolerance_deg"]);
  const outcome = asEnum(obj.outcome, join(path, "outcome"), OUTCOMES);
  const result: ExpectSpec = { outcome };
  if ("direction_deg" in obj) {
    const p = join(path, "direction_deg");
    result.direction_deg = asFinite(obj.direction_deg, p);
    if (outcome !== "fall") fail(p, `solo se admite con outcome "fall", no "${outcome}"`);
    if (!("tolerance_deg" in obj)) fail(join(path, "tolerance_deg"), "campo obligatorio ausente cuando hay direction_deg");
  }
  if ("tolerance_deg" in obj) {
    const p = join(path, "tolerance_deg");
    if (!("direction_deg" in obj)) fail(p, "requiere direction_deg");
    const tol = asPositive(obj.tolerance_deg, p);
    if (tol > 180) fail(p, `no puede superar 180 grados, no ${tol}`);
    result.tolerance_deg = tol;
  }
  return result;
}

function validateList<T>(
  value: unknown,
  path: string,
  item: (v: unknown, p: string) => T,
  idOf: (t: T) => string,
): T[] {
  const arr = asArray(value, path);
  const seen = new Set<string>();
  return arr.map((v, i) => {
    const itemPath = `${path}[${i}]`;
    const parsed = item(v, itemPath);
    const id = idOf(parsed);
    if (seen.has(id)) fail(join(itemPath, "id"), `id duplicado "${id}"`);
    seen.add(id);
    return parsed;
  });
}

function fileStem(fileName: string): string {
  const base = fileName.split(/[\\/]/).pop() ?? fileName;
  return base.replace(/\.json$/i, "");
}

/**
 * Valida un escenario `hpl.scenario.v1` ya leído como JSON.
 * Lanza `ScenarioValidationError` (con la ruta del campo y el motivo) en el primer fallo.
 */
export function validateScenario(json: unknown, options: ValidateOptions = {}): Scenario {
  const root = asObject(json, "");
  // La versión se comprueba antes que el resto: otra versión puede tener otros campos.
  if ("schema" in root) {
    const schema = asString(root.schema, "schema");
    if (schema !== SCENARIO_SCHEMA_ID) fail("schema", `debe ser "${SCENARIO_SCHEMA_ID}", no "${schema}"`);
  }
  checkKeys(root, "", ["schema", "id", "duration_s", "record_hz", "ground", "actors", "props", "effectors"], ["expect"]);

  const id = asId(root.id, "id");
  if (options.fileName !== undefined && fileStem(options.fileName) !== id) {
    fail("id", `debe ser igual al nombre del fichero "${fileStem(options.fileName)}", no "${id}"`);
  }

  const duration_s = asDuration(root.duration_s, "duration_s");

  const record_hz = asFinite(root.record_hz, "record_hz");
  if (record_hz < SCENARIO_RECORD_HZ_MIN || record_hz > SCENARIO_RECORD_HZ_MAX) {
    fail("record_hz", `debe estar entre ${SCENARIO_RECORD_HZ_MIN} y ${SCENARIO_RECORD_HZ_MAX} Hz, no ${record_hz}`);
  }
  if (!Number.isInteger(record_hz) || SCENARIO_PHYSICS_HZ % record_hz !== 0) {
    fail("record_hz", `debe ser divisor exacto de la frecuencia física (${SCENARIO_PHYSICS_HZ} Hz), no ${record_hz}`);
  }

  const groundObj = asObject(root.ground, "ground");
  checkKeys(groundObj, "ground", ["friction"]);
  const ground = { friction: asNonNegative(groundObj.friction, "ground.friction") };

  const actors = validateList(root.actors, "actors", validateActor, (a) => a.id);
  if (actors.length === 0) fail("actors", "debe haber al menos un actor");
  const props = validateList(root.props, "props", validateProp, (p) => p.id);
  const effectors = validateList(root.effectors, "effectors", (v, p) => validateEffector(v, p, actors), (e) => e.id);

  const scenario: Scenario = {
    schema: SCENARIO_SCHEMA_ID,
    id,
    duration_s,
    record_hz,
    ground,
    actors,
    props,
    effectors,
  };
  if ("expect" in root) scenario.expect = validateExpect(root.expect, "expect");
  return scenario;
}
