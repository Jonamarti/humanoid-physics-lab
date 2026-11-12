import { describe, expect, it } from "vitest";
import {
  GRIP_CATALOG, GRIP_NAMES, UNDERHOOK_DEFAULT_ELEVATION_DEG, applyGripForce, equivalentGripPoint, fractionSum,
  gripPointPositions, gripPointStates, gripTorqueAbout, handConfigForGrip, isUnderhookAxis, locateGrip,
  mirrorGripRequest, resolveGrip, splitGripForce, underhookAxis,
  type GripAnchor, type GripModel, type GripName, type LocatedGrip, type ResolveGripOptions, type ResolvedGrip, type Vec3,
} from "../../src/effectors/gripCatalog";
import { createHandState, resolveHandParams, stepHand } from "../../src/effectors/hand";
import { getMujoco, type MujocoModule } from "../../src/scenario/mujocoModule";
import { HUMANOID_SITE_BODY } from "../../src/scenario/schema";
import { PHYSICS_TIMESTEP_S } from "../../src/scenario/worldOptions";
import { relaxedJointDamping, setJointDamping, type DampingModel } from "../../src/uke/damping";
import { generateHumanoidMJCF } from "../../src/uke/generate";
import { applyStance } from "../../src/uke/stances";

const DT = PHYSICS_TIMESTEP_S;
const SIZES = [
  { mass_kg: 55, height_m: 1.55 }, { mass_kg: 75, height_m: 1.75 }, { mass_kg: 100, height_m: 1.95 },
  { mass_kg: 150, height_m: 2.0 },
] as const;

/** Datos que pide cada agarre al resolverlo (cinturón: lado; underhook: hacia dónde está Tori). */
function optionsFor(name: GripName, grip_side: "left" | "right" = "left"): ResolveGripOptions {
  const def = GRIP_CATALOG[name];
  return {
    ...(def.needs_grip_side ? { grip_side } : {}),
    ...(def.needs_toward_tori ? { toward_tori_m: [1, 0, 0] as Vec3 } : {}),
  };
}
const ALL_REQUESTS: Array<{ name: GripName; options: ResolveGripOptions }> = GRIP_NAMES.flatMap((name) =>
  GRIP_CATALOG[name].needs_grip_side
    ? [{ name, options: optionsFor(name, "left") }, { name, options: optionsFor(name, "right") }]
    : [{ name, options: optionsFor(name) }],
);

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a: Vec3): number => Math.sqrt(dot(a, a));
const mirrorY = (v: Vec3): Vec3 => [v[0], -v[1], v[2]];
/** Reflexión de un vector axial (velocidad angular, par) al cambiar Y por −Y. */
const mirrorAxialY = (v: Vec3): Vec3 => [-v[0], v[1], -v[2]];
const fixed = (v: ArrayLike<number>, d = 1): string => Array.from(v, (x) => x.toFixed(d)).join(", ");

const BODY_NAMES = [
  "pelvis", "abdomen", "chest", "head",
  ...["left", "right"].flatMap((s) => [`${s}_upperarm`, `${s}_forearm`, `${s}_thigh`, `${s}_shin`, `${s}_foot`]),
];
/** Nombre del cuerpo espejo (izquierda ↔ derecha; los de línea media son el mismo). */
const mirrorBodyName = (n: string): string => (n.startsWith("left_") ? `right_${n.slice(5)}` : n.startsWith("right_") ? `left_${n.slice(6)}` : n);

// ---------------------------------------------------------------------------------------------------------------------
// Montajes con MuJoCo WASM real
// ---------------------------------------------------------------------------------------------------------------------

type Model = InstanceType<MujocoModule["MjModel"]>;
type Data = InstanceType<MujocoModule["MjData"]>;

async function withUke<T>(
  size: { mass_kg: number; height_m: number }, fn: (mj: MujocoModule, model: Model, data: Data) => T | Promise<T>,
): Promise<T> {
  const mj = await getMujoco();
  const model = mj.MjModel.from_xml_string(generateHumanoidMJCF(size));
  const data = new mj.MjData(model);
  try {
    return await fn(mj, model, data);
  } finally {
    data.delete();
    model.delete();
  }
}

const asGripModel = (m: Model): GripModel => m as unknown as GripModel;
const asGripData = (d: Data): Parameters<typeof applyGripForce>[0] & Parameters<typeof gripPointStates>[1] =>
  d as unknown as Parameters<typeof applyGripForce>[0] & Parameters<typeof gripPointStates>[1];

/** Pose por defecto del MJCF (todos los ángulos a 0, simétrica) con cinemática al día. */
function kinematics(mj: MujocoModule, model: Model, data: Data): void {
  mj.mj_forward(model, data);
  mj.mj_kinematics(model, data);
  mj.mj_comPos(model, data);
  mj.mj_comVel(model, data);
}

const comOf = (data: Data): Vec3 => [data.subtree_com[3]!, data.subtree_com[4]!, data.subtree_com[5]!];

function locate(model: Model, request: { name: GripName; options: ResolveGripOptions }): LocatedGrip {
  return locateGrip(asGripModel(model), resolveGrip(request.name, request.options));
}

/**
 * Montaje de caída libre: Uke `relaxed` (sin controlador; solo amortiguación pasiva y topes de rango, como en el
 * ejecutor) en postura neutra, elevado 1 m sobre el suelo con velocidad cero. Durante 80 ms cae unos 3 cm sin tocar nada
 * y, como la gravedad es uniforme, no produce ningún par respecto al centro de masas: lo único que cambia su momento
 * angular son los efectores declarados. Así el momento angular total respecto al centro de masas es exactamente
 * `∫ Σ (p_i − c) × F_i dt`, y el tronco responde sin que lo sostengan los pies.
 */
async function freeFall(
  size: { mass_kg: number; height_m: number }, request: { name: GripName; options: ResolveGripOptions },
  force: Vec3, steps: number,
) {
  return withUke(size, (mj, model, data) => {
    setJointDamping(model as unknown as DampingModel, relaxedJointDamping(size.mass_kg, size.height_m));
    applyStance(mj, model, data, "neutral");
    data.qpos[2] = data.qpos[2]! + 1;
    mj.mj_forward(model, data);
    const grip = locate(model, request);
    const xfrc = data.xfrc_applied as Float64Array;
    const chest = model.body("chest").id;
    let predictedTorque: Vec3 = [0, 0, 0];
    let comStart: Vec3 = [0, 0, 0];
    let angularImpulse: Vec3 = [0, 0, 0]; // ∫ Σ (p_i − c) × F_i dt con los puntos y el CM de cada paso
    for (let i = 0; i < steps; i++) {
      xfrc.fill(0);
      mj.mj_kinematics(model, data);
      mj.mj_comPos(model, data);
      mj.mj_comVel(model, data);
      const tau = gripTorqueAbout(
        gripPointPositions(asGripData(data), grip), splitGripForce(grip.shares, force), comOf(data),
      );
      angularImpulse = add(angularImpulse, [tau[0] * DT, tau[1] * DT, tau[2] * DT]);
      if (i === 0) {
        comStart = comOf(data);
        predictedTorque = tau;
      }
      applyGripForce(asGripData(data), grip, force);
      mj.mj_step(model, data);
    }
    // Tras `mj_step`, cinemática y velocidades (cvel) quedan un paso atrás: se actualizan antes de medir.
    mj.mj_kinematics(model, data);
    mj.mj_comPos(model, data);
    mj.mj_comVel(model, data);
    mj.mj_subtreeVel(model, data);
    const t = steps * DT;
    const angmom: Vec3 = [data.subtree_angmom[3]!, data.subtree_angmom[4]!, data.subtree_angmom[5]!];
    const chestOmega: Vec3 = [data.cvel[6 * chest]!, data.cvel[6 * chest + 1]!, data.cvel[6 * chest + 2]!];
    const bodies = new Map<string, Vec3>(BODY_NAMES.map((name) => {
      const b = model.body(name).id;
      return [name, [data.xpos[3 * b]!, data.xpos[3 * b + 1]!, data.xpos[3 * b + 2]!] as Vec3];
    }));
    return {
      t, predictedTorque, angularImpulse, angmom, /** dL/dt medido (N·m) */ torqueMeasured: [angmom[0] / t, angmom[1] / t, angmom[2] / t] as Vec3,
      /** aceleración angular media del tronco (rad/s²) */ chestAlpha: [chestOmega[0] / t, chestOmega[1] / t, chestOmega[2] / t] as Vec3,
      chestOmega, bodies, comStart, mass: model.body_subtreemass[1]!,
    };
  });
}

// ---------------------------------------------------------------------------------------------------------------------
// Estructura del catálogo (puro)
// ---------------------------------------------------------------------------------------------------------------------

/** La tabla de la issue #76 escrita aparte del código: unión y reparto (sitio, fracción) de cada agarre. */
const ISSUE_TABLE: Record<string, { link: string; shares: Array<[string, number]> }> = {
  sleeve_left: { link: "rope", shares: [["site:left_sleeve_elbow", 1]] },
  sleeve_right: { link: "rope", shares: [["site:right_sleeve_elbow", 1]] },
  lapel_left: { link: "rope", shares: [["site:nape", 0.7], ["child_origin:chest/right_upperarm", 0.3]] },
  lapel_right: { link: "rope", shares: [["site:nape", 0.7], ["child_origin:chest/left_upperarm", 0.3]] },
  collar_back: { link: "rope", shares: [["site:nape", 1]] },
  belt_front: { link: "rope", shares: [["site_lateral:belt_front/contrary", 1]] },
  belt_back: { link: "rope", shares: [["site_lateral:belt_back/contrary", 1]] },
  wrist_left: { link: "clamp", shares: [["site:left_wrist", 1]] },
  wrist_right: { link: "clamp", shares: [["site:right_wrist", 1]] },
  triceps_left: { link: "clamp", shares: [["site:left_triceps", 1]] },
  triceps_right: { link: "clamp", shares: [["site:right_triceps", 1]] },
  collar_tie: { link: "rope", shares: [["site:nape", 1]] },
  underhook_left: { link: "push", shares: [["site:left_armpit", 1]] },
  underhook_right: { link: "push", shares: [["site:right_armpit", 1]] },
  waist_lock: { link: "clamp", shares: [["geom_semiaxis:pelvis_geom/left", 0.5], ["geom_semiaxis:pelvis_geom/right", 0.5]] },
  knee_back_left: { link: "clamp", shares: [["site:left_knee_back", 1]] },
  knee_back_right: { link: "clamp", shares: [["site:right_knee_back", 1]] },
  ankle_left: { link: "clamp", shares: [["site:left_ankle", 1]] },
  ankle_right: { link: "clamp", shares: [["site:right_ankle", 1]] },
};

function describeAnchor(a: GripAnchor): string {
  switch (a.kind) {
    case "site": return `site:${a.site}`;
    case "child_origin": return `child_origin:${a.body}/${a.child}`;
    case "geom_semiaxis": return `geom_semiaxis:${a.geom}/${a.side}`;
    case "site_lateral": return `site_lateral:${a.site}/${a.side === "contrary_to_grip_side" ? "contrary" : a.side}`;
  }
}

describe("estructura del catálogo", () => {
  it("tiene exactamente los agarres de la tabla de la issue y cada uno coincide con ella (unión, puntos y fracciones)", () => {
    expect([...GRIP_NAMES].sort()).toEqual(Object.keys(ISSUE_TABLE).sort());
    expect(Object.keys(GRIP_CATALOG).sort()).toEqual(Object.keys(ISSUE_TABLE).sort());
    for (const name of GRIP_NAMES) {
      const def = GRIP_CATALOG[name];
      const expected = ISSUE_TABLE[name]!;
      expect(def.name).toBe(name);
      expect(def.link, `${name}: unión`).toBe(expected.link);
      expect(def.shares.map((s) => [describeAnchor(s.anchor), s.fraction]), `${name}: reparto`).toEqual(expected.shares);
      expect(def.rationale.length, `${name}: sin razón`).toBeGreaterThan(20);
    }
  });

  it("las fracciones son positivas y suman 1 en todos los agarres", () => {
    for (const name of GRIP_NAMES) {
      const { shares } = GRIP_CATALOG[name];
      expect(shares.length).toBeGreaterThan(0);
      for (const s of shares) expect(s.fraction, name).toBeGreaterThan(0);
      expect(Math.abs(fractionSum(shares) - 1), `${name}: suma ${fractionSum(shares)}`).toBeLessThan(1e-12);
    }
  });

  it("resolver un agarre desconocido o con datos de más o de menos da un error claro", () => {
    expect(() => resolveGrip("seoi")).toThrow(/Agarre desconocido "seoi".*sleeve_left/);
    expect(() => resolveGrip("__proto__")).toThrow(/Agarre desconocido/);
    expect(() => resolveGrip("belt_front")).toThrow(/belt_front: falta grip_side/);
    expect(() => resolveGrip("belt_back", { grip_side: "up" as never })).toThrow(/falta grip_side/);
    expect(() => resolveGrip("sleeve_left", { grip_side: "left" })).toThrow(/sleeve_left: no admite grip_side/);
    expect(() => resolveGrip("underhook_left")).toThrow(/underhook_left: falta toward_tori_m/);
    expect(() => resolveGrip("underhook_left", { toward_tori_m: [0, 0, 1] })).toThrow(/parte horizontal/);
    expect(() => resolveGrip("underhook_left", { toward_tori_m: [1, 0, 0], underhook_elevation_deg: 120 })).toThrow(/entre 0 y 90/);
    expect(() => resolveGrip("lapel_left", { toward_tori_m: [1, 0, 0] })).toThrow(/no admite toward_tori_m/);
    expect(() => resolveGrip("lapel_left", { toward_tori_m: [Number.NaN, 0, 0] })).toThrow(/no admite/);
  });

  it("resolver deja el lado contrario del cinturón y mantiene el reparto; los agarres sin lado no cambian", () => {
    const left = resolveGrip("belt_front", { grip_side: "left" });
    const right = resolveGrip("belt_front", { grip_side: "right" });
    expect(left.shares).toEqual([{ anchor: { kind: "site_lateral", site: "belt_front", side: "right" }, fraction: 1 }]);
    expect(right.shares).toEqual([{ anchor: { kind: "site_lateral", site: "belt_front", side: "left" }, fraction: 1 }]);
    expect(resolveGrip("lapel_left").shares.map((s) => s.fraction)).toEqual([0.7, 0.3]);
    expect(resolveGrip("waist_lock").axis).toBeNull();
  });

  it("los únicos agarres que no se apoyan solo en sitios del modelo son lapel_*, belt_* y waist_lock", () => {
    const notOnlySites = GRIP_NAMES.filter((n) => GRIP_CATALOG[n].shares.some((s) => s.anchor.kind !== "site"));
    expect(notOnlySites.sort()).toEqual(["belt_back", "belt_front", "lapel_left", "lapel_right", "waist_lock"]);
  });
});

describe("reparto de la fuerza y punto equivalente (puro)", () => {
  const FORCES: Vec3[] = [[100, 0, 0], [0, -37.5, 12], [123.456, -78.9, 456.7], [-500, 250, -250], [0, 0, 0]];

  it("la suma de las fuerzas repartidas es la fuerza de la mano, para todos los agarres", () => {
    for (const { name, options } of ALL_REQUESTS) {
      const grip = resolveGrip(name, options);
      for (const f of FORCES) {
        const parts = splitGripForce(grip.shares, f);
        expect(parts.length).toBe(grip.shares.length);
        const sum = parts.reduce(add, [0, 0, 0] as Vec3);
        for (let k = 0; k < 3; k++) expect(Math.abs(sum[k]! - f[k]!), `${name} ${f}`).toBeLessThanOrEqual(1e-12 * (1 + norm(f)));
        parts.forEach((p, i) => { // cada parte es fracción · F
          for (let k = 0; k < 3; k++) expect(p[k]).toBeCloseTo(grip.shares[i]!.fraction * f[k]!, 12);
        });
      }
    }
  });

  it("rechaza repartos que no suman 1, fracciones no positivas y fuerzas no finitas", () => {
    expect(() => splitGripForce([{ fraction: 0.7 }, { fraction: 0.2 }], [1, 0, 0])).toThrow(/suman/);
    expect(() => splitGripForce([{ fraction: 1.5 }, { fraction: -0.5 }], [1, 0, 0])).toThrow(/fracción inválida/);
    expect(() => splitGripForce([], [1, 0, 0])).toThrow(/no tiene puntos/);
    expect(() => splitGripForce([{ fraction: 1 }], [Number.NaN, 0, 0])).toThrow(/force_n/);
  });

  it("el punto equivalente es la media ponderada de posiciones y de velocidades", () => {
    const shares = [{ fraction: 0.7 }, { fraction: 0.3 }];
    const eq = equivalentGripPoint(shares, [[1, 2, 3], [11, -2, 0]], [[0, 1, 0], [10, 0, -10]]);
    expect(eq.point_m[0]).toBeCloseTo(0.7 * 1 + 0.3 * 11, 12);
    expect(eq.point_m[1]).toBeCloseTo(0.7 * 2 - 0.3 * 2, 12);
    expect(eq.point_m[2]).toBeCloseTo(0.7 * 3, 12);
    expect(eq.velocity_mps[0]).toBeCloseTo(3, 12);
    expect(eq.velocity_mps[1]).toBeCloseTo(0.7, 12);
    expect(eq.velocity_mps[2]).toBeCloseTo(-3, 12);
    const one = equivalentGripPoint([{ fraction: 1 }], [[4, 5, 6]], [[7, 8, 9]]);
    expect(one).toEqual({ point_m: [4, 5, 6], velocity_mps: [7, 8, 9] });
    expect(() => equivalentGripPoint(shares, [[0, 0, 0]], [[0, 0, 0]])).toThrow(/una posición y una velocidad/);
  });

  it("el par respecto a un punto es Σ (p − c) × F y es lineal en el reparto", () => {
    const p: Vec3[] = [[0, 0, 1], [0, 1, 0]];
    const f: Vec3[] = [[10, 0, 0], [0, 0, 10]];
    // (0,0,1)×(10,0,0) = (0,10,0); (0,1,0)×(0,0,10) = (10,0,0)  ->  (10,10,0)
    expect(gripTorqueAbout(p, f, [0, 0, 0])).toEqual([10, 10, 0]);
    expect(gripTorqueAbout(p, f, [0, 0, 1])[1]).toBeCloseTo(0 + 0, 12); // la primera fuerza pasa por el punto
  });
});

describe("eje de underhook: solo hacia arriba y hacia Tori", () => {
  it("el eje es unitario, sin componente lateral y con las dos componentes no negativas, para cualquier posición de Tori", () => {
    for (const toward of [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [3, 4, 0.5], [-2, 1, -3]] as Vec3[]) {
      for (const e of [0, 10, UNDERHOOK_DEFAULT_ELEVATION_DEG, 80, 90]) {
        const axis = underhookAxis(toward, e);
        expect(norm(axis)).toBeCloseTo(1, 12);
        expect(isUnderhookAxis(axis, toward), `${toward} ${e}`).toBe(true);
        expect(axis[2]).toBeCloseTo(Math.sin((e * Math.PI) / 180), 12); // elevación
        const h = Math.hypot(toward[0], toward[1]);
        expect(dot(axis, [toward[0] / h, toward[1] / h, 0])).toBeCloseTo(Math.cos((e * Math.PI) / 180), 12); // hacia Tori
      }
    }
    // «Hacia Tori» es un dato de quien llama: el eje gira con él (Tori delante y Tori detrás dan ejes opuestos en X).
    expect(underhookAxis([2, 0, 0])[0]).toBeGreaterThan(0);
    expect(underhookAxis([-2, 0, 0])[0]).toBeLessThan(0);
    expect(underhookAxis([0, 2, 0])[1]).toBeGreaterThan(0);
    expect(resolveGrip("underhook_right", { toward_tori_m: [0.5, -1, 0], underhook_elevation_deg: 30 }).axis)
      .toEqual(underhookAxis([0.5, -1, 0], 30));
  });

  it("isUnderhookAxis rechaza ejes hacia abajo, hacia atrás, laterales o no unitarios", () => {
    const toward: Vec3 = [1, 0, 0];
    const s = Math.SQRT1_2;
    expect(isUnderhookAxis([s, 0, s], toward)).toBe(true);
    expect(isUnderhookAxis([s, 0, -s], toward)).toBe(false); // hacia abajo
    expect(isUnderhookAxis([-s, 0, s], toward)).toBe(false); // alejándose de Tori
    expect(isUnderhookAxis([s, s, 0], toward)).toBe(false); // lateral
    expect(isUnderhookAxis([1, 0, 1], toward)).toBe(false); // no unitario
    expect(isUnderhookAxis([s, 0, s], [0, 0, 1])).toBe(false);
  });

  it("con la mano push de hand.ts, la fuerza solo puede salir hacia arriba y hacia Tori: probado con objetivos en todas direcciones", () => {
    const toward: Vec3 = [0.6, -0.8, 0];
    const grip = resolveGrip("underhook_left", { toward_tori_m: toward });
    const params = resolveHandParams(handConfigForGrip(grip, { shoulder_m: [1, -1, 1.3], reach_m: 100, max_force_n: 1e6, max_power_w: 1e9 }));
    expect(params.link).toBe("push");
    expect(params.axis).toEqual(grip.axis);
    const axis = grip.axis!;
    const lateral: Vec3 = [0.8, 0.6, 0]; // horizontal y perpendicular a `toward`
    let pushed = 0;
    let silent = 0;
    for (let az = 0; az < 360; az += 15) {
      for (let el = -90; el <= 90; el += 15) {
        const d: Vec3 = [
          Math.cos((el * Math.PI) / 180) * Math.cos((az * Math.PI) / 180), Math.cos((el * Math.PI) / 180) * Math.sin((az * Math.PI) / 180),
          Math.sin((el * Math.PI) / 180),
        ];
        const r = stepHand(params, createHandState(), {
          t: 0, dt: DT, grip_m: [0, 0, 1], grip_velocity_mps: [0, 0, 0],
        });
        void r;
        // Objetivo a 0,2 m del agarre en la dirección `d`, mediante un tramo ya cumplido (el origen es el agarre al cerrar).
        const moving = resolveHandParams(handConfigForGrip(grip, {
          shoulder_m: [1, -1, 1.3], reach_m: 100, max_force_n: 1e6, max_power_w: 1e9,
          phases: [{ start_s: 0, duration_s: 0.01, offset_m: [0.2 * d[0], 0.2 * d[1], 0.2 * d[2]] }],
        }));
        const f = stepHand(moving, createHandState(), { t: 1, dt: DT, grip_m: [0, 0, 1], grip_velocity_mps: [0, 0, 0] }).force_n;
        if (norm(f) === 0) { silent++; continue; }
        pushed++;
        expect(Math.abs(dot(f, lateral)), "componente lateral").toBeLessThanOrEqual(1e-9 * norm(f));
        expect(dot(f, [toward[0], toward[1], 0]), "hacia Tori").toBeGreaterThan(-1e-9);
        expect(f[2], "hacia arriba").toBeGreaterThan(-1e-9);
        expect(norm(sub(f, [axis[0] * norm(f), axis[1] * norm(f), axis[2] * norm(f)]))).toBeLessThanOrEqual(1e-9 * norm(f));
      }
    }
    expect(pushed).toBeGreaterThan(50); // la prueba no es vacía: hay empuje en muchas direcciones del objetivo
    expect(silent).toBeGreaterThan(50); // y hay objetivos que no producen fuerza (detrás o a los lados del eje)
  });
});

describe("espejo izquierda/derecha (puro)", () => {
  it("el espejo de cada agarre tiene las mismas fracciones y la misma unión; su espejo es el original", () => {
    for (const { name, options } of ALL_REQUESTS) {
      const mirrored = mirrorGripRequest(name, options);
      const a = resolveGrip(name, options);
      const b = resolveGrip(mirrored.name, mirrored.options);
      expect(b.link).toBe(a.link);
      expect(b.shares.map((s) => s.fraction).sort()).toEqual(a.shares.map((s) => s.fraction).sort());
      const back = mirrorGripRequest(mirrored.name, mirrored.options);
      expect(back.name).toBe(name);
      expect(back.options).toEqual(options);
      if (name.endsWith("_left")) expect(mirrored.name).toBe(name.replace("_left", "_right"));
      if (name.endsWith("_right")) expect(mirrored.name).toBe(name.replace("_right", "_left"));
    }
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// El catálogo contra el modelo generado (MuJoCo WASM real)
// ---------------------------------------------------------------------------------------------------------------------

describe("el catálogo contra el modelo generado (cuatro tallas)", () => {
  for (const size of SIZES) {
    const label = `${size.mass_kg} kg / ${size.height_m} m`;

    it(`${label}: todo sitio, cuerpo y geometría del catálogo existe y todos los agarres se resuelven`, async () => {
      await withUke(size, (mj, model, data) => {
        kinematics(mj, model, data);
        for (const request of ALL_REQUESTS) {
          const label2 = `${request.name} ${JSON.stringify(request.options)}`;
          const grip = locate(model, request); // lanza si falta algo
          expect(grip.shares.length, label2).toBeGreaterThan(0);
          for (const s of grip.shares) {
            expect(s.bodyId, label2).toBeGreaterThan(0);
            expect(s.local_m.every(Number.isFinite), label2).toBe(true);
          }
          for (const s of GRIP_CATALOG[request.name].shares) {
            if (s.anchor.kind === "site") {
              // El sitio es del cuerpo que dice `HUMANOID_SITE_BODY` (la tabla del esquema de escenarios).
              const id = model.site(s.anchor.site).id;
              expect(id, s.anchor.site).toBeGreaterThanOrEqual(0);
              expect(HUMANOID_SITE_BODY[s.anchor.site], s.anchor.site).toBeDefined();
              expect(model.body(HUMANOID_SITE_BODY[s.anchor.site]!).id, s.anchor.site).toBe(model.site_bodyid[id]);
            }
          }
        }
      });
    }, 30000);

    it(`${label}: un sitio que no existe da error claro`, async () => {
      await withUke(size, (_mj, model) => {
        const bad: ResolvedGrip = {
          name: "collar_back", link: "rope", axis: null,
          shares: [{ anchor: { kind: "site", site: "no_such_site" }, fraction: 1 }],
        };
        expect(() => locateGrip(asGripModel(model), bad)).toThrow(/collar_back: el modelo no tiene el sitio "no_such_site"/);
        const badBody: ResolvedGrip = {
          ...bad, shares: [{ anchor: { kind: "child_origin", body: "chest", child: "pelvis" }, fraction: 1 }],
        };
        expect(() => locateGrip(asGripModel(model), badBody)).toThrow(/no es hijo directo/);
      });
    }, 30000);

    it(`${label}: los puntos derivados caen sobre el cuerpo (pelvis, cinturón, hombro)`, async () => {
      await withUke(size, (mj, model, data) => {
        kinematics(mj, model, data);
        // waist_lock: extremos del semieje Y del elipsoide de la pelvis (sobre su superficie, simétricos).
        const waist = locate(model, { name: "waist_lock", options: {} });
        const pelvisGeom = model.geom("pelvis_geom").id;
        const [ax, ay, az] = [model.geom_size[3 * pelvisGeom]!, model.geom_size[3 * pelvisGeom + 1]!, model.geom_size[3 * pelvisGeom + 2]!];
        for (const s of waist.shares) {
          expect(s.bodyId).toBe(model.body("pelvis").id);
          const [x, y, z] = s.local_m;
          expect((x / ax) ** 2 + (y / ay) ** 2 + (z / az) ** 2).toBeCloseTo(1, 9);
        }
        expect(waist.shares[0]!.local_m[1]).toBeCloseTo(-waist.shares[1]!.local_m[1], 12);
        expect(waist.shares[0]!.local_m[1]).toBeGreaterThan(0.1); // el semieje Y de la pelvis es > 10 cm en todas las tallas
        // cinturón: sobre la cápsula del abdomen (mismo radio y altura que el sitio belt_front), a cada lado.
        for (const side of ["left", "right"] as const) {
          const belt = locate(model, { name: "belt_front", options: { grip_side: side } });
          const share = belt.shares[0]!;
          expect(share.bodyId).toBe(model.body("abdomen").id);
          const radius = model.geom_size[3 * model.geom("abdomen_geom").id]!;
          expect(Math.hypot(share.local_m[0], share.local_m[1])).toBeCloseTo(radius, 9);
          expect(share.local_m[2]).toBeCloseTo(model.site_pos[3 * model.site("belt_front").id + 2]!, 12);
          // El agarre por el lado `side` carga el contrario: izquierda (+Y) → punto con Y < 0.
          expect(Math.sign(share.local_m[1])).toBe(side === "left" ? -1 : 1);
        }
        // lapel: el hombro contrario, en el sistema del pecho, cerca de la superficie del elipsoide del pecho.
        const lapel = locate(model, { name: "lapel_left", options: {} });
        const shoulder = lapel.shares[1]!;
        expect(shoulder.bodyId).toBe(model.body("chest").id);
        expect(shoulder.local_m[1]).toBeLessThan(0); // el hombro derecho
        const chestGeom = model.geom("chest_geom").id;
        const r = Math.sqrt(
          (shoulder.local_m[0] / model.geom_size[3 * chestGeom]!) ** 2 +
          (shoulder.local_m[1] / model.geom_size[3 * chestGeom + 1]!) ** 2 +
          ((shoulder.local_m[2] - model.geom_pos[3 * chestGeom + 2]!) / model.geom_size[3 * chestGeom + 2]!) ** 2,
        );
        expect(r).toBeGreaterThan(0.95);
        expect(r).toBeLessThan(1.1); // la articulación del hombro está ~7 % fuera del elipsoide del pecho
      });
    }, 30000);

    it(`${label}: izquierda y derecha son espejo exacto (mismos puntos reflejados en Y, mismas fracciones)`, async () => {
      await withUke(size, (mj, model, data) => {
        kinematics(mj, model, data);
        let checked = 0;
        for (const { name, options } of ALL_REQUESTS) {
          const mirrored = mirrorGripRequest(name, options);
          const a = locate(model, { name, options });
          const b = locate(model, mirrored);
          const pa = gripPointPositions(asGripData(data), a);
          const pb = gripPointPositions(asGripData(data), b);
          // Cada punto de A, reflejado, es un punto de B con la misma fracción (waist_lock se refleja en sí mismo).
          a.shares.forEach((sa, i) => {
            const j = pb.findIndex((p, k) => norm(sub(p, mirrorY(pa[i]!))) < 1e-9 && b.shares[k]!.fraction === sa.fraction);
            expect(j, `${name}: no hay espejo del punto ${i}`).toBeGreaterThanOrEqual(0);
            expect(norm(sub(pb[j]!, mirrorY(pa[i]!)))).toBeLessThan(1e-12);
          });
          expect(b.shares.length).toBe(a.shares.length);
          checked++;
        }
        expect(checked).toBe(ALL_REQUESTS.length);
      });
    }, 30000);

    it(`${label}: aplicar el reparto suma exactamente la fuerza de la mano en xfrc_applied y solo toca los cuerpos del reparto`, async () => {
      await withUke(size, (mj, model, data) => {
        kinematics(mj, model, data);
        const force: Vec3 = [120, -80, 200];
        for (const request of ALL_REQUESTS) {
          const grip = locate(model, request);
          const xfrc = data.xfrc_applied as Float64Array;
          xfrc.fill(0);
          const parts = applyGripForce(asGripData(data), grip, force);
          const sum = [0, 0, 0];
          const touched = new Set<number>();
          for (let b = 0; b < model.nbody; b++) {
            for (let k = 0; k < 3; k++) sum[k] = sum[k]! + xfrc[6 * b + k]!;
            if (xfrc.slice(6 * b, 6 * b + 6).some((x) => x !== 0)) touched.add(b);
          }
          for (let k = 0; k < 3; k++) expect(sum[k]).toBeCloseTo(force[k]!, 9);
          expect(parts.reduce(add, [0, 0, 0] as Vec3)[2]).toBeCloseTo(200, 9);
          expect([...touched].sort()).toEqual([...new Set(grip.shares.map((s) => s.bodyId))].sort());
          // El par de cada cuerpo es (p − c) × F de sus puntos (la matemática de applyForceAtPoint).
          for (const bodyId of new Set(grip.shares.map((s) => s.bodyId))) {
            const mine = grip.shares.map((s, i) => ({ s, i })).filter(({ s }) => s.bodyId === bodyId);
            const pts = gripPointPositions(asGripData(data), { ...grip, shares: mine.map(({ s }) => s) });
            const c: Vec3 = [data.xipos[3 * bodyId]!, data.xipos[3 * bodyId + 1]!, data.xipos[3 * bodyId + 2]!];
            const tau = gripTorqueAbout(pts, mine.map(({ i }) => parts[i]!), c);
            for (let k = 0; k < 3; k++) expect(xfrc[6 * bodyId + 3 + k]).toBeCloseTo(tau[k]!, 9);
          }
        }
      });
    }, 30000);
  }
});

// ---------------------------------------------------------------------------------------------------------------------
// El par respecto al centro de masas cambia según el reparto
// ---------------------------------------------------------------------------------------------------------------------

describe("el par respecto al centro de masas de Uke cambia según el reparto (modelo real, de pie en neutra)", () => {
  const FORCE: Vec3 = [300, 0, 0];
  for (const size of SIZES) {
    it(`${size.mass_kg} kg / ${size.height_m} m: 300 N hacia delante por solapa, manga y cuello dan pares distintos`, async () => {
      await withUke(size, (mj, model, data) => {
        applyStance(mj, model, data, "neutral");
        kinematics(mj, model, data);
        const c = comOf(data);
        const torque = (name: GripName): Vec3 => {
          const grip = locate(model, { name, options: optionsFor(name) });
          return gripTorqueAbout(gripPointPositions(asGripData(data), grip), splitGripForce(grip.shares, FORCE), c);
        };
        const lapel = torque("lapel_left");
        const sleeve = torque("sleeve_left");
        const collar = torque("collar_back");
        console.log(`par (N·m) sobre el CM con 300 N hacia +X, ${size.mass_kg} kg / ${size.height_m} m:\n` +
          `  lapel_left   [${fixed(lapel)}]\n  sleeve_left  [${fixed(sleeve)}]\n  collar_back  [${fixed(collar)}]`);
        // Cálculo independiente: por linealidad, el par es (Σ f_i p_i − c) × F.
        const grip = locate(model, { name: "lapel_left", options: {} });
        const p = gripPointPositions(asGripData(data), grip);
        const mean: Vec3 = [0, 0, 0].map((_, k) => 0.7 * p[0]![k]! + 0.3 * p[1]![k]!) as unknown as Vec3;
        const r = sub(mean, c);
        const manual: Vec3 = [r[1] * FORCE[2] - r[2] * FORCE[1], r[2] * FORCE[0] - r[0] * FORCE[2], r[0] * FORCE[1] - r[1] * FORCE[0]];
        for (let k = 0; k < 3; k++) expect(lapel[k]).toBeCloseTo(manual[k]!, 9);
        // Cabeceo: la solapa y el cuello (más altos) lo hacen mayor que la manga (a la altura del codo).
        expect(lapel[1]).toBeGreaterThan(1.5 * sleeve[1]);
        expect(collar[1]).toBeGreaterThan(lapel[1]);
        // Guiñada: la manga izquierda (+Y) gira a un lado y la solapa izquierda, que carga el hombro derecho, al contrario.
        expect(Math.sign(sleeve[2])).toBe(-1);
        expect(Math.sign(lapel[2])).toBe(1);
        expect(collar[2]).toBeCloseTo(0, 6); // el cuello está en la línea media
        expect(norm(sub(lapel, sleeve))).toBeGreaterThan(0.25 * norm(lapel));
        expect(norm(sub(lapel, collar))).toBeGreaterThan(0.05 * norm(lapel));
      });
    }, 30000);
  }
});

// ---------------------------------------------------------------------------------------------------------------------
// Dinámica con MuJoCo WASM real
// ---------------------------------------------------------------------------------------------------------------------

describe("Uke relaxed en caída libre: el reparto cambia la respuesta del tronco (MuJoCo WASM real)", () => {
  const FORCE: Vec3 = [300, 0, 0];
  const STEPS = 40; // 80 ms

  for (const size of SIZES) {
    it(`${size.mass_kg} kg / ${size.height_m} m: la misma fuerza por solapa y por manga da aceleraciones angulares distintas del tronco, en el sentido que predice el reparto`, async () => {
      const lapel = await freeFall(size, { name: "lapel_left", options: {} }, FORCE, STEPS);
      const sleeve = await freeFall(size, { name: "sleeve_left", options: {} }, FORCE, STEPS);
      const dTau = sub(lapel.predictedTorque, sleeve.predictedTorque);
      const dAlpha = sub(lapel.chestAlpha, sleeve.chestAlpha);
      console.log(
        `caída libre ${size.mass_kg} kg / ${size.height_m} m, 300 N hacia +X durante ${(STEPS * DT * 1000).toFixed(0)} ms:\n` +
        `  par previsto  lapel [${fixed(lapel.predictedTorque)}] manga [${fixed(sleeve.predictedTorque)}] N·m\n` +
        `  dL/dt medido  lapel [${fixed(lapel.torqueMeasured)}] manga [${fixed(sleeve.torqueMeasured)}] N·m\n` +
        `  aceleración angular del tronco  lapel [${fixed(lapel.chestAlpha)}] manga [${fixed(sleeve.chestAlpha)}] rad/s²\n` +
        `  diferencia solapa − manga: par previsto [${fixed(dTau)}], tronco [${fixed(dAlpha)}]`);
      // 1. El par aplicado es el previsto: el momento angular total respecto al CM es la integral de Σ (p − c) × F
      //    calculada con los puntos y el CM de cada paso (la gravedad no da par respecto al CM y no hay contactos).
      //    El margen del 3 % es la discretización del integrador: con el antebrazo oscilando, el error es proporcional
      //    al paso (medido una vez con paso/4: baja de 0,14 a 0,035 N·m·s con la manga de 55 kg).
      for (const run of [lapel, sleeve]) {
        const err = norm(sub(run.angmom, run.angularImpulse));
        console.log(`  momento angular medido [${fixed(run.angmom, 3)}] frente a ∫par dt [${fixed(run.angularImpulse, 3)}] N·m·s; error ${err.toExponential(2)}`);
        expect(err).toBeLessThan(0.03 * norm(run.angularImpulse));
      }
      // 2. La respuesta del tronco difiere en el sentido de la diferencia de pares (cabeceo, eje Y; guiñada, eje Z).
      for (const k of [1, 2]) {
        expect(Math.abs(dTau[k]!), "el reparto debe cambiar el par").toBeGreaterThan(50);
        expect(Math.sign(dAlpha[k]!), `eje ${k}: signo`).toBe(Math.sign(dTau[k]!));
        expect(Math.abs(dAlpha[k]!), `eje ${k}: tamaño`).toBeGreaterThan(10);
      }
    }, 30000);
  }
});

describe("simetría dinámica: agarre izquierdo con la fuerza reflejada en Y da el resultado reflejado del derecho (MuJoCo WASM real)", () => {
  const CASES: Array<{ name: GripName; options: ResolveGripOptions; force: Vec3 }> = [
    { name: "lapel_left", options: {}, force: [200, 100, 150] },
    { name: "sleeve_left", options: {}, force: [200, 100, 150] },
    { name: "belt_front", options: { grip_side: "left" }, force: [200, 100, 50] },
    { name: "waist_lock", options: {}, force: [200, 100, 50] },
    { name: "underhook_left", options: { toward_tori_m: [1, 0, 0] }, force: [60, 0, 120] },
    { name: "knee_back_left", options: {}, force: [-150, 100, 50] },
  ];
  for (const c of CASES) {
    it(`${c.name}: posiciones de todos los cuerpos y velocidad angular del tronco reflejadas (error < 1e-9)`, async () => {
      const size = SIZES[1];
      const a = await freeFall(size, { name: c.name, options: c.options }, c.force, 40);
      const m = mirrorGripRequest(c.name, c.options);
      const b = await freeFall(size, m, mirrorY(c.force), 40);
      let worst = 0;
      for (const [name, p] of a.bodies) worst = Math.max(worst, norm(sub(mirrorY(p), b.bodies.get(mirrorBodyName(name))!)));
      const wErr = norm(sub(mirrorAxialY(a.chestOmega), b.chestOmega));
      console.log(`espejo ${c.name} (fuerza [${c.force}]): mayor diferencia de posición ${worst.toExponential(2)} m, de ω del tronco ${wErr.toExponential(2)} rad/s; |ω tronco| ${norm(a.chestOmega).toFixed(3)} rad/s`);
      expect(worst).toBeLessThan(1e-9);
      expect(wErr).toBeLessThan(1e-9);
      // La prueba no es vacía: el golpe mueve de verdad el cuerpo, y el resultado no es simétrico en sí mismo
      // (sin reflejar, el izquierdo y el derecho difieren).
      let asymmetry = 0;
      for (const [name, p] of a.bodies) asymmetry = Math.max(asymmetry, norm(sub(p, b.bodies.get(name)!)));
      expect(asymmetry).toBeGreaterThan(1e-3);
    }, 30000);
  }
});

// ---------------------------------------------------------------------------------------------------------------------
// Punto equivalente y mano
// ---------------------------------------------------------------------------------------------------------------------

describe("punto de agarre equivalente con el modelo real y la mano virtual", () => {
  it("la velocidad de cada punto (cvel) coincide con la derivada numérica de su posición (error < 1 µm/s)", async () => {
    await withUke(SIZES[1], (mj, model, data) => {
      // Uke en el aire con velocidades en tronco, brazo y pelvis, para que los puntos del reparto se muevan distinto.
      data.qpos[2] = data.qpos[2]! + 1;
      data.qvel[0] = 0.3; data.qvel[2] = 0.2; data.qvel[3] = 0.5; data.qvel[4] = -0.4; data.qvel[5] = 0.6;
      data.qvel[model.jnt_dofadr[model.jnt("thorax_yaw").id]!] = 1.5;
      data.qvel[model.jnt_dofadr[model.jnt("neck_pitch").id]!] = -2;
      data.qvel[model.jnt_dofadr[model.jnt("right_shoulder_pitch").id]!] = 2.5;
      data.qvel[model.jnt_dofadr[model.jnt("left_shoulder_roll").id]!] = -1.5;
      const grip = locate(model, { name: "lapel_left", options: {} });
      const sample = (): { points_m: Vec3[]; velocities_mps: Vec3[] } => {
        kinematics(mj, model, data);
        return gripPointStates(asGripModel(model), asGripData(data), grip);
      };
      const before = sample();
      // Derivada numérica exacta de la posición con la velocidad generalizada actual: p(q ⊕ h·v) − p(q ⊖ h·v), h = 1e-6 s.
      const qpos0 = Float64Array.from(data.qpos as Float64Array);
      const qvel = Array.from(data.qvel as Float64Array);
      const shifted = (h: number): Vec3[] => {
        data.qpos.set(qpos0);
        mj.mj_integratePos(model, data.qpos, qvel, h);
        kinematics(mj, model, data);
        return gripPointPositions(asGripData(data), grip);
      };
      const h = 1e-6;
      const plus = shifted(h);
      const minus = shifted(-h);
      data.qpos.set(qpos0);
      before.points_m.forEach((_p, i) => {
        const numeric: Vec3 = [0, 1, 2].map((k) => (plus[i]![k]! - minus[i]![k]!) / (2 * h)) as unknown as Vec3;
        const v = before.velocities_mps[i]!;
        const err = norm(sub(numeric, v));
        console.log(`punto ${i} del reparto: |v| ${norm(v).toFixed(3)} m/s, error frente a la derivada numérica ${(err * 1000).toExponential(2)} mm/s`);
        expect(norm(v)).toBeGreaterThan(0.3);
        expect(err).toBeLessThan(1e-6);
      });
      const eq = equivalentGripPoint(grip.shares, before.points_m, before.velocities_mps);
      expect(eq.point_m[2]).toBeCloseTo(0.7 * before.points_m[0]![2] + 0.3 * before.points_m[1]![2], 12);
    });
  });

  it("lapel_left con la mano rope en caída libre: en cada paso la suma de xfrc_applied es la fuerza de la mano y el tope se respeta", async () => {
    const size = SIZES[1];
    await withUke(size, (mj, model, data) => {
      setJointDamping(model as unknown as DampingModel, relaxedJointDamping(size.mass_kg, size.height_m));
      applyStance(mj, model, data, "neutral");
      data.qpos[2] = data.qpos[2]! + 1;
      mj.mj_forward(model, data);
      const grip = locate(model, { name: "lapel_left", options: {} });
      const params = resolveHandParams(handConfigForGrip(grip, {
        shoulder_m: [1.2, 0.2, 2.5], reach_m: 1.5,
        phases: [{ start_s: 0.02, duration_s: 0.2, offset_m: [0.25, 0, 0.1] }],
      }));
      expect(params.link).toBe("rope");
      let state = createHandState();
      const xfrc = data.xfrc_applied as Float64Array;
      let peak = 0;
      let worstSum = 0;
      for (let i = 0; i < 150; i++) {
        xfrc.fill(0);
        kinematics(mj, model, data);
        const { points_m, velocities_mps } = gripPointStates(asGripModel(model), asGripData(data), grip);
        const eq = equivalentGripPoint(grip.shares, points_m, velocities_mps);
        const r = stepHand(params, state, { t: i * DT, dt: DT, grip_m: eq.point_m, grip_velocity_mps: eq.velocity_mps });
        state = r.state;
        applyGripForce(asGripData(data), grip, r.force_n);
        const sum = [0, 0, 0];
        for (let b = 0; b < model.nbody; b++) for (let k = 0; k < 3; k++) sum[k] = sum[k]! + xfrc[6 * b + k]!;
        worstSum = Math.max(worstSum, norm(sub([sum[0]!, sum[1]!, sum[2]!], r.force_n)));
        peak = Math.max(peak, norm(r.force_n));
        mj.mj_step(model, data);
      }
      console.log(`mano rope sobre lapel_left en caída libre: pico de fuerza ${peak.toFixed(1)} N, mayor diferencia entre la suma de xfrc_applied y la fuerza de la mano ${worstSum.toExponential(2)} N`);
      expect(peak).toBeGreaterThan(50); // la mano tira de verdad
      expect(peak).toBeLessThanOrEqual(params.max_force_n + 1e-9);
      expect(worstSum).toBeLessThan(1e-9);
    });
  }, 30000);
});
