import { describe, expect, it } from "vitest";
import {
  GRAVITY_MPS2, OBSTACLE_DEFAULTS, OBSTACLE_FRICTION, appendToWorldbody, applyObstacleCommand, buildObstacleMjcf,
  createObstacleState, obstacleInertia, obstacleStability, obstacleTarget, plannedPeakSpeedMps, readObstacleContactForce,
  readObstacleKinematics, resolveObstacleHandles, resolveObstacleParams, settleObstacleWork, stepObstacle,
  type ObstacleConfig, type ObstacleHandles, type ObstacleParams, type ObstacleSample, type ObstacleState, type Vec3,
} from "../../src/effectors/obstacle";
import { applyForceAtPoint } from "../../src/effectors/hand";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { PHYSICS_TIMESTEP_S, worldOptionTag } from "../../src/scenario/worldOptions";
import { createUkeController, type UkeControllerParams } from "../../src/uke/attitudes";
import { attitudeJointDamping, setJointDamping, type DampingModel } from "../../src/uke/damping";
import { generateHumanoidMJCF } from "../../src/uke/generate";
import { applyStance } from "../../src/uke/stances";
import type { StandingModel, StandingState } from "../../src/control/standingBaseline";

const DT = PHYSICS_TIMESTEP_S;
const TORI_KG = 75;
const TORI_WEIGHT_N = TORI_KG * GRAVITY_MPS2;
/** Topes por defecto con Tori de 75 kg. */
const CAP_V = OBSTACLE_DEFAULTS.vertical_cap_weights * TORI_WEIGHT_N;
const CAP_H = OBSTACLE_DEFAULTS.horizontal_cap_weights * TORI_WEIGHT_N;
/** Mismas opciones de colisión que `buildModel.ts` (simetría de la colisión convexa). */
const COLLISION_ATTRS = `ccd_tolerance="1e-14" ccd_iterations="100"`;
const CONTACT = `solref="0.015 1" solimp="0.9 0.95 0.001"`;

const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a: Vec3): number => Math.sqrt(dot(a, a));
const max = (xs: readonly number[]): number => xs.reduce((m, x) => Math.max(m, x), -Infinity);
const mm = (x: number): string => `${(1000 * x).toFixed(3)} mm`;

function barConfig(extra: Partial<ObstacleConfig> = {}): ObstacleConfig {
  return { name: "obs", shape: "pad", tori_mass_kg: TORI_KG, start_m: [0, 0, 1], ...extra };
}

// ---------------------------------------------------------------------------------------------------------------------
// Mundo de prueba en MuJoCo WASM real: suelo, un obstáculo y cuerpos extra (cargas, bloques).
// ---------------------------------------------------------------------------------------------------------------------

interface Step {
  t: number;
  sample: ObstacleSample;
  /** Velocidad del obstáculo antes y después del paso. */
  vBefore: Vec3;
  vAfter: Vec3;
  /** Fuerza de contacto del obstáculo SOBRE los demás cuerpos (no suelo), tras el paso. */
  contactOnOther: Vec3;
}

interface World {
  step(externalForce?: (data: { xfrc_applied: Float64Array }) => void): Step;
  readonly params: ObstacleParams;
  readonly handles: ObstacleHandles;
  state(): ObstacleState;
  data: Record<string, unknown> & { qpos: Float64Array; qvel: Float64Array };
  model: Record<string, unknown>;
  mj: Awaited<ReturnType<typeof getMujoco>>;
  steps(): number;
  dispose(): void;
}

function floorXml(): string {
  return `<geom name="floor" type="plane" size="5 5 0.1" friction="1 0.005 0.0001" ${CONTACT}/>`;
}

function blockXml(name: string, pos: Vec3, half: Vec3, massKg: number, friction = 1): string {
  return `<body name="${name}" pos="${pos.join(" ")}"><freejoint name="${name}_free"/>
    <geom name="${name}_geom" type="box" size="${half.join(" ")}" mass="${massKg}" friction="${friction} 0.005 0.0001" ${CONTACT}/></body>`;
}

async function makeWorld(
  configs: ObstacleConfig | ObstacleConfig[], extraXml = "", gravity = true,
): Promise<World> {
  const mj = await getMujoco();
  const list = Array.isArray(configs) ? configs : [configs];
  const all = list.map(resolveObstacleParams);
  const params = all[0]!;
  const xml = `<mujoco model="obstacle_probe"><compiler angle="radian"/>${worldOptionTag(COLLISION_ATTRS)}
    <worldbody>${floorXml()}
${all.map(buildObstacleMjcf).join("\n")}
${extraXml}
    </worldbody></mujoco>`;
  const model = mj.MjModel.from_xml_string(xml);
  const data = new mj.MjData(model);
  if (!gravity) (model.opt.gravity as Float64Array).fill(0);
  const handles = resolveObstacleHandles(model as never, params);
  const pyramidal = Boolean(mj.mj_isPyramidal(model));
  let state = createObstacleState();
  let contact: Vec3 = [0, 0, 0];
  let i = 0;
  const qvel = data.qvel as Float64Array;
  return {
    params, handles, mj, model: model as never, data: data as never,
    state: () => state,
    steps: () => i,
    step(external): Step {
      const t = i * DT;
      const vBefore: Vec3 = [qvel[handles.dofAdr[0]]!, qvel[handles.dofAdr[1]]!, qvel[handles.dofAdr[2]]!];
      const k = readObstacleKinematics(params, handles, data);
      const r = stepObstacle(params, state, { t, dt: DT, ...k, contact_force_on_uke_n: contact });
      state = r.state;
      (data.xfrc_applied as Float64Array).fill(0);
      (data.qfrc_applied as Float64Array).fill(0);
      external?.(data as never);
      applyObstacleCommand(handles, data, r);
      mj.mj_step(model, data);
      contact = readObstacleContactForce(model, handles, data as never, { pyramidal }).force_on_other_n;
      i++;
      return { t, sample: r.sample, vBefore, vAfter: [qvel[handles.dofAdr[0]]!, qvel[handles.dofAdr[1]]!, qvel[handles.dofAdr[2]]!], contactOnOther: contact };
    },
    dispose(): void {
      data.delete();
      model.delete();
    },
  };
}

async function withWorld<T>(configs: ObstacleConfig | ObstacleConfig[], extraXml: string, fn: (w: World) => T | Promise<T>, gravity = true): Promise<T> {
  const w = await makeWorld(configs, extraXml, gravity);
  try {
    return await fn(w);
  } finally {
    w.dispose();
  }
}

/** Posición del obstáculo (centro de la geometría) en el mundo. */
function posOf(w: World): Vec3 {
  const k = readObstacleKinematics(w.params, w.handles, w.data);
  return k.position_m;
}

/** Posición de un cuerpo libre `name` (primeros tres `qpos` de su articulación libre). */
function freePos(w: World, name: string): Vec3 {
  const id = (w.model as { jnt(n: string): { id: number } }).jnt(`${name}_free`).id;
  const adr = (w.model as { jnt_qposadr: Int32Array }).jnt_qposadr[id]!;
  return [w.data.qpos[adr]!, w.data.qpos[adr + 1]!, w.data.qpos[adr + 2]!];
}
function freeDof(w: World, name: string): number {
  const id = (w.model as { jnt(n: string): { id: number } }).jnt(`${name}_free`).id;
  return (w.model as { jnt_dofadr: Int32Array }).jnt_dofadr[id]!;
}

// ---------------------------------------------------------------------------------------------------------------------
// Parámetros, trayectoria y MJCF (sin motor)
// ---------------------------------------------------------------------------------------------------------------------

describe("valores por defecto y validación", () => {
  it("los valores por defecto son los de la issue #76 con Tori de 75 kg", () => {
    expect(OBSTACLE_DEFAULTS).toMatchObject({ kp_n_per_m: 20000, damping_ratio: 1, mass_fraction_of_tori: 0.5 });
    expect(OBSTACLE_FRICTION).toEqual({ cloth: 0.8, skin: 0.5 });
    const p = resolveObstacleParams(barConfig({ shape: "bar" }));
    expect(p.mass_kg).toBe(37.5);
    expect(p.kp_n_per_m).toBe(20000);
    expect(p.damping_ns_per_m).toBeCloseTo(2 * Math.sqrt(20000 * 37.5), 9);
    expect(p.max_vertical_force_n).toBeCloseTo(2 * 75 * 9.81, 9);
    expect(p.max_horizontal_force_n).toBeCloseTo(0.8 * 75 * 9.81, 9);
    expect(p.friction).toBe(0.8);
    expect(p.max_speed_mps).toBeNull();
    expect(p.gravity_compensation).toBe(true);
    expect(p.geometry).toEqual({ kind: "capsule", axis: "y", length_m: 0.4, radius_m: 0.11 });
    expect(resolveObstacleParams(barConfig({ surface: "skin" })).friction).toBe(0.5);
    expect(resolveObstacleParams(barConfig({ surface: "skin", friction: 0.3 })).friction).toBe(0.3);
  });

  it("rechaza valores no válidos con el campo en el mensaje", () => {
    expect(() => resolveObstacleParams(barConfig({ name: "1x" }))).toThrow(/name/);
    expect(() => resolveObstacleParams(barConfig({ start_m: [0, Number.NaN, 0] }))).toThrow(/start_m/);
    expect(() => resolveObstacleParams(barConfig({ kp_n_per_m: 0 }))).toThrow(/kp_n_per_m/);
    expect(() => resolveObstacleParams(barConfig({ max_speed_mps: -1 }))).toThrow(/max_speed_mps/);
    expect(() => resolveObstacleParams(barConfig({ shape: "bar", size_m: [1, 1, 1] }))).toThrow(/size_m/);
    expect(() => resolveObstacleParams(barConfig({ shape: "pad", radius_m: 0.1 }))).toThrow(/radius_m/);
    expect(() => resolveObstacleParams({ name: "a", shape: "bar", start_m: [0, 0, 0] })).toThrow(/tori_mass_kg/);
    expect(() => resolveObstacleParams(barConfig({ max_vertical_force_n: 300 }))).toThrow(/peso propio/);
    expect(() => resolveObstacleParams(barConfig({
      phases: [{ start_s: 0, duration_s: 1, offset_m: [1, 0, 0] }, { start_s: 0.5, duration_s: 1, offset_m: [2, 0, 0] }],
    }))).toThrow(/solapa/);
    expect(() => resolveObstacleParams(barConfig({ phases: [{ start_s: 0, duration_s: 1, offset_m: [0, 0, 0], angle_rad: 1 }] }))).toThrow(/rotation/);
    expect(() => resolveObstacleParams(barConfig({ rotation: { axis: [0, 0, 0] } }))).toThrow(/rotation.axis/);
  });

  it("con masa y topes explícitos no hace falta la masa de Tori (segado)", () => {
    const p = resolveObstacleParams({
      name: "reap", shape: "post", start_m: [0, 0, 0.5], mass_kg: 8, max_force_n: 400, max_vertical_force_n: 300, max_speed_mps: 3,
    });
    expect(p).toMatchObject({ mass_kg: 8, max_horizontal_force_n: 400, max_vertical_force_n: 300, max_speed_mps: 3 });
  });

  it("estabilidad del esquema explícito: estable con la masa por defecto y con 1 kg; inestable con 0,1 kg", () => {
    const stab = (mass: number) => obstacleStability(resolveObstacleParams(barConfig({ mass_kg: mass })), DT);
    expect(stab(37.5).stable).toBe(true);
    expect(stab(1).stable).toBe(true);
    expect(stab(0.1).stable).toBe(false);
  });
});

describe("trayectoria objetivo", () => {
  const phases = [
    { start_s: 0.1, duration_s: 0.4, offset_m: [0.2, -0.1, 0] as Vec3 },
    { start_s: 0.6, duration_s: 0.3, offset_m: [0.2, -0.1, 0.05] as Vec3 },
  ];
  const p = { start_m: [1, 2, 3] as Vec3, phases };

  it("parte de start_m, mantiene entre tramos y acaba en el último desplazamiento", () => {
    expect(obstacleTarget(p, 0).position_m).toEqual([1, 2, 3]);
    expect(obstacleTarget(p, 0.55).position_m[0]).toBeCloseTo(1.2, 12);
    expect(obstacleTarget(p, 0.55).velocity_mps).toEqual([0, 0, 0]);
    expect(obstacleTarget(p, 5).position_m).toEqual([1.2, 1.9, 3.05]);
  });

  it("velocidad y aceleración son las derivadas de la posición (diferencias finitas)", () => {
    const h = 1e-6;
    for (const t of [0.15, 0.3, 0.45, 0.7, 0.85]) {
      const a = obstacleTarget(p, t - h), b = obstacleTarget(p, t + h), c = obstacleTarget(p, t);
      for (let i = 0; i < 3; i++) {
        expect(c.velocity_mps[i]!).toBeCloseTo((b.position_m[i]! - a.position_m[i]!) / (2 * h), 5);
        expect(c.acceleration_mps2[i]!).toBeCloseTo((b.velocity_mps[i]! - a.velocity_mps[i]!) / (2 * h), 4);
      }
    }
  });

  it("la velocidad máxima planificada es 1,875·|Δ|/T", () => {
    expect(plannedPeakSpeedMps([{ start_s: 0, duration_s: 0.5, offset_m: [1, 0, 0] }])).toBeCloseTo(3.75, 12);
    let peak = 0;
    for (let t = 0; t <= 1; t += 0.0005) peak = Math.max(peak, norm(obstacleTarget(p, t).velocity_mps));
    expect(peak).toBeCloseTo(plannedPeakSpeedMps(phases), 2);
  });
});

describe("MJCF del obstáculo (MuJoCo WASM real)", () => {
  it("compila con 3 deslizaderas (+ bisagra), masa e inercia de la geometría, y filtro de colisión 2/1", async () => {
    const mj = await getMujoco();
    for (const shape of ["bar", "post", "pad", "foot"] as const) {
      const params = resolveObstacleParams(barConfig({ shape, rotation: shape === "bar" ? {} : undefined }));
      const xml = `<mujoco><compiler angle="radian"/>${worldOptionTag()}<worldbody>${floorXml()}${buildObstacleMjcf(params)}</worldbody></mujoco>`;
      const model = mj.MjModel.from_xml_string(xml);
      try {
        expect(model.nv).toBe(shape === "bar" ? 4 : 3);
        expect(model.body_mass[1]).toBeCloseTo(37.5, 9);
        const diag = Array.from((model.body_inertia as Float64Array).subarray(3, 6)).sort((x: number, y: number) => x - y);
        const mine = [...obstacleInertia(params.geometry, 37.5)].sort((x: number, y: number) => x - y);
        for (let i = 0; i < 3; i++) expect(diag[i]!).toBeCloseTo(mine[i]!, 6);
        const g = model.geom(`${params.name}_geom`).id;
        expect(model.geom_contype[g]).toBe(2);
        expect(model.geom_conaffinity[g]).toBe(1);
        expect(model.geom_priority[g]).toBe(1);
        expect(model.geom_friction[3 * g]).toBeCloseTo(0.8, 12);
        for (const j of ["x", "y", "z"]) expect(model.jnt_type[model.jnt(`${params.name}_${j}`).id]).toBe(2); // slide
      } finally {
        model.delete();
      }
    }
  });

  it("dos obstáculos solapados no chocan entre sí; cada uno sí choca con el suelo y con un cuerpo de Uke", async () => {
    const mj = await getMujoco();
    const a = resolveObstacleParams(barConfig({ name: "a", shape: "pad", start_m: [0, 0, 0.1] }));
    const b = resolveObstacleParams(barConfig({ name: "b", shape: "pad", start_m: [0.05, 0, 0.1] }));
    const xml = `<mujoco><compiler angle="radian"/>${worldOptionTag()}<worldbody>${floorXml()}${buildObstacleMjcf(a)}${buildObstacleMjcf(b)}</worldbody></mujoco>`;
    const model = mj.MjModel.from_xml_string(xml);
    const data = new mj.MjData(model);
    try {
      mj.mj_forward(model, data);
      const pairs = new Set<string>();
      const contacts = data.contact;
      for (let c = 0; c < contacts.size(); c++) {
        const ct = contacts.get(c)!;
        pairs.add([model.geom_bodyid[ct.geom1]!, model.geom_bodyid[ct.geom2]!].sort().join("-"));
        ct.delete();
      }
      contacts.delete();
      // Cuerpos: 1 = a, 2 = b; 0 = mundo (suelo). Los pads miden 0,4 m de alto: tocan el suelo (z = 0,1 - 0,2 < 0).
      expect(pairs.has("0-1")).toBe(true);
      expect(pairs.has("0-2")).toBe(true);
      expect(pairs.has("1-2")).toBe(false);
    } finally {
      data.delete();
      model.delete();
    }
  });

  it("insertarlo tras Uke no cambia el cuerpo 1 (pelvis) ni los seis primeros grados de libertad; extraWorldXml sí los cambia", async () => {
    const mj = await getMujoco();
    const params = resolveObstacleParams(barConfig({ shape: "bar", start_m: [3, 0, 1] }));
    const uke = generateHumanoidMJCF({ mass_kg: 75, height_m: 1.75 });
    const after = mj.MjModel.from_xml_string(appendToWorldbody(uke, buildObstacleMjcf(params)));
    const before = mj.MjModel.from_xml_string(generateHumanoidMJCF({ mass_kg: 75, height_m: 1.75 }, { extraWorldXml: buildObstacleMjcf(params) }));
    try {
      expect(after.body(1).name).toBe("pelvis");
      expect(after.jnt_dofadr[after.jnt("pelvis_root").id]).toBe(0);
      expect(before.body(1).name).toBe("obs"); // por eso el obstáculo NO puede ir en extraWorldXml
      expect(before.jnt_dofadr[before.jnt("pelvis_root").id]).not.toBe(0);
    } finally {
      after.delete();
      before.delete();
    }
  });

  it("no cambia los contactos de Uke con el suelo: mismo asentamiento con y sin obstáculo lejano", async () => {
    const mj = await getMujoco();
    const base = generateHumanoidMJCF({ mass_kg: 75, height_m: 1.75 });
    const withObs = appendToWorldbody(base, buildObstacleMjcf(resolveObstacleParams(barConfig({ shape: "bar", start_m: [3, 0, 1] }))));
    // Solo cuentan los contactos de Uke (el obstáculo, sin orden, cae por su peso al suelo: ese contacto no es de Uke).
    const settle = (xml: string): { qpos: number[]; contacts: number } => {
      const model = mj.MjModel.from_xml_string(xml);
      const data = new mj.MjData(model);
      try {
        applyStance(mj, model, data, "neutral");
        for (let i = 0; i < 250; i++) mj.mj_step(model, data);
        const bodyOfGeom = model.geom_bodyid as Int32Array;
        let contacts = 0;
        const list = data.contact;
        for (let c = 0; c < list.size(); c++) {
          const ct = list.get(c)!;
          if (model.body(bodyOfGeom[ct.geom1]!).name !== "obs" && model.body(bodyOfGeom[ct.geom2]!).name !== "obs") contacts++;
          ct.delete();
        }
        list.delete();
        return { qpos: Array.from((data.qpos as Float64Array).subarray(0, 35)), contacts };
      } finally {
        data.delete();
        model.delete();
      }
    };
    const a = settle(base);
    const b = settle(withObs);
    expect(b.contacts).toBe(a.contacts);
    expect(a.contacts).toBeGreaterThan(0);
    for (let i = 0; i < 35; i++) expect(b.qpos[i]!).toBeCloseTo(a.qpos[i]!, 9);
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// Seguimiento sin Uke
// ---------------------------------------------------------------------------------------------------------------------

/** Trayectoria de la prueba: avance de 15 cm en 0,4 s, subida de 5 cm y desvío lateral de 10 cm. */
const TRAJECTORY = [
  { start_s: 0.1, duration_s: 0.4, offset_m: [0.15, 0, 0] as Vec3 },
  { start_s: 0.6, duration_s: 0.3, offset_m: [0.15, 0, 0.05] as Vec3 },
  { start_s: 1.0, duration_s: 0.4, offset_m: [0.15, 0.1, 0.05] as Vec3 },
];

async function trackingError(extra: Partial<ObstacleConfig>): Promise<{ maxErr: number; steps: Step[]; w: ObstacleState }> {
  return withWorld(barConfig({ shape: "bar", phases: TRAJECTORY, ...extra }), "", (w) => {
    const steps: Step[] = [];
    for (let i = 0; i < 1000; i++) steps.push(w.step());
    return { maxErr: max(steps.map((s) => s.sample.deviation_magnitude_m)), steps, w: w.state() };
  });
}

describe("seguimiento sin Uke (MuJoCo WASM real)", () => {
  it("sigue su trayectoria con error < 5 mm (con compensación del peso y alimentación directa)", async () => {
    const { maxErr, steps } = await trackingError({});
    const last = steps[steps.length - 1]!.sample;
    console.log(`seguimiento: error máx ${mm(maxErr)}, pico de velocidad planificada ${plannedPeakSpeedMps(TRAJECTORY).toFixed(3)} m/s, error final ${mm(last.deviation_magnitude_m)}`);
    expect(maxErr).toBeLessThan(0.005);
    expect(steps.every((s) => !s.sample.saturated)).toBe(true);
  });

  it("sin alimentación directa, el retardo del PD (c·v/kp) supera los 5 mm: la alimentación directa es lo que cumple el criterio", async () => {
    const { maxErr } = await trackingError({ feedforward: false });
    console.log(`sin alimentación directa: error máx ${mm(maxErr)}`);
    expect(maxErr).toBeGreaterThan(0.005);
  });

  it("sin compensación del peso, el obstáculo cuelga m·g/kp bajo su objetivo; con ella, la orden vertical en reposo es m·g", async () => {
    const sag = await withWorld(barConfig({ gravity_compensation: false }), "", (w) => {
      for (let i = 0; i < 1500; i++) w.step();
      return 1 - posOf(w)[2];
    });
    expect(sag).toBeCloseTo((37.5 * GRAVITY_MPS2) / 20000, 3); // 1,84 cm
    const hold = await withWorld(barConfig(), "", (w) => {
      let last: Step | undefined;
      for (let i = 0; i < 500; i++) last = w.step();
      return { z: posOf(w)[2], fz: last!.sample.force_n[2] };
    });
    expect(hold.z).toBeCloseTo(1, 9);
    expect(hold.fz).toBeCloseTo(37.5 * GRAVITY_MPS2, 6);
  });

  it("el obstáculo nunca se queda fijo: sin la orden, cae por su peso", async () => {
    // Prueba de que la compensación es una fuerza del actuador (consume tope) y no `gravcomp` ni un cuerpo fijo.
    const fall = await withWorld(barConfig(), "", (w) => {
      for (let i = 0; i < 250; i++) {
        (w.data.qfrc_applied as unknown as Float64Array).fill(0);
        (w.mj as { mj_step(m: unknown, d: unknown): void }).mj_step(w.model, w.data);
      }
      return 1 - posOf(w)[2];
    });
    expect(fall).toBeGreaterThan(0.4); // 0,5 s de caída libre: 1,2 m menos el suelo
  });

  it("la bisagra sigue su ángulo con error < 0,01 rad y respeta su tope de par", async () => {
    const config = barConfig({
      shape: "bar", rotation: { axis: [0, 1, 0] },
      phases: [{ start_s: 0.1, duration_s: 0.4, offset_m: [0, 0, 0], angle_rad: 0.5 }, { start_s: 0.7, duration_s: 0.3, offset_m: [0, 0, 0], angle_rad: -0.2 }],
    });
    const result = await withWorld(config, "", (w) => {
      const steps: Step[] = [];
      for (let i = 0; i < 700; i++) steps.push(w.step());
      return {
        angleErr: max(steps.map((s) => Math.abs(s.sample.angle_deviation_rad))),
        torque: max(steps.map((s) => Math.abs(s.sample.torque_nm))),
        posErr: max(steps.map((s) => s.sample.deviation_magnitude_m)),
        cap: w.params.rotation!.max_torque_nm,
      };
    });
    console.log(`bisagra: error de ángulo máx ${result.angleErr.toFixed(5)} rad, par máx ${result.torque.toFixed(1)} N·m (tope ${result.cap.toFixed(1)}), error de posición ${mm(result.posErr)}`);
    expect(result.angleErr).toBeLessThan(0.01);
    expect(result.torque).toBeLessThanOrEqual(result.cap + 1e-9);
    expect(result.posErr).toBeLessThan(0.005);
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// Cargas verticales: peso de 3 y 0,5 veces el tope vertical
// ---------------------------------------------------------------------------------------------------------------------

async function loadRun(loadWeightN: number, seconds: number): Promise<{ steps: Step[]; maxSag: number; finalSag: number; blockZ: number }> {
  const mass = loadWeightN / GRAVITY_MPS2;
  const padTop = 1 + 0.2;
  return withWorld(
    barConfig(), blockXml("load", [0, 0, padTop + 0.1 + 0.0005], [0.15, 0.15, 0.1], mass),
    (w) => {
      const steps: Step[] = [];
      for (let i = 0; i < Math.round(seconds / DT); i++) steps.push(w.step());
      const sags = steps.map((s) => -s.sample.deviation_m[2]);
      return { steps, maxSag: max(sags), finalSag: sags[sags.length - 1]!, blockZ: freePos(w, "load")[2] };
    },
  );
}

describe("cargas verticales sobre el obstáculo (MuJoCo WASM real)", () => {
  it("un peso de 3 veces el tope vertical lo hunde y lo registra como saturado", async () => {
    const r = await loadRun(3 * CAP_V, 0.5);
    const last = r.steps[r.steps.length - 1]!;
    console.log(`carga 3×tope (${(3 * CAP_V).toFixed(0)} N): hundimiento ${(100 * r.maxSag).toFixed(1)} cm en 0,5 s, orden vertical final ${last.sample.force_n[2].toFixed(1)} N (tope ${CAP_V.toFixed(1)})`);
    expect(r.maxSag).toBeGreaterThan(0.3);
    expect(last.sample.vertical_saturated).toBe(true);
    expect(max(r.steps.map((s) => Math.abs(s.sample.force_n[2])))).toBeLessThanOrEqual(CAP_V * (1 + 1e-9));
    expect(r.finalSag).toBeGreaterThan(r.steps[Math.round(0.25 / DT)]!.sample.deviation_m[2] * -1); // sigue hundiéndose
  });

  it("con 0,5 veces el tope, la hundimiento estático coincide con peso/kp (±5 %)", async () => {
    const r = await loadRun(0.5 * CAP_V, 2);
    const expected = (0.5 * CAP_V) / 20000;
    console.log(`carga 0,5×tope (${(0.5 * CAP_V).toFixed(0)} N): hundimiento máx ${mm(r.maxSag)}, final ${mm(r.finalSag)}, peso/kp = ${mm(expected)}`);
    expect(Math.abs(r.finalSag - expected) / expected).toBeLessThan(0.05);
    expect(r.steps.some((s) => s.sample.saturated)).toBe(false);
  });

  // CRITERIO DE LA ISSUE NO CUMPLIDO con kp = 20000 N/m: un muelle puro cede F/kp = 0,5 · 1471,5 N / 20000 = 3,68 cm, no < 1 cm.
  // `it.fails` pasa mientras la afirmación siga fallando; si alguien cambia kp o añade un término integral y el criterio se
  // cumple, esta prueba pasará a rojo y avisará de que hay que convertirla en `it`.
  it.fails("[criterio de la issue NO cumplido: medido 3,7 cm] con 0,5 veces el tope, el obstáculo se mueve menos de 1 cm", async () => {
    const r = await loadRun(0.5 * CAP_V, 2);
    expect(r.maxSag).toBeLessThan(0.01);
  });

  it("la rigidez que sí cumpliría el criterio es kp ≥ peso/1 cm = 73 575 N/m (con kp = 100 000 N/m, 0,5 veces el tope se hunde < 1 cm)", async () => {
    const mass = (0.5 * CAP_V) / GRAVITY_MPS2;
    const sag = await withWorld(
      barConfig({ kp_n_per_m: 100000 }), blockXml("load", [0, 0, 1.2 + 0.1 + 0.0005], [0.15, 0.15, 0.1], mass),
      (w) => {
        const sags: number[] = [];
        for (let i = 0; i < 1000; i++) sags.push(-w.step().sample.deviation_m[2]);
        return max(sags);
      },
    );
    console.log(`con kp = 100000 N/m: hundimiento máx ${mm(sag)}`);
    expect(sag).toBeLessThan(0.01);
  });

  it("la reacción de Tori acumulada en reposo con carga es −(peso del obstáculo + de la carga)·t", async () => {
    const load = 0.5 * CAP_V;
    const r = await withWorld(barConfig(), blockXml("load", [0, 0, 1.2 + 0.1 + 0.0005], [0.15, 0.15, 0.1], load / GRAVITY_MPS2), (w) => {
      const reactionBefore = Array.from({ length: 0 });
      void reactionBefore;
      for (let i = 0; i < 1500; i++) w.step();
      const s1 = w.state().totals.impulse_ns[2];
      for (let i = 0; i < 500; i++) w.step(); // 1 s en reposo
      const s2 = w.state().totals.impulse_ns[2];
      return { rate: s2 - s1, reaction: w.state().totals.reaction_impulse_ns[2] + w.state().totals.impulse_ns[2] };
    });
    expect(r.rate).toBeCloseTo(37.5 * GRAVITY_MPS2 + load, -1); // N·s en 1 s = N (±5 N)
    expect(r.reaction).toBe(0);
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// Fuerza de contacto sobre el otro cuerpo, saturación y cesión
// ---------------------------------------------------------------------------------------------------------------------

/**
 * Trineo de 100 kg (una sola deslizadera en X, sin rozamiento con nada) que llega a la cara del obstáculo con la velocidad
 * `v0` (m/s); opcionalmente lo empuja una fuerza constante `pushN` (N) en +X. Sirve de «Uke» de un solo grado de libertad:
 * sin suelo ni gravedad de por medio, toda la fuerza horizontal pasa por el contacto.
 */
async function impact(v0: number, pushN: number, seconds: number) {
  const sled = `<body name="blk" pos="${-0.05 - 0.15 - 0.01} 0 1"><joint name="blk_free" type="slide" axis="1 0 0" damping="0" armature="0"/>
    <geom name="blk_geom" type="box" size="0.15 0.15 0.15" mass="100" friction="1 0.005 0.0001" ${CONTACT}/></body>`;
  return withWorld(barConfig(), sled, (w) => {
    const model = w.model as { jnt(n: string): { id: number }; jnt_dofadr: Int32Array; jnt_qposadr: Int32Array; body(n: string): { id: number } };
    const id = model.jnt("blk_free").id;
    w.data.qvel[model.jnt_dofadr[id]!] = v0;
    const bodyId = model.body("blk").id;
    const steps: Step[] = [];
    for (let i = 0; i < Math.round(seconds / DT); i++) {
      steps.push(w.step((d) => { d.xfrc_applied[6 * bodyId] = pushN; }));
    }
    return { steps, blockX: w.data.qpos[model.jnt_qposadr[id]!]! - 0.21, obstaclePos: posOf(w) };
  });
}

/** Bloque libre de 100 kg lanzado en diagonal contra el obstáculo (para la simetría en Y), desplazado `y0` en Y. */
async function obliqueImpact(sign: 1 | -1, seconds: number) {
  const y0 = sign * 0.05;
  return withWorld(
    barConfig(), blockXml("blk", [-0.05 - 0.15 - 0.01, y0, 1], [0.15, 0.15, 0.15], 100),
    (w) => {
      const dof = freeDof(w, "blk");
      [2, sign * 0.8, 0].forEach((v, i) => { w.data.qvel[dof + i] = v; });
      const steps: Step[] = [];
      for (let i = 0; i < Math.round(seconds / DT); i++) steps.push(w.step());
      return { steps, blockPos: freePos(w, "blk"), obstaclePos: posOf(w) };
    },
  );
}

/** Relación máxima entre la fuerza de contacto y el límite `tope + m·|a − g|` de cada eje, en todos los pasos. */
function worstContactRatio(steps: Step[], params: ObstacleParams): { ratio: number; identityErr: number } {
  const m = params.mass_kg;
  let ratio = 0;
  let identityErr = 0;
  for (const s of steps) {
    const a: Vec3 = [(s.vAfter[0] - s.vBefore[0]) / DT, (s.vAfter[1] - s.vBefore[1]) / DT, (s.vAfter[2] - s.vBefore[2]) / DT];
    const on = s.contactOnOther; // sobre Uke (el otro cuerpo); sobre el obstáculo es la opuesta
    const properH = Math.hypot(a[0], a[1]);
    const boundH = params.max_horizontal_force_n + m * properH;
    ratio = Math.max(ratio, Math.hypot(on[0], on[1]) / boundH);
    const boundV = params.max_vertical_force_n + m * Math.abs(a[2] + GRAVITY_MPS2);
    ratio = Math.max(ratio, Math.abs(on[2]) / boundV);
    // Identidad de Newton del obstáculo: m·a = F_orden + F_contacto_sobre_obstáculo + m·g_vec.
    const resid: Vec3 = [
      m * a[0] - s.sample.force_n[0] + on[0], m * a[1] - s.sample.force_n[1] + on[1], m * (a[2] + GRAVITY_MPS2) - s.sample.force_n[2] + on[2],
    ];
    identityErr = Math.max(identityErr, norm(resid));
  }
  return { ratio, identityErr };
}

describe("fuerza de contacto sobre el otro cuerpo (MuJoCo WASM real)", () => {
  it("nunca supera el tope más el término inercial m·|a − g| (±10 %): choque de un bloque de 100 kg a 2 m/s", async () => {
    const r = await impact(2, 0, 0.6);
    const params = resolveObstacleParams(barConfig());
    const w = worstContactRatio(r.steps, params);
    const peak = max(r.steps.map((s) => norm(s.contactOnOther)));
    console.log(`choque 2 m/s: pico de contacto ${peak.toFixed(0)} N; relación máx fuerza/(tope + m·|a − g|) = ${w.ratio.toFixed(3)}; residuo máx de la identidad de Newton ${w.identityErr.toFixed(1)} N; saturó: ${r.steps.some((s) => s.sample.saturated)}`);
    expect(peak).toBeGreaterThan(300); // hubo choque de verdad
    expect(w.ratio).toBeLessThanOrEqual(1.1);
    expect(max(r.steps.map((s) => Math.hypot(s.sample.force_n[0], s.sample.force_n[1])))).toBeLessThanOrEqual(CAP_H * (1 + 1e-9));
  });

  it("la fuerza medida en los contactos cumple la segunda ley del obstáculo (valida el adaptador de contactos y su signo)", async () => {
    const r = await impact(2, 0, 0.6);
    const w = worstContactRatio(r.steps, resolveObstacleParams(barConfig()));
    const peak = max(r.steps.map((s) => norm(s.contactOnOther)));
    expect(w.identityErr).toBeLessThan(0.02 * peak);
    // El bloque empuja al obstáculo hacia +X, luego el obstáculo empuja al bloque hacia −X.
    const worst = r.steps.reduce((a, s) => (norm(s.contactOnOther) > norm(a.contactOnOther) ? s : a));
    expect(worst.contactOnOther[0]).toBeLessThan(0);
  });

  it("un obstáculo saturado cede y lo registra: bandera de saturación, desviación creciente y sin pico de fuerza", async () => {
    // Empuje constante de 1,5 veces el tope horizontal sobre un bloque de 100 kg apoyado en el obstáculo.
    const push: Vec3 = [1.5 * CAP_H, 0, 0];
    const r = await impact(0, push[0], 0.6);
    const dev = r.steps.map((s) => s.sample.deviation_m[0]);
    const firstSat = r.steps.findIndex((s) => s.sample.saturated);
    const params = resolveObstacleParams(barConfig());
    const after = r.steps.slice(firstSat);
    console.log(`saturado: empuje ${push[0].toFixed(0)} N (tope ${CAP_H.toFixed(0)}); satura en t = ${(firstSat * DT).toFixed(3)} s con desviación ${mm(dev[firstSat]!)}; desviación final ${(100 * dev[dev.length - 1]!).toFixed(1)} cm; contacto máx ${max(after.map((s) => norm(s.contactOnOther))).toFixed(0)} N; orden horizontal máx ${max(r.steps.map((s) => Math.hypot(s.sample.force_n[0], s.sample.force_n[1]))).toFixed(1)} N`);
    expect(firstSat).toBeGreaterThanOrEqual(0);
    // Satura cuando la demanda (kp·e + c·ė, con el amortiguador) alcanza el tope, no antes: la del paso anterior estaba por debajo.
    expect(norm(r.steps[firstSat]!.sample.demand_force_n)).toBeGreaterThan(CAP_H);
    expect(Math.hypot(r.steps[firstSat - 1]!.sample.demand_force_n[0], r.steps[firstSat - 1]!.sample.demand_force_n[1])).toBeLessThanOrEqual(CAP_H);
    expect(r.steps.slice(firstSat).every((s) => s.sample.saturated)).toBe(true);
    for (let i = firstSat + 50; i < dev.length; i += 50) expect(dev[i]!).toBeGreaterThan(dev[i - 50]!); // cede: la desviación crece
    expect(dev[dev.length - 1]!).toBeGreaterThan(0.1);
    expect(r.blockX).toBeGreaterThan(0.1); // el bloque avanza con él: cede de verdad
    expect(max(r.steps.map((s) => Math.hypot(s.sample.force_n[0], s.sample.force_n[1])))).toBeLessThanOrEqual(CAP_H * (1 + 1e-9));
    // Sin pico: la fuerza sobre el bloque se queda en el empuje (el obstáculo no «rebota»): ≤ 1,1·(tope + m·a) en cada paso.
    expect(worstContactRatio(r.steps, params).ratio).toBeLessThanOrEqual(1.1);
    // En régimen (últimos 0,2 s) la fuerza sobre el bloque es la del empuje menos m_bloque·a: entre el tope y el empuje.
    const steady = r.steps.slice(-100).map((s) => -s.contactOnOther[0]);
    expect(Math.min(...steady)).toBeGreaterThanOrEqual(CAP_H * 0.99);
    expect(Math.max(...steady)).toBeLessThanOrEqual(push[0] * 1.01);
    expect(r.steps.some((s) => s.sample.saturated)).toBe(true);
    expect(settleObstacleWork(createObstacleState(), [0, 0, 0])).toBeDefined();
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// Segado y barrido: límite de velocidad
// ---------------------------------------------------------------------------------------------------------------------

describe("segado y barrido: post con tope de velocidad (MuJoCo WASM real)", () => {
  const sweep = (extra: Partial<ObstacleConfig>): ObstacleConfig => ({
    name: "reap", shape: "post", start_m: [0, 0, 0.6], mass_kg: 8, max_force_n: 400, max_vertical_force_n: 400,
    phases: [{ start_s: 0.1, duration_s: 0.25, offset_m: [0.8, 0, 0] }], ...extra,
  });

  it("no supera max_speed_mps, y sin el límite la misma trayectoria sí lo supera", async () => {
    const planned = plannedPeakSpeedMps(sweep({}).phases!);
    const run = (extra: Partial<ObstacleConfig>) => withWorld(sweep(extra), "", (w) => {
      const steps: Step[] = [];
      for (let i = 0; i < 400; i++) steps.push(w.step());
      return {
        vMax: max(steps.map((s) => s.sample.speed_mps)),
        vMaxAfter: max(steps.map((s) => norm(s.vAfter))),
        limited: steps.filter((s) => s.sample.speed_limited).length,
        fMax: max(steps.map((s) => Math.hypot(s.sample.force_n[0], s.sample.force_n[1]))),
        dev: max(steps.map((s) => s.sample.deviation_magnitude_m)),
      };
    });
    const free = await run({});
    const limited = await run({ max_speed_mps: 2 });
    console.log(`barrido: velocidad planificada ${planned.toFixed(2)} m/s; sin límite ${free.vMaxAfter.toFixed(3)} m/s (fuerza máx ${free.fMax.toFixed(0)} N); con límite 2 m/s: ${limited.vMaxAfter.toFixed(4)} m/s (${limited.limited} pasos limitados, desviación máx ${mm(limited.dev)}, fuerza máx ${limited.fMax.toFixed(0)} N)`);
    expect(planned).toBeGreaterThan(5);
    expect(free.vMaxAfter).toBeGreaterThan(3);
    expect(limited.vMaxAfter).toBeLessThanOrEqual(2 * (1 + 1e-3));
    expect(limited.limited).toBeGreaterThan(10);
    expect(limited.fMax).toBeLessThanOrEqual(400 * (1 + 1e-9));
  });

  it("respeta su propio tope de fuerza (max_force_n) aunque la trayectoria la exija mayor", async () => {
    const r = await withWorld(sweep({ max_force_n: 100, phases: [{ start_s: 0.1, duration_s: 0.1, offset_m: [0.8, 0, 0] }] }), "", (w) => {
      const steps: Step[] = [];
      for (let i = 0; i < 300; i++) steps.push(w.step());
      return { fMax: max(steps.map((s) => Math.hypot(s.sample.force_n[0], s.sample.force_n[1]))), sat: steps.some((s) => s.sample.horizontal_saturated), dev: max(steps.map((s) => s.sample.deviation_magnitude_m)) };
    });
    expect(r.fMax).toBeLessThanOrEqual(100 * (1 + 1e-9));
    expect(r.sat).toBe(true);
    expect(r.dev).toBeGreaterThan(0.05); // no llega a su objetivo: cede la trayectoria, no la física
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// Rozamiento contra Uke (priority) y simetría en Y
// ---------------------------------------------------------------------------------------------------------------------

describe("rozamiento del obstáculo (MuJoCo WASM real)", () => {
  /** Bloque de 10 kg (rozamiento 1,0) sobre la almohadilla, empujado en X con `ratio` veces su peso. */
  async function slides(config: Partial<ObstacleConfig>, ratio: number): Promise<number> {
    return withWorld(barConfig(config), blockXml("blk", [0, 0, 1.2 + 0.05 + 0.0005], [0.1, 0.1, 0.05], 10, 1), (w) => {
      for (let i = 0; i < 500; i++) {
        w.step((d) => {
          const id = (w.model as { body(n: string): { id: number } }).body("blk").id;
          d.xfrc_applied[6 * id] = ratio * 10 * GRAVITY_MPS2;
        });
      }
      return freePos(w, "blk")[0];
    });
  }

  it("el rozamiento del obstáculo manda sobre el del bloque: con cloth (0,8) agarra a 0,7·N y desliza a 0,9·N; con skin (0,5) desliza a 0,7·N", async () => {
    const cloth07 = await slides({ surface: "cloth" }, 0.7);
    const cloth09 = await slides({ surface: "cloth" }, 0.9);
    const skin07 = await slides({ surface: "skin" }, 0.7);
    const skin04 = await slides({ surface: "skin" }, 0.4);
    console.log(`rozamiento: desplazamiento del bloque en 1 s: cloth 0,7·N ${(100 * cloth07).toFixed(2)} cm; cloth 0,9·N ${(100 * cloth09).toFixed(1)} cm; skin 0,7·N ${(100 * skin07).toFixed(1)} cm; skin 0,4·N ${(100 * skin04).toFixed(2)} cm`);
    expect(Math.abs(cloth07)).toBeLessThan(0.02);
    expect(cloth09).toBeGreaterThan(0.5);
    expect(skin07).toBeGreaterThan(0.5);
    expect(Math.abs(skin04)).toBeLessThan(0.02);
  });
});

describe("simetría en Y (MuJoCo WASM real)", () => {
  it("un choque oblicuo y su reflejo en Y dan trayectorias reflejadas del obstáculo y del bloque", async () => {
    const run = (sign: 1 | -1) => obliqueImpact(sign, 0.5);
    const a = await run(1);
    const b = await run(-1);
    const dPad = Math.max(
      Math.abs(a.obstaclePos[0] - b.obstaclePos[0]), Math.abs(a.obstaclePos[1] + b.obstaclePos[1]), Math.abs(a.obstaclePos[2] - b.obstaclePos[2]),
    );
    const dBlk = Math.max(
      Math.abs(a.blockPos[0] - b.blockPos[0]), Math.abs(a.blockPos[1] + b.blockPos[1]), Math.abs(a.blockPos[2] - b.blockPos[2]),
    );
    const dForce = max(a.steps.map((s, i) => Math.abs(s.contactOnOther[0] - b.steps[i]!.contactOnOther[0]) + Math.abs(s.contactOnOther[1] + b.steps[i]!.contactOnOther[1])));
    console.log(`simetría Y: obstáculo y bloque se desplazan en Y ${(100 * a.obstaclePos[1]).toFixed(3)} y ${(100 * a.blockPos[1]).toFixed(3)} cm; asimetría máx obstáculo ${dPad.toExponential(2)} m, bloque ${dBlk.toExponential(2)} m, contacto ${dForce.toExponential(2)} N`);
    expect(Math.abs(a.obstaclePos[1])).toBeGreaterThan(0.001); // se movió de verdad en Y
    expect(dPad).toBeLessThan(1e-6);
    expect(dBlk).toBeLessThan(1e-6);
  });

  it("la trayectoria con desplazamiento lateral +d y −d da posiciones reflejadas (sin Uke)", async () => {
    const run = (sign: 1 | -1) => withWorld(barConfig({ shape: "bar", phases: [{ start_s: 0.1, duration_s: 0.4, offset_m: [0.1, sign * 0.12, 0.03] }] }), "", (w) => {
      for (let i = 0; i < 400; i++) w.step();
      return posOf(w);
    });
    const a = await run(1);
    const b = await run(-1);
    expect(Math.abs(a[0] - b[0])).toBeLessThan(1e-9);
    expect(Math.abs(a[1] + b[1])).toBeLessThan(1e-9);
    expect(Math.abs(a[2] - b[2])).toBeLessThan(1e-9);
  });
});

// ---------------------------------------------------------------------------------------------------------------------
// Uke `tense` apoyado contra una barra a la altura de la cadera
// ---------------------------------------------------------------------------------------------------------------------

const UKE_MASS_KG = 75;
const UKE_HEIGHT_M = 1.75;
const SETTLE_S = 0.5;

interface UkeBarResult {
  lateralComM: number;
  lateralPelvisM: number;
  lateralRelativeM: number;
  yawDeg: number;
  forwardComM: number;
  chestTiltDeg: number;
  maxContactN: number;
  contactRatio: number;
  maxHorizontalCmdN: number;
  barDeviationM: number;
  saturatedSteps: number;
  upright: boolean;
  frictionUse: number;
  rootForceMax: number;
}

/**
 * Montaje: Uke `tense` (75 kg, 1,75 m, postura neutra) de cara a +X, con la barra delante de la pelvis, a su altura y a
 * 1 mm del contacto. Tras 0,5 s de asentamiento sin empuje, una fuerza horizontal en +X sobre el sitio `sternum`
 * (efector declarado, aplicado con `applyForceAtPoint` sobre el cuerpo `chest`, nunca sobre la raíz) sube en 0,3 s hasta
 * `pushN` y se mantiene 2 s: la pelvis se apoya contra la barra, que cede o no según su tope. Se mide el desvío lateral
 * (Y) del centro de masas, el de la pelvis y el de la pelvis respecto a la barra.
 */
async function ukeAgainstBar(options: { pushN: number; lateralN?: number; barY?: number; seconds?: number; friction?: number }): Promise<UkeBarResult> {
  const mj = await getMujoco();
  const seconds = options.seconds ?? 2;
  const barY = options.barY ?? 0;
  const damping = attitudeJointDamping("tense", UKE_MASS_KG, UKE_HEIGHT_M, 1);
  const base = (): string => generateHumanoidMJCF({ mass_kg: UKE_MASS_KG, height_m: UKE_HEIGHT_M }, {
    friction: 1, optionAttrs: COLLISION_ATTRS, modelName: "uke_bar", jointDampingByName: damping,
  });
  // Sonda: dónde queda la pelvis tras colocar la postura (para situar la barra).
  let pelvisZ = 0, pelvisHalfDepth = 0;
  {
    const model = mj.MjModel.from_xml_string(base());
    const data = new mj.MjData(model);
    try {
      applyStance(mj, model, data, "neutral");
      mj.mj_forward(model, data);
      pelvisZ = data.xpos[3 + 2]!;
      pelvisHalfDepth = model.geom_size[3 * model.geom("pelvis_geom").id]!;
    } finally {
      data.delete();
      model.delete();
    }
  }
  const barParams = resolveObstacleParams({
    name: "hipbar", shape: "bar", tori_mass_kg: TORI_KG, start_m: [pelvisHalfDepth + 0.11 + 0.001, barY, pelvisZ],
    ...(options.friction !== undefined ? { friction: options.friction } : {}),
  });
  const model = mj.MjModel.from_xml_string(appendToWorldbody(base(), buildObstacleMjcf(barParams)));
  const data = new mj.MjData(model);
  try {
    expect(model.body(1).name).toBe("pelvis");
    setJointDamping(model as unknown as DampingModel, damping);
    const placement = applyStance(mj, model, data, "neutral");
    mj.mj_forward(model, data);
    const ukeParams: UkeControllerParams = {
      attitude: "tense", stance: "neutral", mass_kg: UKE_MASS_KG, height_m: UKE_HEIGHT_M, strength: 1, targets: placement.targets,
    };
    const uke = createUkeController(ukeParams, model as unknown as StandingModel, data as unknown as StandingState);
    const handles = resolveObstacleHandles(model as never, barParams);
    const pyramidal = Boolean(mj.mj_isPyramidal(model));
    const ukeBodies = new Set<number>();
    for (let b = 1; b < model.nbody; b++) if (b !== handles.bodyId) ukeBodies.add(b);
    const chest = model.body("chest").id;
    const sternum = model.site("sternum").id;
    const qfrc = data.qfrc_applied as Float64Array;
    const xfrc = data.xfrc_applied as Float64Array;
    let state = createObstacleState();
    let contact: Vec3 = [0, 0, 0];
    const steps: Step[] = [];
    const maxCmd: number[] = [];
    let rootForceMax = 0;
    let y0 = NaN, pelvisY0 = NaN, x0 = NaN;
    let lateralComM = 0, lateralPelvisM = 0, lateralRelativeM = 0, yawDeg = 0, forwardComM = 0, chestTilt = 0, frictionUse = 0;
    let yaw0 = NaN;
    const total = Math.round((SETTLE_S + seconds) / DT);
    const settleSteps = Math.round(SETTLE_S / DT);
    for (let i = 0; i < total; i++) {
      const t = (i - settleSteps) * DT;
      xfrc.fill(0);
      mj.mj_kinematics(model, data);
      mj.mj_comPos(model, data);
      mj.mj_comVel(model, data);
      if (i === settleSteps) {
        y0 = data.subtree_com[1]!; x0 = data.subtree_com[0]!; pelvisY0 = data.xpos[4]!;
        yaw0 = Math.atan2(data.xmat[9 + 3]!, data.xmat[9]!);
      }
      if (t >= 0 && (options.pushN !== 0 || (options.lateralN ?? 0) !== 0)) {
        const ramp = Math.min(1, t / 0.3);
        applyForceAtPoint(data as never, chest, [data.site_xpos[3 * sternum]!, data.site_xpos[3 * sternum + 1]!, data.site_xpos[3 * sternum + 2]!], [ramp * options.pushN, ramp * (options.lateralN ?? 0), 0]);
      }
      mj.mj_subtreeVel(model, data);
      uke.step(model as unknown as StandingModel, data as unknown as StandingState, t);
      for (let k = 0; k < 6; k++) rootForceMax = Math.max(rootForceMax, Math.abs(qfrc[k]!));
      const vBefore: Vec3 = [data.qvel[handles.dofAdr[0]]!, data.qvel[handles.dofAdr[1]]!, data.qvel[handles.dofAdr[2]]!];
      const k = readObstacleKinematics(barParams, handles, data);
      const r = stepObstacle(barParams, state, { t, dt: DT, ...k, contact_force_on_uke_n: contact });
      state = r.state;
      applyObstacleCommand(handles, data, r); // después del controlador de Uke, que pone qfrc_applied a cero
      mj.mj_step(model, data);
      contact = readObstacleContactForce(model, handles, data as never, { pyramidal, isOtherBody: (b) => ukeBodies.has(b) }).force_on_other_n;
      if (t >= 0) {
        steps.push({ t, sample: r.sample, vBefore, vAfter: [data.qvel[handles.dofAdr[0]]!, data.qvel[handles.dofAdr[1]]!, data.qvel[handles.dofAdr[2]]!], contactOnOther: contact });
        maxCmd.push(Math.hypot(r.force_n[0], r.force_n[1]));
        lateralComM = Math.max(lateralComM, Math.abs(data.subtree_com[1]! - y0));
        lateralPelvisM = Math.max(lateralPelvisM, Math.abs(data.xpos[4]! - pelvisY0));
        lateralRelativeM = Math.max(lateralRelativeM, Math.abs((data.xpos[4]! - pelvisY0) - (r.sample.position_m[1] - barY)));
        yawDeg = Math.max(yawDeg, Math.abs(Math.atan2(data.xmat[9 + 3]!, data.xmat[9]!) - yaw0) * 180 / Math.PI);
        forwardComM = Math.max(forwardComM, data.subtree_com[0]! - x0);
        const ez = data.xmat[9 * chest + 8]!;
        chestTilt = Math.max(chestTilt, Math.acos(Math.min(1, ez)) * 180 / Math.PI);
        // Uso de rozamiento: |fuerza tangencial| / (µ·normal) aproximado con el contacto neto (solo informativo).
        const on = contact;
        if (Math.abs(on[0]) > 1) frictionUse = Math.max(frictionUse, Math.hypot(on[1], on[2]) / (0.8 * Math.abs(on[0])));
      }
    }
    const w = worstContactRatio(steps, barParams);
    const last = steps[steps.length - 1]!;
    void last;
    return {
      lateralComM, lateralPelvisM, lateralRelativeM, yawDeg, forwardComM, chestTiltDeg: chestTilt,
      maxContactN: max(steps.map((s) => norm(s.contactOnOther))), contactRatio: w.ratio, maxHorizontalCmdN: max(maxCmd),
      barDeviationM: max(steps.map((s) => s.sample.deviation_magnitude_m)), saturatedSteps: steps.filter((s) => s.sample.saturated).length,
      upright: chestTilt < 45, frictionUse, rootForceMax,
    };
  } finally {
    data.delete();
    model.delete();
  }
}

describe("Uke tense apoyado contra una barra a la altura de la cadera (MuJoCo WASM real)", () => {
  it("no resbala de lado: desvío lateral < 2 cm en 2 s (empuje de 80 N en el esternón)", async () => {
    const r = await ukeAgainstBar({ pushN: 80 });
    console.log(`Uke tense contra barra (80 N): desvío lateral máx del CM ${mm(r.lateralComM)}, de la pelvis ${mm(r.lateralPelvisM)}, de la pelvis respecto a la barra ${mm(r.lateralRelativeM)}; giro de pelvis ${r.yawDeg.toFixed(3)}°; avance del CM ${mm(r.forwardComM)}; inclinación máx del pecho ${r.chestTiltDeg.toFixed(1)}°; contacto máx ${r.maxContactN.toFixed(0)} N (relación con tope + m·a: ${r.contactRatio.toFixed(3)}); la barra se desvía ${mm(r.barDeviationM)}, saturó ${r.saturatedSteps} pasos; fuerza en la raíz ${r.rootForceMax}`);
    expect(r.rootForceMax).toBe(0);
    expect(r.upright).toBe(true);
    expect(r.maxContactN).toBeGreaterThan(20); // se apoya de verdad
    expect(r.lateralComM).toBeLessThan(0.02);
    expect(r.lateralPelvisM).toBeLessThan(0.02);
    expect(r.contactRatio).toBeLessThanOrEqual(1.1);
  }, 60000);

  it("robustez y simetría: con la barra desplazada +1 cm y −1 cm en Y el desvío sigue < 2 cm y es especular", async () => {
    const p = await ukeAgainstBar({ pushN: 80, barY: 0.01 });
    const n = await ukeAgainstBar({ pushN: 80, barY: -0.01 });
    console.log(`barra +1 cm: desvío lateral CM ${mm(p.lateralComM)}, pelvis ${mm(p.lateralPelvisM)}; barra −1 cm: CM ${mm(n.lateralComM)}, pelvis ${mm(n.lateralPelvisM)}`);
    expect(p.lateralComM).toBeLessThan(0.02);
    expect(n.lateralComM).toBeLessThan(0.02);
    expect(Math.abs(p.lateralComM - n.lateralComM)).toBeLessThan(0.005);
  }, 120000);

  it("con una componente lateral del 25 % del empuje (20 N en Y) tampoco resbala: desvío < 2 cm; con rozamiento 0,05 (control) la pelvis sí se desvía", async () => {
    const rough = await ukeAgainstBar({ pushN: 80, lateralN: 20 });
    const slick = await ukeAgainstBar({ pushN: 80, lateralN: 20, friction: 0.05 });
    console.log(`lateral 20 N: µ=0,8: CM ${mm(rough.lateralComM)}, pelvis ${mm(rough.lateralPelvisM)}, pelvis−barra ${mm(rough.lateralRelativeM)}; µ=0,05: CM ${mm(slick.lateralComM)}, pelvis ${mm(slick.lateralPelvisM)}, pelvis−barra ${mm(slick.lateralRelativeM)}`);
    expect(rough.lateralComM).toBeLessThan(0.02);
    expect(rough.lateralPelvisM).toBeLessThan(0.02);
    expect(slick.lateralRelativeM).toBeGreaterThan(1.5 * rough.lateralRelativeM); // control: el rozamiento sí importa
  }, 120000);

  it("sin empuje (Uke solo asentado junto a la barra) tampoco se desvía y la barra no se mueve", async () => {
    const r = await ukeAgainstBar({ pushN: 0, seconds: 1 });
    expect(r.lateralComM).toBeLessThan(0.005);
    expect(r.barDeviationM).toBeLessThan(0.002);
  }, 60000);
});
