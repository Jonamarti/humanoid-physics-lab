import { afterEach, describe, expect, it } from "vitest";
import { detectArtifacts } from "../../src/metrics/artifacts";
import type { Recording, RecordingDiagnostics } from "../../src/recording/schema";
import { ArtifactProbe, type ProbeContext } from "../../src/scenario/artifactProbe";
import { getMujoco } from "../../src/scenario/mujocoModule";

/**
 * `ArtifactProbe` con MuJoCo WASM real sobre modelos de juguete: por cada magnitud medida por paso, un caso que supera el umbral
 * y otro que no, y la evaluación con `detectArtifacts`. Los casos de `joint_speed`, `tremor`, `foot_slip` y `segment_penetration`
 * no se pueden provocar de forma fiable con los escenarios actuales, así que se fuerza el estado y se comprueba que la sonda
 * mide lo que debe (y que el detector lo clasifica).
 */
const FLOOR = `<geom name="floor" type="plane" size="5 5 0.1" friction="1 0.005 0.0001" solref="0.015 1" solimp="0.9 0.95 0.001"/>`;
const OPTION = `<option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="10"/>`;
const CONTACT = `solref="0.015 1" solimp="0.9 0.95 0.001"`;

const cleanups: Array<() => void> = [];
afterEach(() => {
  while (cleanups.length) cleanups.pop()!();
});

interface Rig {
  mj: Awaited<ReturnType<typeof getMujoco>>;
  model: any; // eslint-disable-line @typescript-eslint/no-explicit-any
  data: any; // eslint-disable-line @typescript-eslint/no-explicit-any
  probe: ArtifactProbe;
}

async function rig(body: string, ctx: Omit<ProbeContext, "dt" | "bodies" | "joints">, names: { bodies: string[]; joints: string[] }): Promise<Rig> {
  const mj = await getMujoco();
  const model = mj.MjModel.from_xml_string(`<mujoco>${OPTION}<worldbody>${FLOOR}${body}</worldbody></mujoco>`);
  const data = new mj.MjData(model);
  cleanups.push(() => { data.delete(); model.delete(); });
  mj.mj_forward(model, data);
  const probe = new ArtifactProbe(mj, model, data, { ...ctx, dt: 0.002, bodies: ["world", ...names.bodies], joints: names.joints });
  return { mj, model, data, probe };
}

/** Cuerpo libre con una caja (semi-lados `h`) en `pos`, de masa `mass`. */
const freeBox = (name: string, pos: string, h: string, mass: number): string =>
  `<body name="${name}" pos="${pos}"><freejoint name="${name}_free"/><geom type="box" size="${h}" mass="${mass}" ${CONTACT}/></body>`;

/** Una medida (paso de contactos) sobre el estado ya calculado con `mj_forward`. */
function measureContacts(r: Rig, steps = 1): void {
  for (let k = 0; k < steps; k++) {
    r.probe.beforeStep();
    r.probe.afterStep(k * 0.002);
  }
}

/** Evalúa unos diagnósticos con el detector (grabación sintética mínima de 2 s). */
function evaluate(d: RecordingDiagnostics) {
  const rec = {
    schema: "hpl.recording.v1",
    meta: {} as never, model: {} as never, peaks: { effectors: {} },
    samples: [0, 1, 2].map((t) => ({ t, qpos: [], qvel: [], effectors: [], contacts: [] })),
    summary: {} as never,
    diagnostics: d,
  } as Recording;
  return detectArtifacts(rec).artifacts.map((a) => a.type);
}

const HUMANOID = { actorBodies: [] as string[], humanoid: true };

describe("ArtifactProbe: penetraciones y fuerza de contacto (contactos reales de MuJoCo)", () => {
  it("penetración en el suelo: 2 cm dispara (ground_penetration) y 0,5 mm no", async () => {
    const deep = await rig(freeBox("chest", "0 0 0.03", "0.1 0.1 0.05", 10), { ...HUMANOID, actorBodies: ["chest"] }, { bodies: ["chest"], joints: ["chest_free"] });
    measureContacts(deep);
    const dd = deep.probe.diagnostics();
    expect(dd.ground_penetration.value).toBeCloseTo(0.02, 3);
    expect(dd.ground_penetration.where).toBe("chest");
    expect(evaluate(dd)).toContain("ground_penetration");
    const shallow = await rig(freeBox("chest", "0 0 0.0495", "0.1 0.1 0.05", 10), { ...HUMANOID, actorBodies: ["chest"] }, { bodies: ["chest"], joints: ["chest_free"] });
    measureContacts(shallow);
    const sd = shallow.probe.diagnostics();
    expect(sd.ground_penetration.value).toBeCloseTo(0.0005, 4);
    expect(evaluate(sd)).not.toContain("ground_penetration");
  });

  it("penetración entre segmentos: 3 cm dispara (segment_penetration) y 1 cm no; los props y el suelo no cuentan", async () => {
    const two = (x: number): string =>
      freeBox("left_thigh", "0 0 1", "0.1 0.1 0.1", 5) + freeBox("right_thigh", `${x} 0 1`, "0.1 0.1 0.1", 5);
    const names = { bodies: ["left_thigh", "right_thigh"], joints: ["left_thigh_free", "right_thigh_free"] };
    const ctx = { ...HUMANOID, actorBodies: ["left_thigh", "right_thigh"] };
    const deep = await rig(two(0.17), ctx, names);
    measureContacts(deep);
    expect(deep.probe.diagnostics().segment_penetration!.value).toBeCloseTo(0.03, 3);
    expect(deep.probe.diagnostics().segment_penetration!.where).toBe("left_thigh|right_thigh");
    expect(evaluate(deep.probe.diagnostics())).toContain("segment_penetration");
    const shallow = await rig(two(0.19), ctx, names);
    measureContacts(shallow);
    expect(shallow.probe.diagnostics().segment_penetration!.value).toBeCloseTo(0.01, 3);
    expect(evaluate(shallow.probe.diagnostics())).not.toContain("segment_penetration");
    // El mismo solape con un cuerpo que NO es del actor no es un artefacto del actor.
    const prop = await rig(two(0.17), { ...HUMANOID, actorBodies: ["left_thigh"] }, names);
    measureContacts(prop);
    expect(prop.probe.diagnostics().segment_penetration!.value).toBe(0);
  });

  it("fuerza de contacto: una caja de 2000 kg apoyada 1 cm hundida dispara y una de 75 kg apoyada 1 mm no", async () => {
    const heavy = await rig(freeBox("chest", "0 0 0.04", "0.2 0.2 0.05", 2000), { ...HUMANOID, actorBodies: ["chest"] }, { bodies: ["chest"], joints: ["chest_free"] });
    measureContacts(heavy);
    const hd = heavy.probe.diagnostics();
    expect(hd.contact_force.value).toBeGreaterThan(10_000);
    expect(hd.contact_force.where).toBe("chest|world");
    expect(evaluate(hd)).toContain("contact_force");
    const light = await rig(freeBox("chest", "0 0 0.049", "0.2 0.2 0.05", 75), { ...HUMANOID, actorBodies: ["chest"] }, { bodies: ["chest"], joints: ["chest_free"] });
    measureContacts(light);
    expect(light.probe.diagnostics().contact_force.value).toBeGreaterThan(100);
    expect(light.probe.diagnostics().contact_force.value).toBeLessThan(10_000);
    expect(evaluate(light.probe.diagnostics())).not.toContain("contact_force");
  });
});

describe("ArtifactProbe: articulaciones (velocidad y temblor)", () => {
  const ARM = `<body name="pelvis" pos="0 0 2"><freejoint name="pelvis_free"/><geom type="sphere" size="0.1" mass="10"/>
    <body name="left_shin" pos="0 0 -0.1"><joint name="left_knee" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04" mass="3"/></body></body>`;
  const ctx = { ...HUMANOID, actorBodies: ["pelvis", "left_shin"] };
  const names = { bodies: ["pelvis", "left_shin"], joints: ["pelvis_free", "left_knee"] };

  it("velocidad articular: 30 rad/s dispara (joint_speed) y 24 rad/s no; la raíz libre no cuenta", async () => {
    const hit = await rig(ARM, ctx, names);
    (hit.data.qvel as Float64Array)[6] = 30;
    (hit.data.qvel as Float64Array)[0] = 99; // la raíz libre no es una articulación interna
    hit.probe.state(0.25);
    const d = hit.probe.diagnostics();
    expect(d.joint_speed).toMatchObject({ value: 30, t_s: 0.25, where: "left_knee" });
    expect(evaluate(d)).toContain("joint_speed");
    const ok = await rig(ARM, ctx, names);
    (ok.data.qvel as Float64Array)[6] = -24;
    ok.probe.state(0.25);
    expect(ok.probe.diagnostics().joint_speed!.value).toBe(24);
    expect(evaluate(ok.probe.diagnostics())).not.toContain("joint_speed");
  });

  /** Mueve la rodilla `q = amp·sin(2π f t)` durante 2 s a 500 Hz y devuelve lo medido. */
  async function wobble(f: number, amp: number): Promise<RecordingDiagnostics> {
    const r = await rig(ARM, ctx, names);
    for (let k = 0; k <= 1000; k++) {
      (r.data.qpos as Float64Array)[7] = amp * Math.sin(2 * Math.PI * f * k * 0.002);
      r.probe.state(k * 0.002);
    }
    return r.probe.diagnostics();
  }

  it("temblor: 8 Hz (16 cambios de sentido por segundo) dispara; 4 Hz (8) y 1 Hz no", async () => {
    const fast = await wobble(8, 0.05);
    expect(fast.tremor!.value).toBeGreaterThanOrEqual(15);
    expect(fast.tremor!.value).toBeLessThanOrEqual(17);
    expect(fast.tremor!.where).toBe("left_knee");
    expect(evaluate(fast)).toContain("tremor");
    const mid = await wobble(4, 0.05);
    expect(mid.tremor!.value).toBeGreaterThanOrEqual(7);
    expect(mid.tremor!.value).toBeLessThanOrEqual(9);
    expect(evaluate(mid)).not.toContain("tremor");
    expect((await wobble(1, 0.05)).tremor!.value).toBeLessThanOrEqual(2);
  });

  it("temblor: una oscilación más pequeña que la amplitud mínima (0,002 rad a 20 Hz) no cuenta; es una limitación documentada", async () => {
    const tiny = await wobble(20, 0.002);
    expect(tiny.tremor!.value).toBe(0);
    expect(evaluate(tiny)).not.toContain("tremor");
  });
});

describe("ArtifactProbe: pie que patina", () => {
  const FEET = (pelvisMass: number): string => freeBox("left_foot", "0 0 0.049", "0.15 0.05 0.05", 75) + freeBox("pelvis", "2 0 3", "0.1 0.1 0.1", pelvisMass);
  const names = { bodies: ["left_foot", "pelvis"], joints: ["left_foot_free", "pelvis_free"] };
  const ctx = { ...HUMANOID, actorBodies: ["left_foot", "pelvis"] };

  /** Estado con el pie en reposo sobre el suelo (rozamiento sin saturar, o saturado con una fuerza lateral), y velocidad impuesta. */
  async function creeping(speed: number, opts: { pelvisMass?: number; lateralN?: number } = {}): Promise<RecordingDiagnostics> {
    const r = await rig(FEET(opts.pelvisMass ?? 1), ctx, names);
    const xfrc = r.data.xfrc_applied as Float64Array;
    const foot = 1; // índice del cuerpo (0 = world)
    xfrc[6 * foot] = opts.lateralN ?? 0;
    r.mj.mj_forward(r.model, r.data);
    for (let k = 0; k < 500; k++) {
      (r.data.qvel as Float64Array)[0] = speed; // el pie «se arrastra» a esa velocidad con el contacto activo
      r.mj.mj_comVel(r.model, r.data);
      r.probe.beforeStep();
      r.probe.afterStep(k * 0.002);
    }
    return r.probe.diagnostics();
  }

  it("5 cm de arrastre en 1 s con el pie cargado y el rozamiento sin saturar dispara (foot_slip); 5 mm no", async () => {
    const hit = await creeping(0.05);
    expect(hit.foot_slip!.value).toBeCloseTo(0.05, 2);
    expect(hit.foot_slip!.where).toBe("left_foot");
    expect(evaluate(hit)).toContain("foot_slip");
    const ok = await creeping(0.005);
    expect(ok.foot_slip!.value).toBeCloseTo(0.005, 3);
    expect(evaluate(ok)).not.toContain("foot_slip");
  });

  it("si el escenario lo provoca (fuerza lateral de 1,5 veces el peso: rozamiento saturado) el mismo arrastre no es un artefacto", async () => {
    const provoked = await creeping(0.05, { lateralN: 1.5 * 75 * 9.81 });
    expect(provoked.foot_slip!.value).toBe(0);
    expect(evaluate(provoked)).not.toContain("foot_slip");
  });

  it("un pie poco cargado (< 20 % del peso del actor) no se evalúa como apoyado", async () => {
    const light = await creeping(0.05, { pelvisMass: 600 });
    expect(light.foot_slip!.value).toBe(0);
  });
});

describe("ArtifactProbe: balance de energía (pasos reales de mj_step)", () => {
  const BALL = `<body name="pelvis" pos="0 0 2"><freejoint name="pelvis_free"/><geom type="sphere" size="0.1" mass="10" contype="0" conaffinity="0"/>
    <body name="left_shin" pos="0 0 0"><joint name="left_knee" type="hinge" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="2" contype="0" conaffinity="0"/></body></body>`;
  const names = { bodies: ["pelvis", "left_shin"], joints: ["pelvis_free", "left_knee"] };
  const ctx = { ...HUMANOID, actorBodies: ["pelvis", "left_shin"] };

  /** Avanza `steps` pasos con una fuerza vertical en la pelvis y un par en la rodilla; muestrea la energía cada 10 pasos. */
  async function run(opts: { forceN?: number; torqueNm?: number; hiddenKick?: boolean; declareForceWork?: boolean }): Promise<{ d: RecordingDiagnostics; deltaE: number; independentWork: number }> {
    const r = await rig(BALL, ctx, names);
    const { mj, model, data, probe } = r;
    const energy = (): number => { mj.mj_energyPos(model, data); mj.mj_energyVel(model, data); return (data.energy as Float64Array)[0]! + (data.energy as Float64Array)[1]!; };
    let forceWork = 0;
    let torqueWork = 0;
    let workAtLastSample = 0;
    let e0 = NaN;
    let eLast = NaN;
    for (let k = 0; k < 150; k++) {
      mj.mj_forward(model, data);
      if (k % 10 === 0) {
        probe.sample(k * 0.002, opts.declareForceWork ? forceWork : 0);
        eLast = energy();
        workAtLastSample = forceWork + torqueWork; // el trabajo hasta este mismo instante
        if (Number.isNaN(e0)) e0 = eLast;
      }
      if (opts.hiddenKick && k === 100) (data.qvel as Float64Array)[2] -= 3; // sube la energía cinética (unos 120 J) y nadie la declara
      (data.xfrc_applied as Float64Array)[6] = 0;
      (data.xfrc_applied as Float64Array)[8] = opts.forceN ?? 0;
      (data.qfrc_applied as Float64Array)[6] = opts.torqueNm ?? 0;
      probe.beforeStep();
      const vz0 = (data.qvel as Float64Array)[2]!;
      const w0 = (data.qvel as Float64Array)[6]!;
      mj.mj_step(model, data);
      probe.afterStep(k * 0.002);
      torqueWork += (opts.torqueNm ?? 0) * 0.5 * (w0 + (data.qvel as Float64Array)[6]!) * 0.002;
      forceWork += (opts.forceN ?? 0) * 0.5 * (vz0 + (data.qvel as Float64Array)[2]!) * 0.002;
    }
    return { d: probe.diagnostics(), deltaE: eLast - e0, independentWork: workAtLastSample };
  }

  it("caída libre: la energía se conserva, el exceso es ~0 y no dispara", async () => {
    const { d } = await run({});
    expect(d.energy!.scale_j).toBeCloseTo(9.81 * (10 * 2 + 2 * 2), 0);
    expect(d.energy!.max_excess_j).toBeLessThan(0.01);
    expect(evaluate(d)).not.toContain("energy_gain");
  });

  it("un efector declarado que empuja hacia arriba (200 N) aporta energía y el trabajo declarado la explica: no dispara", async () => {
    const { d, deltaE, independentWork } = await run({ forceN: 200, declareForceWork: true });
    expect(deltaE).toBeGreaterThan(20); // sube de verdad: hay energía que explicar
    expect(Math.abs(deltaE - independentWork)).toBeLessThan(0.02 * deltaE); // y el trabajo calculado aparte la explica
    expect(d.energy!.max_excess_j).toBeLessThan(0.01 * d.energy!.scale_j);
    expect(evaluate(d)).not.toContain("energy_gain");
  });

  it("la misma fuerza SIN declarar su trabajo (fuente oculta) dispara (energy_gain)", async () => {
    const { d } = await run({ forceN: 200, declareForceWork: false });
    expect(d.energy!.max_excess_j).toBeGreaterThan(20);
    expect(evaluate(d)).toContain("energy_gain");
  });

  it("un par interno en la rodilla aporta energía y su trabajo (signo incluido) la explica: no dispara", async () => {
    const { d, deltaE, independentWork } = await run({ torqueNm: 3 });
    expect(deltaE).toBeGreaterThan(5); // el par sube la energía respecto a la caída libre
    // Residuo del integrador (medido: ~4 % del trabajo con un par de 3 N·m a 2 ms; el efector sale mucho mejor): el trabajo
    // por trapecio no es exacto con una matriz de inercia que cambia. Por eso la tolerancia es una fracción de m·g·z y no 0.
    expect(Math.abs(deltaE - independentWork)).toBeLessThan(0.1 * deltaE);
    expect(d.energy!.max_excess_j).toBeLessThan(0.01 * d.energy!.scale_j);
    expect(evaluate(d)).not.toContain("energy_gain");
  });

  it("un salto de velocidad hacia abajo que nadie declara dispara", async () => {
    const { d } = await run({ hiddenKick: true });
    expect(d.energy!.max_excess_j).toBeGreaterThan(30);
    expect(evaluate(d)).toContain("energy_gain");
  });

  it("fuerza sobre la raíz: un par escrito en los 6 primeros grados queda registrado", async () => {
    const r = await rig(BALL, ctx, names);
    (r.data.qfrc_applied as Float64Array)[3] = 0.7;
    r.probe.beforeStep();
    expect(r.probe.diagnostics().root_generalized_force).toBe(0.7);
    expect(evaluate(r.probe.diagnostics())).toContain("root_force");
  });
});
