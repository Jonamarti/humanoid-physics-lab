import { describe, expect, it } from "vitest";
import {
  HAND_DEFAULTS, applyForceAtPoint, createHandState, handTargetOffset, minimumJerk, resolveHandParams, settleHandWork,
  stepHand, type HandConfig, type HandParams, type HandSample, type HandState, type Vec3,
} from "../../src/effectors/hand";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { PHYSICS_TIMESTEP_S } from "../../src/scenario/worldOptions";

const DT = PHYSICS_TIMESTEP_S;
const ORIGIN: Vec3 = [0, 0, 0];

const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a: Vec3): number => Math.sqrt(dot(a, a));
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mirrorY = (v: Vec3): Vec3 => [v[0], -v[1], v[2]];

/** Mano con hombro en el origen y alcance enorme (para no mezclar el recorte de alcance en las demás pruebas). */
function config(extra: Partial<HandConfig> = {}): HandConfig {
  return { link: "rope", shoulder_m: ORIGIN, reach_m: 100, ...extra };
}

/** Un paso con el agarre parado en `grip` (sin motor). */
function pureStep(params: HandParams, state: HandState, t: number, grip: Vec3, velocity: Vec3 = ORIGIN) {
  return stepHand(params, state, { t, dt: DT, grip_m: grip, grip_velocity_mps: velocity });
}

// ---------------------------------------------------------------------------------------------------------------------
// Cuerpo de prueba en MuJoCo WASM real: masa `mass` con tres deslizaderas (X, Y, Z), sin gravedad; el agarre es su origen.
// ---------------------------------------------------------------------------------------------------------------------

interface BodyRun {
  mass_kg: number;
  params: HandParams;
  steps: number;
  start?: Vec3;
  velocity0?: Vec3;
  /** Rigidez (N/m) de un muelle de anclaje al punto de partida, en las tres deslizaderas. */
  hold_n_per_m?: number;
}

interface Trace {
  samples: HandSample[];
  positions: Vec3[];
  /** Velocidad al comienzo de cada paso. */
  velocities: Vec3[];
  /** Velocidad tras el último paso. */
  finalVelocity: Vec3;
  finalPosition: Vec3;
  state: HandState;
  kineticEnergy0_j: number;
  kineticEnergyEnd_j: number;
}

async function runBody(run: BodyRun): Promise<Trace> {
  const mj = await getMujoco();
  const start = run.start ?? ORIGIN;
  const stiff = run.hold_n_per_m ? ` stiffness="${run.hold_n_per_m}"` : "";
  const joints = (["x", "y", "z"] as const).map((a, i) => {
    const axis = ["1 0 0", "0 1 0", "0 0 1"][i];
    return `<joint name="j${a}" type="slide" axis="${axis}"${stiff}/>`;
  }).join("");
  const xml = `<mujoco><option timestep="${DT}" gravity="0 0 0" integrator="implicitfast"/><worldbody>
    <body name="probe" pos="${start.join(" ")}">${joints}
      <inertial pos="0 0 0" mass="${run.mass_kg}" diaginertia="1 1 1"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body></worldbody></mujoco>`;
  const model = mj.MjModel.from_xml_string(xml);
  const data = new mj.MjData(model);
  try {
    if (run.velocity0) for (let i = 0; i < 3; i++) data.qvel[i] = run.velocity0[i]!;
    const velocity = (): Vec3 => [data.qvel[0]!, data.qvel[1]!, data.qvel[2]!];
    const energy = (): number => 0.5 * run.mass_kg * dot(velocity(), velocity());
    const kineticEnergy0_j = energy();
    let state = createHandState();
    const samples: HandSample[] = [];
    const positions: Vec3[] = [];
    const velocities: Vec3[] = [];
    const position = (): Vec3 => [start[0] + data.qpos[0]!, start[1] + data.qpos[1]!, start[2] + data.qpos[2]!];
    for (let i = 0; i < run.steps; i++) {
      mj.mj_kinematics(model, data);
      mj.mj_comPos(model, data);
      const grip = position();
      const v = velocity();
      const r = stepHand(run.params, state, { t: i * DT, dt: DT, grip_m: grip, grip_velocity_mps: v });
      state = r.state;
      samples.push(r.sample);
      positions.push(grip);
      velocities.push(v);
      (data.xfrc_applied as Float64Array).fill(0);
      applyForceAtPoint(data as unknown as Parameters<typeof applyForceAtPoint>[0], 1, grip, r.force_n);
      mj.mj_step(model, data);
    }
    const finalVelocity = velocity();
    state = settleHandWork(state, finalVelocity);
    return {
      samples, positions, velocities, finalVelocity, finalPosition: position(), state, kineticEnergy0_j,
      kineticEnergyEnd_j: energy(),
    };
  } finally {
    data.delete();
    model.delete();
  }
}

/** Tirón fuerte: el objetivo se aleja 3 m en 1 s; `slip_after_s` grande para que la mano no se suelte. */
const HARD_PULL: HandConfig = config({
  slip_after_s: 100,
  phases: [{ start_s: 0, duration_s: 1, offset_m: [3, 0, 0] }],
});

const max = (xs: readonly number[]): number => xs.reduce((m, x) => Math.max(m, x), -Infinity);

describe("valores por defecto y validación", () => {
  it("los valores por defecto son los de la issue #76", () => {
    expect(HAND_DEFAULTS).toEqual({
      k_n_per_m: 3000, c_ns_per_m: 200, rope_slack_m: 0.03, max_force_n: 500, max_power_w: 600, reach_m: 0.75,
      slip_after_s: 0.4,
    });
    const rope = resolveHandParams({ link: "rope", shoulder_m: ORIGIN });
    expect(rope).toMatchObject({
      k_n_per_m: 3000, c_ns_per_m: 200, slack_m: 0.03, max_force_n: 500, max_power_w: 600, reach_m: 0.75, slip_after_s: 0.4,
      close_s: 0, release_s: null,
    });
    expect(resolveHandParams({ link: "clamp", shoulder_m: ORIGIN }).slack_m).toBe(0);
    expect(resolveHandParams({ link: "push", shoulder_m: ORIGIN }).slack_m).toBe(0);
  });

  it("rechaza valores no válidos con el campo en el mensaje", () => {
    expect(() => resolveHandParams(config({ k_n_per_m: 0 }))).toThrow(/k_n_per_m/);
    expect(() => resolveHandParams(config({ max_force_n: Number.NaN }))).toThrow(/max_force_n/);
    expect(() => resolveHandParams(config({ max_power_w: -1 }))).toThrow(/max_power_w/);
    expect(() => resolveHandParams(config({ c_ns_per_m: -1 }))).toThrow(/c_ns_per_m/);
    expect(() => resolveHandParams(config({ shoulder_m: [0, Number.POSITIVE_INFINITY, 0] }))).toThrow(/shoulder_m/);
    expect(() => resolveHandParams(config({ link: "clamp", slack_m: 0.01 }))).toThrow(/clamp no tiene holgura/);
    expect(() => resolveHandParams(config({ axis: [1, 0, 0] }))).toThrow(/solo se admite en push/);
    expect(() => resolveHandParams(config({ link: "push", axis: [0, 0, 0] }))).toThrow(/axis/);
    expect(() => resolveHandParams(config({ close_s: 1, release_s: 0.5 }))).toThrow(/release_s/);
    expect(() => resolveHandParams(config({
      phases: [{ start_s: 0, duration_s: 1, offset_m: [1, 0, 0] }, { start_s: 0.5, duration_s: 1, offset_m: [2, 0, 0] }],
    }))).toThrow(/solapa/);
    expect(() => resolveHandParams(config({ close_s: 1, phases: [{ start_s: 0.5, duration_s: 1, offset_m: [1, 0, 0] }] })))
      .toThrow(/anterior a close_s/);
  });
});

describe("objetivo por fases (mínimo tirón)", () => {
  it("el perfil va de 0 a 1 con velocidad nula en los extremos y simétrico", () => {
    expect(minimumJerk(0)).toBe(0);
    expect(minimumJerk(1)).toBe(1);
    expect(minimumJerk(0.5)).toBeCloseTo(0.5, 12);
    const h = 1e-6;
    expect((minimumJerk(h) - minimumJerk(0)) / h).toBeLessThan(1e-4);
    expect((minimumJerk(1) - minimumJerk(1 - h)) / h).toBeLessThan(1e-4);
    let previous = 0;
    for (let i = 1; i <= 100; i++) {
      const value = minimumJerk(i / 100);
      expect(value).toBeGreaterThanOrEqual(previous);
      expect(value + minimumJerk(1 - i / 100)).toBeCloseTo(1, 12);
      previous = value;
    }
  });

  it("los tramos se encadenan, se sostienen entre ellos y vuelven a un desplazamiento anterior", () => {
    const phases = [
      { start_s: 0.1, duration_s: 0.4, offset_m: [0.2, 0, 0] as const },
      { start_s: 0.8, duration_s: 0.2, offset_m: [0.2, 0.1, -0.3] as const },
    ];
    expect(handTargetOffset(phases, 0)).toEqual([0, 0, 0]);
    expect(handTargetOffset(phases, 0.3)[0]).toBeCloseTo(0.1, 12);
    expect(handTargetOffset(phases, 0.6)).toEqual([0.2, 0, 0]); // entre tramos, se sostiene
    expect(handTargetOffset(phases, 0.9)[1]).toBeCloseTo(0.05, 12);
    expect(handTargetOffset(phases, 5)).toEqual([0.2, 0.1, -0.3]);
  });

  it("el origen del objetivo es la posición del agarre al cerrarse la mano, no la anterior", () => {
    const params = resolveHandParams(config({
      link: "clamp", close_s: 0.2, phases: [{ start_s: 0.2, duration_s: 0.1, offset_m: [0.1, 0, 0] }],
    }));
    let state = createHandState();
    let r = pureStep(params, state, 0.1, [5, 5, 5]); // antes de cerrar: no actúa
    expect(r.sample.held).toBe(false);
    expect(r.force_n).toEqual([0, 0, 0]);
    expect(r.sample.target_m).toBeNull();
    state = r.state;
    r = pureStep(params, state, 0.2, [1, 2, 3]);
    expect(r.events).toEqual([{ t: 0.2, kind: "close" }]);
    expect(r.state.origin_m).toEqual([1, 2, 3]);
    expect(r.sample.target_m).toEqual([1, 2, 3]); // el desplazamiento empieza en 0
    expect(r.force_n).toEqual([0, 0, 0]);
    r = pureStep(params, r.state, 0.4, [1, 2, 3]);
    expect(r.sample.target_m).toEqual([1.1, 2, 3]);
  });
});

describe("rope: solo tracción, con holgura", () => {
  it("nunca empuja: la fuerza apunta siempre hacia el objetivo, de cualquier posición y velocidad (barrido puro)", () => {
    const params = resolveHandParams(config({ max_force_n: 1e9, max_power_w: 1e9 }));
    let checked = 0;
    for (let i = -4; i <= 4; i++) {
      for (let j = -4; j <= 4; j++) {
        for (const speed of [-3, -0.4, 0, 0.4, 3]) {
          const grip: Vec3 = [0.03 * i, 0.03 * j, 0.02 * (i - j)];
          const v: Vec3 = [speed, -speed / 2, speed / 3];
          const p = resolveHandParams(config({
            max_force_n: 1e9, max_power_w: 1e9, close_s: 0, phases: [{ start_s: 0, duration_s: 0.01, offset_m: [0.05 * j, 0.1, 0.04 * i] }],
          }));
          const closed = pureStep(p, createHandState(), 0, ORIGIN);
          const r = pureStep(p, closed.state, 0.01, grip, v);
          const toTarget = sub(r.sample.target_m!, grip);
          expect(dot(r.force_n, toTarget)).toBeGreaterThanOrEqual(0);
          if (norm(r.force_n) > 0) {
            // En la línea mano–agarre: paralela a d y en su mismo sentido.
            expect(dot(r.force_n, toTarget)).toBeCloseTo(norm(r.force_n) * norm(toTarget), 6);
          }
          checked++;
        }
      }
    }
    expect(params.link).toBe("rope");
    expect(checked).toBe(9 * 9 * 5);
  });

  it("dentro de la holgura la fuerza es exactamente 0; al acercarse el objetivo al agarre la fuerza cae a 0 (agarre parado)", () => {
    const params = resolveHandParams(config({
      phases: [
        { start_s: 0.0, duration_s: 0.2, offset_m: [0.15, 0, 0] }, // se aleja: tensa
        { start_s: 0.3, duration_s: 0.2, offset_m: [0.02, 0, 0] }, // vuelve hacia el agarre, a 2 cm
        { start_s: 0.6, duration_s: 0.2, offset_m: [0, 0, 0] },
      ],
    }));
    let state = createHandState();
    const forces: number[] = [];
    for (let i = 0; i <= 400; i++) {
      const t = i * DT;
      const r = pureStep(params, state, t, ORIGIN);
      state = r.state;
      forces.push(r.force_n[0]);
      const L = norm(sub(r.sample.target_m!, ORIGIN));
      if (L <= 0.03) expect(r.force_n).toEqual([0, 0, 0]);
      expect(r.force_n[0]).toBeGreaterThanOrEqual(0);
    }
    expect(max(forces)).toBeCloseTo(3000 * (0.15 - 0.03), 6); // estaba tensa de verdad
    expect(forces[forces.length - 1]).toBe(0);
    expect(forces[300]).toBe(0); // 0,6 s: hacia el agarre, dentro de la holgura
  });

  it("con un cuerpo de 10 kg en el motor: la fuerza nunca apunta fuera del objetivo y es 0 dentro de la holgura", async () => {
    // El objetivo va y viene: el cuerpo lo persigue, lo sobrepasa y pasa varias veces por la zona de holgura.
    const params = resolveHandParams(config({
      slip_after_s: 100,
      phases: [
        { start_s: 0, duration_s: 0.3, offset_m: [0.2, 0.1, 0] },
        { start_s: 0.5, duration_s: 0.3, offset_m: [-0.2, 0.1, 0.05] },
        { start_s: 1.0, duration_s: 0.3, offset_m: [0.1, -0.1, 0] },
      ],
    }));
    const trace = await runBody({ mass_kg: 10, params, steps: 1000 });
    let inSlack = 0;
    let pulling = 0;
    trace.samples.forEach((s, i) => {
      const d = sub(s.target_m!, trace.positions[i]!);
      expect(dot(s.force_n, d)).toBeGreaterThanOrEqual(-1e-9);
      if (norm(d) <= 0.03) {
        expect(s.force_n).toEqual([0, 0, 0]);
        inSlack++;
      }
      if (s.force_magnitude_n > 0) pulling++;
    });
    console.log(`rope 10 kg: ${inSlack} pasos dentro de la holgura (fuerza 0) y ${pulling} pasos tirando, de ${trace.samples.length}`);
    expect(inSlack).toBeGreaterThan(50);
    expect(pulling).toBeGreaterThan(50);
  });
});

describe("push solo empuja y clamp hace las dos cosas", () => {
  const fixed = (link: "rope" | "clamp" | "push", targetOffset: Vec3, extra: Partial<HandConfig> = {}): Vec3 => {
    const params = resolveHandParams(config({
      link, ...(link === "push" ? { axis: [1, 0, 0] as const } : {}), slack_m: link === "clamp" ? 0 : 0.03,
      phases: [{ start_s: 0, duration_s: 0.01, offset_m: targetOffset }], ...extra,
    }));
    const closed = pureStep(params, createHandState(), 0, ORIGIN);
    return pureStep(params, closed.state, 0.01, ORIGIN).force_n; // agarre parado en el origen
  };

  it("push: con el objetivo delante en su eje empuja (k·(s − holgura)); detrás no hace nada", () => {
    expect(fixed("push", [0.1, 0, 0])[0]).toBeCloseTo(3000 * (0.1 - 0.03), 9);
    expect(fixed("push", [0.1, 0, 0])[1]).toBe(0);
    expect(fixed("push", [-0.1, 0, 0])).toEqual([0, 0, 0]); // el eje apunta a +X: no puede tirar
    expect(fixed("push", [0.02, 0, 0])).toEqual([0, 0, 0]); // dentro de la holgura
    expect(fixed("push", [0, 0.2, 0])).toEqual([0, 0, 0]); // solo cuenta la componente en el eje
    // Contacto perdido: el objetivo queda detrás del agarre y el agarre se aleja deprisa. Sin contacto no hay fuerza (ni de amortiguador).
    const params = resolveHandParams(config({ link: "push", axis: [1, 0, 0], phases: [{ start_s: 0, duration_s: 0.01, offset_m: [-0.1, 0, 0] }] }));
    const closed = pureStep(params, createHandState(), 0, ORIGIN);
    for (const vx of [-5, -1, 1, 5]) expect(pureStep(params, closed.state, 0.01, ORIGIN, [vx, 0, 0]).force_n).toEqual([0, 0, 0]);
  });

  it("clamp: tira y empuja, F = k·d − c·v", () => {
    expect(fixed("clamp", [0.1, 0, 0])[0]).toBeCloseTo(300, 9);
    expect(fixed("clamp", [-0.1, 0, 0])[0]).toBeCloseTo(-300, 9);
    expect(fixed("clamp", [0, 0.1, 0.05])).toEqual([0, 300, 150]);
    const params = resolveHandParams(config({ link: "clamp", phases: [] }));
    const closed = pureStep(params, createHandState(), 0, ORIGIN);
    const r = pureStep(params, closed.state, 0.01, ORIGIN, [0.3, -0.2, 0.1]); // objetivo = agarre: solo amortigua
    expect(r.force_n[0]).toBeCloseTo(-60, 9);
    expect(r.force_n[1]).toBeCloseTo(40, 9);
    expect(r.force_n[2]).toBeCloseTo(-20, 9);
  });

  it("rope no empuja pero clamp sí, con el mismo objetivo y el mismo cuerpo (comparación directa)", () => {
    // El agarre se mueve hacia el objetivo más deprisa de lo que el amortiguador tolera: rope suelta la tensión, clamp frena.
    const grip: Vec3 = [0, 0, 0];
    const v: Vec3 = [2, 0, 0]; // c·v = 400 N frente a k·(L − holgura) = 3000·0.07 = 210 N: el corchete es negativo
    const make = (link: "rope" | "clamp") => {
      const p = resolveHandParams(config({ link, slack_m: link === "clamp" ? 0 : 0.03, phases: [{ start_s: 0, duration_s: 0.01, offset_m: [0.1, 0, 0] }] }));
      const closed = pureStep(p, createHandState(), 0, ORIGIN);
      return pureStep(p, closed.state, 0.01, grip, v).force_n;
    };
    expect(make("rope")).toEqual([0, 0, 0]);
    expect(make("clamp")[0]).toBeCloseTo(3000 * 0.1 - 400, 9); // -100 N: empuja hacia atrás, frenando
  });

  it("push por defecto empuja alejándose del hombro; con el cuerpo en el motor nunca tira y solo actúa en el eje", async () => {
    const params = resolveHandParams({
      link: "push", shoulder_m: [-0.5, 0, 0], reach_m: 100, slip_after_s: 100,
      phases: [
        { start_s: 0, duration_s: 0.3, offset_m: [0.2, 0.05, 0] },
        { start_s: 0.5, duration_s: 0.3, offset_m: [-0.2, 0.05, 0] },
      ],
    });
    const trace = await runBody({ mass_kg: 10, params, steps: 600 });
    expect(trace.state.axis).toEqual([1, 0, 0]); // del hombro (-0,5, 0, 0) al agarre (0, 0, 0)
    let pushing = 0;
    for (const s of trace.samples) {
      expect(s.force_n[0]).toBeGreaterThanOrEqual(0);
      expect(s.force_n[1]).toBe(0);
      expect(s.force_n[2]).toBe(0);
      if (s.force_n[0] > 0) pushing++;
    }
    expect(pushing).toBeGreaterThan(50);
  });
});

describe("topes de fuerza y de potencia (cuerpo de 10 kg en MuJoCo WASM)", () => {
  it("la fuerza nunca supera el tope y la potencia nunca supera el tope (±2 %)", async () => {
    const params = resolveHandParams(HARD_PULL);
    const trace = await runBody({ mass_kg: 10, params, steps: 750 });
    const peakForce = max(trace.samples.map((s) => s.force_magnitude_n));
    // Potencia con la velocidad al comienzo del paso (la del tope) y con la velocidad media del paso (energía real).
    const powerStart = trace.samples.map((s, i) => dot(s.force_n, trace.velocities[i]!));
    const powerMean = trace.samples.map((s, i) => {
      const next = trace.velocities[i + 1] ?? trace.finalVelocity;
      return dot(s.force_n, [(trace.velocities[i]![0] + next[0]) / 2, (trace.velocities[i]![1] + next[1]) / 2, (trace.velocities[i]![2] + next[2]) / 2]);
    });
    const peakPowerStart = max(powerStart);
    const peakPowerMean = max(powerMean);
    const forceLimited = trace.samples.filter((s) => s.force_limited).length;
    const powerLimited = trace.samples.filter((s) => s.power_limited).length;
    console.log(`topes 10 kg: fuerza máx ${peakForce.toFixed(4)} N (tope 500); potencia máx F·v_inicio ${peakPowerStart.toFixed(3)} W, `
      + `F·v_media ${peakPowerMean.toFixed(3)} W (tope 600); pasos en tope de fuerza ${forceLimited}, de potencia ${powerLimited}, de ${trace.samples.length}`);
    expect(forceLimited).toBeGreaterThan(5); // la prueba no es vacua: los dos topes actuaron
    expect(powerLimited).toBeGreaterThan(50);
    expect(peakForce).toBeLessThanOrEqual(500 * (1 + 1e-12));
    expect(peakPowerStart).toBeLessThanOrEqual(600 * (1 + 1e-9));
    expect(peakPowerMean).toBeLessThanOrEqual(600 * 1.02);
    expect(trace.state.totals.max_force_n).toBeLessThanOrEqual(500 * (1 + 1e-12));
    expect(trace.state.totals.max_power_w).toBeLessThanOrEqual(600 * (1 + 1e-9));
  });

  it("el tope de fuerza se aplica antes que el de potencia, y los topes configurados mandan sobre los de por defecto", async () => {
    // Demanda 3000·0,27 = 810 N con el agarre parado: tope de fuerza 500 N. Con v = 2 m/s, F·v = 1000 W > 600 W: tope de potencia.
    const params = resolveHandParams(config({ phases: [{ start_s: 0, duration_s: 0.01, offset_m: [0.3, 0, 0] }] }));
    const closed = pureStep(params, createHandState(), 0, ORIGIN);
    const r = pureStep(params, closed.state, 0.01, ORIGIN, [0, 0, 0]);
    expect(r.sample.demand_force_n).toBeCloseTo(810, 9);
    expect(r.force_n[0]).toBeCloseTo(500, 9);
    expect(r.sample.force_limited).toBe(true);
    expect(r.sample.power_limited).toBe(false);
    // Con velocidad v a lo largo de la fuerza: la demanda baja con el amortiguador c·v; se verifica el orden con valores exactos.
    const v = 2;
    const demand = 3000 * (0.3 - 0.03) - 200 * v; // 410 N (< 500): solo el de potencia puede actuar
    const r2 = pureStep(params, closed.state, 0.01, ORIGIN, [v, 0, 0]);
    expect(r2.sample.demand_force_n).toBeCloseTo(demand, 9);
    expect(r2.sample.force_limited).toBe(false);
    expect(r2.sample.power_limited).toBe(true);
    expect(r2.force_n[0]).toBeCloseTo(600 / v, 9); // P = F·v = 600 W exactos
    // Ahora el tope de fuerza actúa primero y después el de potencia sobre la fuerza ya recortada (agarre que se aleja).
    const strong = resolveHandParams(config({ k_n_per_m: 6000, max_power_w: 450, phases: [{ start_s: 0, duration_s: 0.01, offset_m: [0.3, 0, 0] }] }));
    const c2 = pureStep(strong, createHandState(), 0, ORIGIN);
    const r3 = pureStep(strong, c2.state, 0.01, ORIGIN, [1, 0, 0]);
    expect(r3.sample.force_limited).toBe(true);
    expect(r3.sample.power_limited).toBe(true);
    expect(r3.force_n[0]).toBeCloseTo(450, 9); // 500 N recortados a 450 W / 1 m/s
  });

  it("la potencia negativa (la fuerza frena el agarre) no se recorta", () => {
    const params = resolveHandParams(config({ link: "clamp", max_power_w: 10, max_force_n: 5000, phases: [{ start_s: 0, duration_s: 0.01, offset_m: [-0.1, 0, 0] }] }));
    const closed = pureStep(params, createHandState(), 0, ORIGIN);
    const r = pureStep(params, closed.state, 0.01, ORIGIN, [3, 0, 0]);
    expect(r.sample.power_w).toBeLessThan(0);
    expect(r.sample.power_limited).toBe(false);
    expect(r.force_n[0]).toBeCloseTo(-300 - 600, 9);
  });

  it("el trabajo total aportado nunca supera max_power_w × duración, y coincide con la energía cinética ganada", async () => {
    for (const steps of [250, 500, 750]) {
      const params = resolveHandParams(HARD_PULL);
      const trace = await runBody({ mass_kg: 10, params, steps });
      const duration = steps * DT;
      const work = trace.state.totals.work_j;
      const gained = trace.kineticEnergyEnd_j - trace.kineticEnergy0_j;
      // Acumulado en cada instante: W(t) <= P·t.
      let worst = 0;
      trace.samples.forEach((s) => { worst = Math.max(worst, s.work_j - 600 * s.t); });
      console.log(`trabajo 10 kg, ${duration.toFixed(2)} s: W = ${work.toFixed(3)} J, tope P·T = ${(600 * duration).toFixed(1)} J (${(100 * work / (600 * duration)).toFixed(1)} %), `
        + `ΔEc = ${gained.toFixed(3)} J, máx(W(t) − P·t) = ${worst.toFixed(3)} J`);
      expect(work).toBeLessThanOrEqual(600 * duration);
      expect(worst).toBeLessThanOrEqual(0);
      expect(work).toBeGreaterThan(0);
      // Sin gravedad ni otras fuerzas, todo el trabajo de la mano es energía cinética (el trapecio es exacto en el esquema del motor).
      expect(work).toBeCloseTo(gained, 6);
    }
  });

  it("con el tope de potencia actuando todo el tiempo (objetivo que huye), el trabajo se acerca a P·T sin superarlo", async () => {
    // El objetivo huye a más velocidad de la que el cuerpo puede alcanzar con 600 W: el tope de potencia es el que manda.
    const params = resolveHandParams(config({ slip_after_s: 100, phases: [{ start_s: 0, duration_s: 1, offset_m: [40, 0, 0] }] }));
    for (const steps of [250, 500, 750]) {
      const trace = await runBody({ mass_kg: 10, params, steps });
      const duration = steps * DT;
      const work = trace.state.totals.work_j;
      let worst = -Infinity;
      trace.samples.forEach((s) => { worst = Math.max(worst, s.work_j - 600 * s.t); });
      const limited = trace.samples.filter((s) => s.power_limited).length;
      console.log(`trabajo con huida, ${duration.toFixed(2)} s: W = ${work.toFixed(3)} J frente a P·T = ${(600 * duration).toFixed(1)} J (${(100 * work / (600 * duration)).toFixed(2)} %); pasos limitados por potencia ${limited}/${steps}; máx(W(t) − P·t) = ${worst.toFixed(3)} J`);
      expect(limited).toBeGreaterThan(0.5 * steps);
      expect(work).toBeLessThanOrEqual(600 * duration);
      expect(worst).toBeLessThanOrEqual(0);
      expect(work).toBeGreaterThan(0.6 * 600 * duration); // el tope se aprovecha: la prueba es ajustada, no holgada
    }
  });

  it("el tope de potencia se mide con la velocidad al comienzo del paso: con un cuerpo ya en marcha el trabajo no excede P·T en más de un 2 %", async () => {
    // Caso desfavorable (no es el de la issue): el cuerpo ya va a 1,2 m/s (donde F = 500 N y P = 600 W coinciden) al cerrarse la mano.
    const params = resolveHandParams(config({ slip_after_s: 100, phases: [{ start_s: 0, duration_s: 1, offset_m: [40, 0, 0] }] }));
    const trace = await runBody({ mass_kg: 10, params, steps: 250, velocity0: [1.2, 0, 0] });
    const duration = 250 * DT;
    const work = trace.state.totals.work_j;
    console.log(`trabajo con cuerpo veloz (1,2 m/s): W = ${work.toFixed(3)} J frente a P·T = ${(600 * duration).toFixed(1)} J (${(100 * work / (600 * duration)).toFixed(2)} %)`);
    expect(work).toBeLessThanOrEqual(600 * duration * 1.02);
  });
});

describe("fuerza estática con el cuerpo fijado", () => {
  it("F = k·(L − holgura) en rope, k·d en clamp y k·(s − holgura) en push (agarre parado, sin motor)", () => {
    const L = 0.12;
    const make = (link: "rope" | "clamp" | "push", dir: Vec3) => {
      const params = resolveHandParams(config({
        link, ...(link === "push" ? { axis: dir } : {}), max_force_n: 1e6, max_power_w: 1e9,
        phases: [{ start_s: 0, duration_s: 0.01, offset_m: [L * dir[0], L * dir[1], L * dir[2]] }],
      }));
      const closed = pureStep(params, createHandState(), 0, [0.5, -0.2, 1]);
      return pureStep(params, closed.state, 0.05, [0.5, -0.2, 1]).force_n;
    };
    const dir: Vec3 = [0.6, 0, 0.8];
    const rope = make("rope", dir);
    expect(norm(rope)).toBeCloseTo(3000 * (L - 0.03), 9);
    expect(rope[0] / norm(rope)).toBeCloseTo(0.6, 12);
    expect(norm(make("clamp", dir))).toBeCloseTo(3000 * L, 9);
    expect(norm(make("push", dir))).toBeCloseTo(3000 * L, 9); // holgura 0 por defecto
  });

  it("en el motor: un cuerpo de 10 kg sujeto por un muelle de anclaje queda en equilibrio y la fuerza de la mano es k·(L − holgura) y es la que el anclaje equilibra", async () => {
    const hold = 2000;
    const params = resolveHandParams(config({
      max_force_n: 5000, max_power_w: 1e6, slip_after_s: 100,
      phases: [{ start_s: 0, duration_s: 0.2, offset_m: [0.2, 0, 0] }],
    }));
    const trace = await runBody({ mass_kg: 10, params, steps: 3000, hold_n_per_m: hold });
    const last = trace.samples[trace.samples.length - 1]!;
    const grip = trace.finalPosition;
    const L = norm(sub(last.target_m!, grip));
    const expected = 3000 * (L - 0.03);
    const anchor = hold * grip[0];
    console.log(`estática 10 kg: L = ${L.toFixed(6)} m, F mano = ${last.force_magnitude_n.toFixed(4)} N, k(L − holgura) = ${expected.toFixed(4)} N, anclaje K·x = ${anchor.toFixed(4)} N, |v| = ${norm(trace.finalVelocity).toExponential(2)} m/s`);
    expect(norm(trace.finalVelocity)).toBeLessThan(1e-6);
    expect(last.force_magnitude_n).toBeCloseTo(expected, 9);
    expect(last.force_magnitude_n).toBeCloseTo(anchor, 3); // el motor equilibra lo que la mano aplica
    expect(grip[0]).toBeCloseTo((3000 * (0.2 - 0.03)) / (3000 + hold), 5); // x = k·(0,2 − holgura)/(k + K)
  });
});

describe("suelta: programada y por resbalón", () => {
  it("suelta programada: a partir de release_s la fuerza es 0 y queda un evento", async () => {
    const params = resolveHandParams(config({
      release_s: 0.5, slip_after_s: 100, phases: [{ start_s: 0, duration_s: 0.4, offset_m: [0.5, 0, 0] }],
    }));
    const trace = await runBody({ mass_kg: 10, params, steps: 500 });
    expect(trace.state.events).toEqual([{ t: 0, kind: "close" }, { t: 0.5, kind: "release" }]);
    trace.samples.forEach((s) => {
      if (s.t < 0.5 - 1e-9) expect(s.held).toBe(true);
      else {
        expect(s.held).toBe(false);
        expect(s.force_n).toEqual([0, 0, 0]);
      }
    });
    expect(max(trace.samples.filter((s) => s.t < 0.5).map((s) => s.force_magnitude_n))).toBeGreaterThan(100);
  });

  it("la mano suelta sola tras slip_after_s en el tope de fuerza y deja un evento", async () => {
    // El objetivo huye (20 m en 1 s) más deprisa de lo que el tope de potencia deja seguirlo: la demanda se queda en el tope.
    const params = resolveHandParams(config({ phases: [{ start_s: 0, duration_s: 1, offset_m: [20, 0, 0] }] }));
    const trace = await runBody({ mass_kg: 10, params, steps: 750 });
    const firstCap = trace.samples.find((s) => s.force_limited)!;
    const slip = trace.state.events.find((e) => e.kind === "slip");
    expect(slip).toBeDefined();
    const heldAtCap = slip!.t - firstCap.t;
    console.log(`resbalón 10 kg: primer paso en el tope a t = ${firstCap.t.toFixed(3)} s, suelta a t = ${slip!.t.toFixed(3)} s (${heldAtCap.toFixed(4)} s en el tope; slip_after_s = 0,4)`);
    expect(heldAtCap).toBeGreaterThanOrEqual(0.4 - 1e-9);
    expect(heldAtCap).toBeLessThanOrEqual(0.4 + 2 * DT);
    // Hasta ese instante siempre en el tope; después, fuerza 0 y mano suelta.
    trace.samples.forEach((s) => {
      if (s.t >= firstCap.t && s.t < slip!.t - 1e-9) expect(s.force_limited).toBe(true);
      if (s.t >= slip!.t - 1e-9) {
        expect(s.held).toBe(false);
        expect(s.force_n).toEqual([0, 0, 0]);
      }
    });
    expect(trace.state.events.filter((e) => e.kind === "slip")).toHaveLength(1);
    expect(trace.state.totals.time_at_cap_s).toBeCloseTo(heldAtCap, 6);
  });

  it("no suelta si el tope dura menos que slip_after_s o si se interrumpe (control negativo)", () => {
    // Agarre parado. 0,3 s en el tope, 0,1 s por debajo, otros 0,3 s en el tope: ninguno de los dos tramos llega a 0,4 s.
    const params = resolveHandParams(config({
      phases: [
        { start_s: 0, duration_s: 0.01, offset_m: [0.3, 0, 0] }, // demanda 810 N: tope
        { start_s: 0.3, duration_s: 0.01, offset_m: [0.1, 0, 0] }, // 210 N: por debajo
        { start_s: 0.4, duration_s: 0.01, offset_m: [0.3, 0, 0] },
      ],
    }));
    let state = createHandState();
    let capped = 0;
    for (let i = 0; i < 350; i++) {
      const r = pureStep(params, state, i * DT, ORIGIN);
      state = r.state;
      if (r.sample.force_limited) capped++;
    }
    expect(capped).toBeGreaterThan(200); // sí estuvo mucho tiempo en el tope
    expect(state.events.filter((e) => e.kind === "slip")).toHaveLength(0);
    expect(state.phase).toBe("holding");
  });
});

describe("alcance", () => {
  it("recorta el objetivo a la esfera de radio reach_m alrededor del hombro (puro)", () => {
    const shoulder: Vec3 = [0.1, -0.3, 1.2];
    const params = resolveHandParams({
      link: "rope", shoulder_m: shoulder, phases: [{ start_s: 0, duration_s: 0.01, offset_m: [1, 2, 2] }],
    });
    const grip: Vec3 = [0.3, -0.3, 1.2]; // dentro de la esfera (a 0,2 m del hombro)
    const closed = pureStep(params, createHandState(), 0, grip);
    const r = pureStep(params, closed.state, 0.05, grip);
    expect(r.sample.target_unclipped_m).toEqual([1.3, 1.7, 3.2]);
    expect(r.sample.reach_limited).toBe(true);
    const fromShoulder = sub(r.sample.target_m!, shoulder);
    expect(norm(fromShoulder)).toBeCloseTo(0.75, 12);
    // Misma dirección que el objetivo sin recortar visto desde el hombro.
    const raw = sub(r.sample.target_unclipped_m!, shoulder);
    expect(dot(fromShoulder, raw)).toBeCloseTo(norm(fromShoulder) * norm(raw), 9);
    // Un objetivo dentro de la esfera no se toca.
    const inside = resolveHandParams({ link: "rope", shoulder_m: shoulder, phases: [{ start_s: 0, duration_s: 0.01, offset_m: [0.1, 0, 0] }] });
    const c2 = pureStep(inside, createHandState(), 0, grip);
    const r2 = pureStep(inside, c2.state, 0.05, grip);
    expect(r2.sample.reach_limited).toBe(false);
    expect(r2.sample.target_m).toEqual(r2.sample.target_unclipped_m);
  });

  it("en el motor: el objetivo nunca sale de la esfera y el cuerpo acaba dentro de ella (más la holgura)", async () => {
    const params = resolveHandParams({
      link: "rope", shoulder_m: [0, 0, 0], slip_after_s: 100,
      phases: [{ start_s: 0, duration_s: 1, offset_m: [3, 0, 0] }],
    });
    const trace = await runBody({ mass_kg: 10, params, steps: 1500, start: [0.2, 0, 0] });
    const maxTarget = max(trace.samples.map((s) => norm(s.target_m!)));
    const maxBody = max(trace.positions.map((p) => norm(p)));
    const finalDistance = norm(trace.finalPosition);
    console.log(`alcance 10 kg: |objetivo| máx ${maxTarget.toFixed(6)} m, |cuerpo| máx ${maxBody.toFixed(4)} m (alcance 0,75 m, holgura 0,03 m); final ${finalDistance.toFixed(4)} m`);
    expect(maxTarget).toBeLessThanOrEqual(0.75 + 1e-12);
    expect(trace.samples.some((s) => s.reach_limited)).toBe(true);
    // El cuerpo se pasa del alcance por su cantidad de movimiento (la cuerda no empuja), pero vuelve: sin recorte llegaría a más de 3 m.
    expect(maxBody).toBeLessThan(1);
    expect(finalDistance).toBeLessThanOrEqual(0.75 + 0.03 + 1e-3);
  });
});

describe("simetría (reflejo en Y)", () => {
  const phases = [
    { start_s: 0.0, duration_s: 0.3, offset_m: [0.25, 0.15, 0.05] as Vec3 },
    { start_s: 0.4, duration_s: 0.3, offset_m: [0.05, -0.1, 0.1] as Vec3 },
  ];
  const mirrorPhases = phases.map((p) => ({ ...p, offset_m: mirrorY(p.offset_m) }));

  it("para cada tipo de unión, el caso reflejado da la fuerza reflejada (puro, exacto)", () => {
    for (const link of ["rope", "clamp", "push"] as const) {
      const base: Partial<HandConfig> = { link, shoulder_m: [-0.4, 0.2, 0.1], reach_m: 0.9, slip_after_s: 100, ...(link === "push" ? { axis: [0.5, 0.3, 0.2] as Vec3 } : {}) };
      const a = resolveHandParams({ ...base, phases } as HandConfig);
      const b = resolveHandParams({
        ...base, shoulder_m: mirrorY(base.shoulder_m!), phases: mirrorPhases, ...(base.axis ? { axis: mirrorY(base.axis) } : {}),
      } as HandConfig);
      let sa = createHandState();
      let sb = createHandState();
      let nonzero = 0;
      for (let i = 0; i < 400; i++) {
        const grip: Vec3 = [0.02 * Math.sin(i / 20), 0.05 * Math.cos(i / 17) + 0.01, 0.03 * Math.sin(i / 9)];
        const v: Vec3 = [0.3 * Math.cos(i / 20), -0.4 * Math.sin(i / 17), 0.2 * Math.cos(i / 9)];
        const ra = stepHand(a, sa, { t: i * DT, dt: DT, grip_m: grip, grip_velocity_mps: v });
        const rb = stepHand(b, sb, { t: i * DT, dt: DT, grip_m: mirrorY(grip), grip_velocity_mps: mirrorY(v) });
        sa = ra.state;
        sb = rb.state;
        const m = mirrorY(ra.force_n);
        for (let k = 0; k < 3; k++) expect(rb.force_n[k]).toBeCloseTo(m[k]!, 12);
        expect(rb.sample.force_limited).toBe(ra.sample.force_limited);
        expect(rb.sample.reach_limited).toBe(ra.sample.reach_limited);
        if (norm(ra.force_n) > 1) nonzero++;
      }
      expect(nonzero).toBeGreaterThan(20);
      expect(sb.totals.work_j).toBeCloseTo(sa.totals.work_j, 9);
    }
  });

  it("en el motor: el cuerpo de 10 kg sigue la trayectoria reflejada (diferencia < 1e-9 m) y el trabajo es el mismo", async () => {
    const cfg = { link: "rope", shoulder_m: [0.1, 0.2, 0] as Vec3, slip_after_s: 100 } as const;
    const a = await runBody({ mass_kg: 10, params: resolveHandParams({ ...cfg, phases }), steps: 600, start: [0, 0.1, 0] });
    const b = await runBody({
      mass_kg: 10, steps: 600, start: [0, -0.1, 0],
      params: resolveHandParams({ ...cfg, shoulder_m: mirrorY(cfg.shoulder_m), phases: mirrorPhases }),
    });
    let worst = 0;
    a.positions.forEach((p, i) => {
      const q = mirrorY(b.positions[i]!);
      worst = Math.max(worst, norm(sub(p, q)));
    });
    console.log(`simetría 10 kg: desviación máxima entre la trayectoria y su reflejo ${worst.toExponential(2)} m; trabajo ${a.state.totals.work_j.toFixed(6)} J frente a ${b.state.totals.work_j.toFixed(6)} J`);
    expect(norm(sub(a.finalPosition, mirrorY(b.finalPosition)))).toBeLessThan(1e-9);
    expect(worst).toBeLessThan(1e-9);
    expect(b.state.totals.work_j).toBeCloseTo(a.state.totals.work_j, 9);
    expect(max(a.samples.map((s) => s.force_magnitude_n))).toBeGreaterThan(50); // la prueba mueve de verdad el cuerpo
  });
});

describe("estabilidad con k = 3000 N/m, c = 200 N·s/m y paso de 2 ms", () => {
  /** Respuesta a un escalón de 10 cm del objetivo (en 20 ms) con holgura nula en clamp y por defecto en rope. */
  async function response(mass: number, link: "rope" | "clamp") {
    const params = resolveHandParams(config({
      link, max_force_n: 1e6, max_power_w: 1e9, slip_after_s: 100,
      phases: [{ start_s: 0, duration_s: 0.02, offset_m: [0.1, 0, 0] }],
    }));
    const trace = await runBody({ mass_kg: mass, params, steps: 2500 });
    const slack = link === "rope" ? 0.03 : 0;
    // Exceso del error de seguimiento sobre la holgura: dentro de la holgura la cuerda está floja y no hay nada que estabilizar.
    const err = trace.positions.map((p, i) => Math.max(0, Math.abs(trace.samples[i]!.target_m![0] - p[0]) - slack));
    const window = (a: number, b: number): number => max(err.slice(Math.round(a / DT), Math.round(b / DT)));
    const speed = trace.velocities.map((v) => norm(v));
    // Cambios de sentido de la velocidad (con |v| > 1 mm/s) tras 0,05 s: oscilación visible.
    let reversals = 0;
    for (let i = Math.round(0.05 / DT) + 1; i < trace.velocities.length; i++) {
      const a = trace.velocities[i - 1]![0];
      const b = trace.velocities[i]![0];
      if (a * b < 0 && Math.abs(a) > 1e-3 && Math.abs(b) > 1e-3) reversals++;
    }
    return {
      early: window(0.05, 0.5), mid: window(0.5, 1.5), late: window(3.5, 5), finalErr: err[err.length - 1]!,
      peakSpeed: max(speed), lateSpeed: max(speed.slice(Math.round(3.5 / DT))), reversals,
    };
  }

  it.each([10, 1])("%s kg: no oscila creciendo (rope y clamp)", async (mass) => {
    for (const link of ["rope", "clamp"] as const) {
      const r = await response(mass, link);
      console.log(`estabilidad ${mass} kg ${link}: exceso de error sobre la holgura 0,05–0,5 s ${r.early.toExponential(2)} m, 0,5–1,5 s ${r.mid.toExponential(2)} m, 3,5–5 s ${r.late.toExponential(2)} m; `
        + `velocidad máx ${r.peakSpeed.toFixed(3)} m/s, en 3,5–5 s ${r.lateSpeed.toExponential(2)} m/s; cambios de sentido (|v| > 1 mm/s): ${r.reversals}`);
      expect(Number.isFinite(r.peakSpeed)).toBe(true);
      expect(r.peakSpeed).toBeLessThan(5);
      expect(r.mid).toBeLessThanOrEqual(r.early + 1e-12);
      expect(r.late).toBeLessThanOrEqual(r.mid + 1e-12);
      expect(r.reversals).toBeLessThanOrEqual(3);
      expect(r.late).toBeLessThan(1e-4); // acaba dentro de la holgura (rope) o sobre el objetivo (clamp)
      expect(r.finalErr).toBeLessThan(1e-4);
      // clamp amortigua siempre; rope no amortigua dentro de la holgura (la cuerda está floja) y puede seguir a la deriva despacio.
      expect(r.lateSpeed).toBeLessThan(link === "clamp" ? 1e-3 : 0.05);
    }
  });

  it("barrido de masas ligeras (medido, se documenta en docs/VIRTUAL_HAND.md): estable desde 0,25 kg, inestable a 0,1 kg con clamp", async () => {
    const rows: string[] = [];
    const growth: Record<string, boolean> = {};
    for (const mass of [0.1, 0.2, 0.25, 0.5, 1, 2, 5, 10]) {
      for (const link of ["clamp", "rope"] as const) {
        const params = resolveHandParams(config({
          link, max_force_n: 1e9, max_power_w: 1e12, slip_after_s: 100,
          phases: [{ start_s: 0, duration_s: 0.02, offset_m: [0.1, 0, 0] }],
        }));
        const trace = await runBody({ mass_kg: mass, params, steps: 1500 });
        const speed = trace.velocities.map((v) => norm(v));
        const a = max(speed.slice(100, 400));
        const b = max(speed.slice(1000));
        const ratio = b / Math.max(a, 1e-300);
        const diverged = !Number.isFinite(b) || b > 10 || ratio > 1;
        growth[`${mass}/${link}`] = diverged;
        rows.push(`${mass} kg ${link}: v máx en 0,2–0,8 s ${a.toExponential(2)}, en 2–3 s ${b.toExponential(2)} m/s → ${diverged ? "CRECE" : "decae"}`);
      }
    }
    console.log(`barrido de estabilidad:\n${rows.join("\n")}`);
    for (const mass of [0.25, 0.5, 1, 2, 5, 10]) {
      expect(growth[`${mass}/clamp`]).toBe(false);
      expect(growth[`${mass}/rope`]).toBe(false);
    }
    expect(growth["0.1/clamp"]).toBe(true); // c·dt/m = 4 > 2: el amortiguador explícito invierte y amplifica la velocidad
  });
});

describe("pureza, determinismo y contabilidad", () => {
  it("stepHand no modifica sus argumentos y es determinista", () => {
    const params = resolveHandParams(config({ phases: [{ start_s: 0, duration_s: 0.5, offset_m: [0.3, 0.1, 0] }], release_s: 0.9 }));
    const run = () => {
      let state = createHandState();
      const out: unknown[] = [];
      for (let i = 0; i < 500; i++) {
        const before = JSON.stringify(state);
        const input = { t: i * DT, dt: DT, grip_m: [0.001 * i, 0, 0] as Vec3, grip_velocity_mps: [0.5, 0, 0] as Vec3 };
        const frozen = JSON.stringify(input);
        const r = stepHand(params, state, input);
        expect(JSON.stringify(state)).toBe(before);
        expect(JSON.stringify(input)).toBe(frozen);
        expect(r.state).not.toBe(state);
        state = r.state;
        out.push(r);
      }
      return JSON.stringify(out);
    };
    expect(run()).toBe(run());
    expect(JSON.stringify(params)).toBe(JSON.stringify(resolveHandParams(config({ phases: [{ start_s: 0, duration_s: 0.5, offset_m: [0.3, 0.1, 0] }], release_s: 0.9 }))));
  });

  it("rechaza entradas no finitas o un paso no positivo", () => {
    const params = resolveHandParams(config());
    const s = createHandState();
    expect(() => stepHand(params, s, { t: 0, dt: 0, grip_m: ORIGIN, grip_velocity_mps: ORIGIN })).toThrow(/dt/);
    expect(() => stepHand(params, s, { t: 0, dt: DT, grip_m: [Number.NaN, 0, 0], grip_velocity_mps: ORIGIN })).toThrow(/grip_m/);
    expect(() => stepHand(params, s, { t: 0, dt: DT, grip_m: ORIGIN, grip_velocity_mps: [0, Number.POSITIVE_INFINITY, 0] })).toThrow(/grip_velocity_mps/);
  });

  it("la reacción de Tori es −F en cada paso y su impulso acumulado es −∫F dt", async () => {
    const trace = await runBody({ mass_kg: 10, params: resolveHandParams(HARD_PULL), steps: 400 });
    const sum: [number, number, number] = [0, 0, 0];
    trace.samples.forEach((s) => {
      for (let k = 0; k < 3; k++) expect(s.reaction_n[k]).toBeCloseTo(-s.force_n[k]!, 15);
      for (let k = 0; k < 3; k++) sum[k] = sum[k]! - s.force_n[k]! * DT;
    });
    const total = trace.state.totals;
    for (let k = 0; k < 3; k++) {
      expect(total.reaction_impulse_ns[k]).toBeCloseTo(sum[k]!, 9);
      expect(total.impulse_ns[k]).toBeCloseTo(-sum[k]!, 9);
    }
    expect(total.reaction_impulse_ns[0]).toBeLessThan(-1);
    // Con una sola mano y sin otras fuerzas, el impulso sobre el cuerpo es su cantidad de movimiento: 10 kg · v.
    expect(total.impulse_ns[0]).toBeCloseTo(10 * trace.finalVelocity[0], 6);
  });
});

describe("applyForceAtPoint: fuerza y par r × F (MuJoCo WASM real)", () => {
  it("acumula la fuerza y el par respecto al centro de masas y el cuerpo gira como dice la dinámica", async () => {
    const mj = await getMujoco();
    const xml = `<mujoco><option timestep="${DT}" gravity="0 0 0" integrator="implicitfast"/><worldbody>
      <body name="free" pos="1 2 3"><freejoint/>
        <inertial pos="0 0 0" mass="10" diaginertia="0.5 0.5 0.5"/>
        <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>`;
    const model = mj.MjModel.from_xml_string(xml);
    const data = new mj.MjData(model);
    try {
      mj.mj_kinematics(model, data);
      mj.mj_comPos(model, data);
      const xfrc = data.xfrc_applied as Float64Array;
      xfrc.fill(0);
      const wrench = data as unknown as Parameters<typeof applyForceAtPoint>[0];
      // Centro de masas en (1; 2; 3). Fuerza (0, 100, 0) en el punto (1,3; 2; 3): brazo (0,3; 0; 0) → par z = 0,3·100 = 30 N·m.
      applyForceAtPoint(wrench, 1, [1.3, 2, 3], [0, 100, 0]);
      expect(Array.from(xfrc.subarray(6, 12))).toEqual([0, 100, 0, 0, 0, expect.closeTo(30, 12)]);
      // Es acumulativa y con un punto en el propio centro de masas no añade par.
      applyForceAtPoint(wrench, 1, [1, 2, 3], [0, 0, 50]);
      expect(Array.from(xfrc.subarray(6, 12))).toEqual([0, 100, 50, expect.closeTo(0, 12), expect.closeTo(0, 12), expect.closeTo(30, 12)]);
      // Dinámica: un paso desde el reposo.
      xfrc.fill(0);
      applyForceAtPoint(wrench, 1, [1.3, 2, 3], [0, 100, 0]);
      mj.mj_step(model, data);
      const qvel = data.qvel as Float64Array;
      expect(qvel[1]).toBeCloseTo((100 / 10) * DT, 9); // a = F/m
      expect(Math.abs(qvel[3]!) + Math.abs(qvel[4]!)).toBeLessThan(1e-9);
      expect(qvel[5]).toBeCloseTo((30 / 0.5) * DT, 6); // α = τ/I (la inercia está en el centro de masas)
      expect(() => applyForceAtPoint(wrench, 0, [0, 0, 0], [1, 0, 0])).toThrow(/Cuerpo/);
      expect(() => applyForceAtPoint(wrench, 1, [0, 0, 0], [Number.NaN, 0, 0])).toThrow(/force_n/);
    } finally {
      data.delete();
      model.delete();
    }
  });
});
