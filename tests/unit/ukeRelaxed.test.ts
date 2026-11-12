import { describe, expect, it } from "vitest";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { runScenario } from "../../src/scenario/run";
import type { Scenario } from "../../src/scenario/schema";
import { validateScenario } from "../../src/scenario/validate";
import { generateHumanoidMJCF } from "../../src/uke/generate";
import { attitudeJointDamping, relaxedJointDamping, setJointDamping, type DampingModel } from "../../src/uke/damping";
import { ukeJointDamping } from "../../src/uke/gains";

const DEG = Math.PI / 180;

function scenario(extra: Record<string, unknown>, actor: Record<string, unknown> = {}): Scenario {
  return validateScenario({
    schema: "hpl.scenario.v1", id: "uke_probe", duration_s: 3, record_hz: 125, ground: { friction: 1 },
    actors: [{ id: "uke", kind: "humanoid", mass_kg: 75, height_m: 1.75, ...actor }],
    props: [], effectors: [], ...extra,
  });
}

/** Empujón sagital (hacia +X) de `impulse` N·s, medio seno de 0,1 s, en el sitio `sternum` con un desplazamiento opcional. */
function push(impulseNs: number, offset: [number, number, number] = [0, 0, 0], direction = 1): Record<string, unknown> {
  const peak = (impulseNs * Math.PI) / (2 * 0.1);
  return {
    effectors: [{
      id: "push", kind: "prescribed_force", actor: "uke", site: "sternum", local_point_m: offset, frame: "world",
      force_n: [direction * peak, 0, 0], start_s: 0.2, duration_s: 0.1, waveform: "half_sine",
    }],
  };
}

describe("relaxed: amortiguación escrita en dof_damping en tiempo de ejecución (MuJoCo WASM real)", () => {
  it("el binding permite escribir model.dof_damping y el cambio tiene efecto en la simulación", async () => {
    const mj = await getMujoco();
    const xml = generateHumanoidMJCF({ mass_kg: 75, height_m: 1.76 });
    const model = mj.MjModel.from_xml_string(xml);
    const data = new mj.MjData(model);
    try {
      const dof = model.jnt_dofadr[model.jnt("left_elbow").id]!;
      expect(model.dof_damping[dof]).toBeCloseTo(1.6, 9); // valor del MJCF
      const run = (damping: number): number => {
        mj.mj_resetData(model, data);
        setJointDamping(model as unknown as DampingModel, { left_elbow: damping });
        expect(model.dof_damping[dof]).toBe(damping); // la lectura devuelve lo escrito
        data.qvel[dof] = -3; // rad/s; el codo se mueve en el aire (pelvis libre, cae 0,1 s)
        for (let i = 0; i < 50; i++) mj.mj_step(model, data);
        return Math.abs(data.qvel[dof]!);
      };
      const light = run(1.6);
      const heavy = run(200);
      const again = run(1.6);
      console.log(`dof_damping en marcha: velocidad del codo tras 0,1 s con 1,6: ${light.toFixed(3)} rad/s; con 200: ${heavy.toFixed(3)} rad/s`);
      expect(heavy).toBeLessThan(0.5 * light);
      expect(again).toBe(light); // determinista y reversible
      // Solo bisagras: la raíz libre no se toca.
      expect(() => setJointDamping(model as unknown as DampingModel, { pelvis_root: 1 })).toThrow();
    } finally {
      data.delete();
      model.delete();
    }
  });

  it("tense conserva la amortiguación de gains.ts y relaxed es 1,6·m·h^1,5 en todas las bisagras", () => {
    expect(attitudeJointDamping("tense", 75, 1.75, 1)).toEqual(ukeJointDamping(75, 1.75, 1));
    const r = relaxedJointDamping(75, 1.76);
    for (const v of Object.values(r)) expect(v).toBeCloseTo(1.6, 12);
    expect(relaxedJointDamping(150, 2.2)["left_knee"]).toBeCloseTo(1.6 * 2 * 1.25 ** 1.5, 12);
  });
});

describe("relaxed: energía mecánica (MuJoCo WASM real)", () => {
  it("cayendo desde de pie, la energía cinética + potencial nunca sube más de un 1 % sobre su máximo anterior", async () => {
    const mj = await getMujoco();
    const energies: number[] = [];
    const rec = await runScenario(scenario({}, { attitude: "relaxed" }), {
      engineVersion: "test",
      onSample: (m, d) => {
        // mj_energyPos / mj_energyVel del binding (sin activar el flag: se llaman directamente tras el mj_forward del muestreo).
        const model = m as Parameters<typeof mj.mj_energyPos>[0];
        const data = d as Parameters<typeof mj.mj_energyPos>[1];
        mj.mj_energyPos(model, data);
        mj.mj_energyVel(model, data);
        energies.push(data.energy[0]! + data.energy[1]!);
      },
    });
    let maxPrev = -Infinity;
    let worstRise = 0;
    for (const e of energies) {
      if (maxPrev > -Infinity) worstRise = Math.max(worstRise, (e - maxPrev) / Math.abs(maxPrev));
      maxPrev = Math.max(maxPrev, e);
    }
    const last = energies[energies.length - 1]!;
    console.log(`energía (${energies.length} muestras a 125 Hz): inicial ${energies[0]!.toFixed(1)} J, final ${last.toFixed(1)} J, mayor subida sobre el máximo anterior ${(100 * worstRise).toFixed(4)} %, resultado ${rec.summary.outcome}`);
    expect(energies.every(Number.isFinite)).toBe(true);
    expect(worstRise).toBeLessThanOrEqual(0.01);
    expect(last).toBeLessThan(energies[0]! * 0.5); // de verdad cae (pierde energía)
  }, 60000);
});

describe("péndulo: cuerpo rígido con tobillos libres (MuJoCo WASM real)", () => {
  it("de 5° a 45° tarda lo mismo que el péndulo físico equivalente (error < 10 %)", async () => {
    const mj = await getMujoco();
    // Modelo rigidizado: se quitan todas las bisagras (cuerpos soldados entre sí), se desactivan las colisiones y cada
    // tobillo se clava al mundo con una restricción de punto. Queda un solo grado de libertad: el giro de todo el cuerpo
    // alrededor del eje de los tobillos (Y). Sin rozamiento ni amortiguación.
    const xml = generateHumanoidMJCF({ mass_kg: 75, height_m: 1.76 })
      .replace(/<joint name="[^"]+" type="hinge"[^>]*\/>/g, "")
      .replace(/<geom /g, '<geom contype="0" conaffinity="0" ')
      .replace("</mujoco>", `<equality><connect body1="left_foot" anchor="0 0 0"/><connect body1="right_foot" anchor="0 0 0"/></equality></mujoco>`);
    const model = mj.MjModel.from_xml_string(xml);
    const data = new mj.MjData(model);
    try {
      expect(model.nv).toBe(6); // solo la raíz libre
      model.dof_damping.fill(0);
      (model.dof_armature as Float64Array).fill(0);
      mj.mj_forward(model, data);
      const foot = model.body("left_foot").id;
      const px = data.xpos[3 * foot]!, pz = data.xpos[3 * foot + 2]!;
      // Momento de inercia respecto al eje Y que pasa por los tobillos y masa total.
      let inertia = 0, mass = 0;
      for (let b = 1; b < model.nbody; b++) {
        const m = model.body_mass[b]!;
        mass += m;
        const rx = data.xipos[3 * b]! - px, rz = data.xipos[3 * b + 2]! - pz;
        const R = Array.from((data.ximat as Float64Array).subarray(9 * b, 9 * b + 9));
        // a = Y del mundo: a^T R diag(I) R^T a = sum_k I_k * (R[1][k])^2 (fila 1 de R = componentes de Y en los ejes principales)
        const I = [model.body_inertia[3 * b]!, model.body_inertia[3 * b + 1]!, model.body_inertia[3 * b + 2]!];
        inertia += I[0]! * R[3]! ** 2 + I[1]! * R[4]! ** 2 + I[2]! * R[5]! ** 2 + m * (rx * rx + rz * rz);
      }
      const com = (): [number, number] => [data.subtree_com[3]!, data.subtree_com[5]!]; // subtree de la raíz = todo el cuerpo
      const [cx0, cz0] = com();
      const dist = Math.hypot(cx0 - px, cz0 - pz);
      const theta0 = Math.atan2(cx0 - px, cz0 - pz);
      // Gira todo el cuerpo alrededor del eje Y que pasa por los tobillos hasta que la línea tobillo-CM forme 5° con la vertical.
      const phi = 5 * DEG - theta0;
      const dx = data.qpos[0]! - px, dz = data.qpos[2]! - pz;
      data.qpos[0] = px + Math.cos(phi) * dx + Math.sin(phi) * dz;
      data.qpos[2] = pz - Math.sin(phi) * dx + Math.cos(phi) * dz;
      data.qpos[3] = Math.cos(phi / 2); data.qpos[4] = 0; data.qpos[5] = Math.sin(phi / 2); data.qpos[6] = 0;
      mj.mj_forward(model, data);
      const angle = (): number => { const [x, z] = com(); return Math.atan2(x - px, z - pz); };
      expect(angle() / DEG).toBeCloseTo(5, 6);
      let tSim = NaN;
      for (let i = 1; i <= 3000; i++) {
        mj.mj_step(model, data);
        mj.mj_forward(model, data);
        if (angle() >= 45 * DEG) { tSim = i * model.opt.timestep; break; }
      }
      // Péndulo físico: I θ'' = m g d sin θ. RK4.
      const k = (mass * 9.81 * dist) / inertia;
      const f = (th: number): number => k * Math.sin(th);
      let th = 5 * DEG, om = 0, tRef = 0;
      const h = 1e-4;
      while (th < 45 * DEG && tRef < 10) {
        const a1 = f(th), k1 = om;
        const a2 = f(th + 0.5 * h * k1), k2 = om + 0.5 * h * a1;
        const a3 = f(th + 0.5 * h * k2), k3 = om + 0.5 * h * a2;
        const a4 = f(th + h * k3), k4 = om + h * a3;
        th += (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
        om += (h / 6) * (a1 + 2 * a2 + 2 * a3 + a4);
        tRef += h;
      }
      const err = Math.abs(tSim - tRef) / tRef;
      console.log(`péndulo: masa ${mass.toFixed(2)} kg, d ${dist.toFixed(3)} m, I ${inertia.toFixed(2)} kg·m², ω² ${k.toFixed(3)} 1/s²; t(45°) simulado ${tSim.toFixed(4)} s, péndulo físico ${tRef.toFixed(4)} s, error ${(100 * err).toFixed(2)} %`);
      expect(Number.isFinite(tSim)).toBe(true);
      expect(err).toBeLessThan(0.1);
    } finally {
      data.delete();
      model.delete();
    }
  });
});

interface Trace { latDevM: number; pelvisTurnDeg: number; outcome: string; direction: number | null }

async function traceOf(sc: Scenario): Promise<Trace> {
  let y0 = NaN;
  const yAxis0: number[] = [];
  let latDevM = 0, pelvisTurnDeg = 0;
  const rec = await runScenario(sc, {
    engineVersion: "test",
    onSample: (m, d) => {
      const data = d as { subtree_com: Float64Array; xmat: Float64Array };
      const pelvis = 1; // el primer cuerpo tras `world` es la pelvis
      const ya = [data.xmat[9 * pelvis + 1]!, data.xmat[9 * pelvis + 4]!, data.xmat[9 * pelvis + 7]!];
      const comY = data.subtree_com[3 * pelvis + 1]!;
      void m;
      if (Number.isNaN(y0)) { y0 = comY; yAxis0.push(...ya); }
      latDevM = Math.max(latDevM, Math.abs(comY - y0));
      const cos = ya[0]! * yAxis0[0]! + ya[1]! * yAxis0[1]! + ya[2]! * yAxis0[2]!;
      pelvisTurnDeg = Math.max(pelvisTurnDeg, Math.acos(Math.min(1, cos)) / DEG);
    },
  });
  return { latDevM, pelvisTurnDeg, outcome: rec.summary.outcome, direction: rec.summary.direction_deg };
}

describe("simetría y robustez del empujón sagital sobre Uke tense (MuJoCo WASM real)", () => {
  const IMPULSE = 150;
  it("simetría: 150 N·s hacia delante centrados en sternum -> desvío lateral del CM < 2 cm y giro de pelvis < 3°", async () => {
    const t = await traceOf(scenario(push(IMPULSE)));
    console.log(`simetría: resultado ${t.outcome}, desvío lateral máx del CM ${(100 * t.latDevM).toFixed(3)} cm, giro máx de pelvis ${t.pelvisTurnDeg.toFixed(3)}°`);
    expect(t.latDevM).toBeLessThan(0.02);
    expect(t.pelvisTurnDeg).toBeLessThan(3);
  }, 60000);

  it("robustez: el punto desplazado 1 cm en Y da el mismo veredicto y la misma dirección (±15°)", async () => {
    const a = await traceOf(scenario(push(IMPULSE)));
    const b = await traceOf(scenario(push(IMPULSE, [0, 0.01, 0])));
    const c = await traceOf(scenario(push(IMPULSE, [0, -0.01, 0])));
    console.log(`robustez: centrado ${a.outcome} ${a.direction?.toFixed(1)}°; +1 cm Y ${b.outcome} ${b.direction?.toFixed(1)}°, giro ${b.pelvisTurnDeg.toFixed(2)}°; -1 cm Y ${c.outcome} ${c.direction?.toFixed(1)}°`);
    for (const t of [b, c]) {
      expect(t.outcome).toBe(a.outcome);
      expect(t.direction).not.toBeNull();
      expect(Math.abs(((t.direction! - a.direction! + 540) % 360) - 180)).toBeLessThanOrEqual(15);
    }
  }, 120000);
});
