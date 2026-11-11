import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { applyHingePD, type JointPDSettings } from "../../src/control/jointPD";
import { NEUTRAL_PD_GAINS } from "../../src/control/standingBaseline";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { generateHumanoidMJCF } from "../../src/uke/generate";
import { IMPLICIT_DAMPING_FRACTION, jointGroupOf, scaledGains, ukeJointDamping } from "../../src/uke/gains";
import { jointRangeRad } from "../../src/uke/rangeOfMotion";
import { UKE_HINGE_JOINTS } from "../../src/uke/stances";

const SIZES: ReadonlyArray<readonly [string, number, number]> = [
  ["55_155", 55, 1.55], ["75_175", 75, 1.75], ["100_195", 100, 1.95], ["150_200", 150, 2.0],
];

describe("scaledGains (ley de escalado)", () => {
  it("con 75 kg y 1,76 m devuelve la mitad de NEUTRAL_PD_GAINS (como makeStandingController) y el par completo", () => {
    const g = scaledGains(75, 1.76, 1);
    expect(g.groups.hip.kp).toBeCloseTo(NEUTRAL_PD_GAINS.hip.kp * 0.5, 9);
    expect(g.groups.hip.kd).toBeCloseTo(NEUTRAL_PD_GAINS.hip.kd * 0.5, 9);
    expect(g.groups.hip.maxNm).toBeCloseTo(NEUTRAL_PD_GAINS.hip.maxNm, 9);
    expect(g.ankleLimitNm).toBeCloseTo(160, 9);
  });

  it("kp ~ m·h, kd ~ m·h^1,5 y el par máximo ~ fuerza·m^(2/3)·h", () => {
    const a = scaledGains(75, 1.76, 1), b = scaledGains(150, 2.2, 1.5);
    const m = 2, h = 1.25;
    expect(b.groups.knee.kp / a.groups.knee.kp).toBeCloseTo(m * h, 9);
    expect(b.groups.knee.kd / a.groups.knee.kd).toBeCloseTo(m * h ** 1.5, 9);
    expect(b.groups.knee.maxNm / a.groups.knee.maxNm).toBeCloseTo(1.5 * m ** (2 / 3) * h, 9);
  });

  it("rechaza valores no positivos o no finitos", () => {
    expect(() => scaledGains(0, 1.76, 1)).toThrow(RangeError);
    expect(() => scaledGains(75, NaN, 1)).toThrow(RangeError);
    expect(() => scaledGains(75, 1.76, -1)).toThrow(RangeError);
  });
});

describe("estabilidad numérica por articulación (escalón de 0,2 rad, raíz fijada, MuJoCo WASM real)", () => {
  for (const [label, mass, height] of SIZES) {
    it(`${label}: ninguna articulación oscila más de 3 veces ni crece`, async () => {
      const mj = await loadMujoco();
      // Sin articulación libre y con la pelvis alta: las piernas cuelgan sin tocar el suelo.
      const xml = generateHumanoidMJCF({ mass_kg: mass, height_m: height }, { jointDampingByName: ukeJointDamping(mass, height, 1) })
        .replace('<freejoint name="pelvis_root"/>', "")
        // Sin articulación, MuJoCo funde la pelvis con el mundo y deja de filtrar sus contactos con sus hijos.
        .replace("</contact>", `<exclude body1="pelvis" body2="abdomen"/><exclude body1="pelvis" body2="left_thigh"/>
          <exclude body1="pelvis" body2="right_thigh"/></contact>`)
        .replace(/<body name="pelvis" pos="0 0 [\d.]+">/, '<body name="pelvis" pos="0 0 2.5">');
      const model = mj.MjModel.from_xml_string(xml);
      const data = new mj.MjData(model);
      const gains = scaledGains(mass, height, 1);
      const settings = (target: ReadonlyMap<number, number>): Map<number, JointPDSettings> => {
        const m = new Map<number, JointPDSettings>();
        for (const name of UKE_HINGE_JOINTS) {
          const g = gains.groups[jointGroupOf(name)];
          const id = model.jnt(name).id;
          m.set(id, { targetRad: target.get(id) ?? 0, stiffnessNmPerRad: g.kp, dampingNmsPerRad: g.kd * (1 - IMPLICIT_DAMPING_FRACTION), maxTorqueNm: g.maxNm });
        }
        return m;
      };
      let worstCrossings = 0;
      let worstGrowth = -Infinity;
      try {
        expect(model.nq).toBe(UKE_HINGE_JOINTS.length); // sin raíz libre
        for (const name of UKE_HINGE_JOINTS) {
          const id = model.jnt(name).id;
          const [lo, hi] = jointRangeRad(name);
          const target = hi >= 0.25 ? 0.2 : lo <= -0.25 ? -0.2 : (lo + hi) / 2 * 0; // dentro del rango
          expect(Math.abs(target), `${name} sin margen para el escalón`).toBeGreaterThan(0);
          mj.mj_resetData(model, data);
          const gainMap = settings(new Map([[id, target]]));
          const adr = model.jnt_qposadr[id]!;
          const n = 1500; // 3 s
          const q = new Float64Array(n);
          const dq = new Float64Array(n);
          for (let i = 0; i < n; i++) {
            data.qfrc_applied.fill(0);
            applyHingePD(model, data, gainMap);
            mj.mj_step(model, data);
            q[i] = data.qpos[adr]!;
            dq[i] = data.qvel[model.jnt_dofadr[id]!]!;
          }
          expect(Array.from(q).every(Number.isFinite), `${name} no finito`).toBe(true);
          const qf = q.slice(n - 125).reduce((a, b) => a + b, 0) / 125; // media del último 0,25 s
          let crossings = 0;
          let sign = 0;
          for (let i = 0; i < n; i++) {
            const e = q[i]! - qf;
            if (Math.abs(e) < 0.002) continue; // 1 % del escalón
            const s = Math.sign(e);
            if (sign !== 0 && s !== sign) crossings++;
            sign = s;
          }
          // Crecimiento: la velocidad articular máxima de la ventana tardía (2,25-3 s) no supera la de la temprana (0,5-1,5 s).
          const peakSpeed = (a: number, b: number): number => {
            let m = 0;
            for (let i = a; i < b; i++) m = Math.max(m, Math.abs(dq[i]!));
            return m;
          };
          const growth = peakSpeed(1125, 1500) - peakSpeed(250, 750);
          worstCrossings = Math.max(worstCrossings, crossings);
          worstGrowth = Math.max(worstGrowth, growth);
          expect(crossings, `${name}: cruces de la respuesta`).toBeLessThanOrEqual(3);
          expect(growth, `${name}: crecimiento de la oscilación`).toBeLessThanOrEqual(1e-4); // rad/s
        }
        console.log(`estabilidad ${label}: máximo de cruces ${worstCrossings}, crecimiento máximo de la velocidad pico ${worstGrowth.toExponential(2)} rad/s`);
      } finally {
        data.delete();
        model.delete();
      }
    }, 120000);
  }
});

describe("Uke tense de pie 10 s (escenarios uke_stand_<talla>_neutral, MuJoCo WASM real)", () => {
  for (const [label] of SIZES) {
    it(`${label}: tronco < 3°, pelvis > 90 % de su altura, sin fuerza en la raíz y asentamiento < 2 cm/s`, async () => {
      const file = `uke_stand_${label}_neutral.json`;
      const scenario = validateScenario(
        JSON.parse(readFileSync(new URL(`../../scenarios/${file}`, import.meta.url), "utf8")) as unknown,
        { fileName: file },
      );
      const rec = await runScenario(scenario, { engineVersion: "test" });
      const u = rec.summary.uke!;
      const ratio = u.min_pelvis_height_m / u.initial_pelvis_height_m;
      const com = u.initial_com_height_m;
      const actor = scenario.actors[0]!;
      console.log(
        `${label}: masa ${actor.mass_kg} kg, CM ${(100 * com / actor.height_m).toFixed(1)} % de la altura, tronco máx ${u.max_trunk_tilt_deg.toFixed(2)}°, ` +
        `pelvis mín ${(100 * ratio).toFixed(2)} % (${u.min_pelvis_height_m.toFixed(4)} m), par máx ${Number(u.controller.max_torque_nm).toFixed(1)} N·m en ` +
        `${String(u.controller.max_torque_joint)} (límite ${Number(u.controller.ankle_limit_nm).toFixed(1)} N·m, ratio ${Number(u.controller.max_torque_ratio).toFixed(2)}), ` +
        `asentamiento ${(100 * u.settle_com_speed_mps).toFixed(2)} cm/s`,
      );
      expect(rec.summary.outcome).toBe("standing");
      expect(u.max_trunk_tilt_deg).toBeLessThan(3);
      expect(ratio).toBeGreaterThan(0.9);
      expect(u.max_root_generalized_force).toBe(0); // ninguna muestra con fuerza generalizada en los 6 DOF de la raíz
      expect(u.settle_com_speed_mps).toBeLessThan(0.02);
      // Ningún par supera su límite (el controlador recorta y registra la razón).
      expect(Number(u.controller.max_torque_ratio)).toBeLessThanOrEqual(1 + 1e-9);
    }, 60000);
  }
});
