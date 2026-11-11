import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { computeHumanoidSpec, generateHumanoidMJCF } from "../../src/uke/generate";
import { jointRangeRad } from "../../src/uke/rangeOfMotion";

const V2 = new URL("../../public/models/humanoid_step_v2.xml", import.meta.url);

const BODIES = [
  "pelvis", "abdomen", "chest", "head", "left_upperarm", "left_forearm", "right_upperarm", "right_forearm",
  "left_thigh", "left_shin", "left_foot", "right_thigh", "right_shin", "right_foot",
];
const ISSUE_SITES = [
  "nape", "sternum", "belt_front", "belt_back", "left_sleeve_elbow", "right_sleeve_elbow", "left_wrist", "right_wrist",
  "left_triceps", "right_triceps", "left_armpit", "right_armpit", "left_knee_back", "right_knee_back",
  "left_ankle", "right_ankle", "head_front", "head_side_left", "head_side_right", "abdomen_front",
];
const V2_SITES = [
  "pelvis_origin", "sternum_impact", "left_shoulder_impact", "left_forearm_impact", "left_sole",
  "right_shoulder_impact", "right_forearm_impact", "right_sole",
];

type Mj = Awaited<ReturnType<typeof loadMujoco>>;

async function load(mj: Mj, xml: string) {
  const model = mj.MjModel.from_xml_string(xml);
  const data = new mj.MjData(model);
  mj.mj_forward(model, data);
  return { model, data };
}

function vec(a: ArrayLike<number>, i: number): [number, number, number] {
  return [a[3 * i]!, a[3 * i + 1]!, a[3 * i + 2]!];
}

describe("generateHumanoidMJCF (MuJoCo WASM real)", () => {
  it("con 75 kg y 1,76 m reproduce humanoid_step_v2", async () => {
    const mj = await loadMujoco();
    const ref = await load(mj, await readFile(V2, "utf8"));
    const gen = await load(mj, generateHumanoidMJCF({ mass_kg: 75, height_m: 1.76 }));
    try {
      for (const b of BODIES) {
        const r = ref.model.body(b).id, g = gen.model.body(b).id;
        const mr = ref.model.body_mass[r]!, mg = gen.model.body_mass[g]!;
        expect(Math.abs(mg - mr) / mr, `masa de ${b}: v2 ${mr}, generado ${mg}`).toBeLessThan(0.01);
        const pr = vec(ref.data.xpos, r), pg = vec(gen.data.xpos, g);
        for (let k = 0; k < 3; k++) expect(Math.abs(pr[k]! - pg[k]!), `posición de ${b}`).toBeLessThan(1e-3);
      }
      // La masa del antebrazo + mano (excluida de la comparación por cuerpo) también coincide.
      expect(gen.model.body_mass[gen.model.body("left_forearm").id]).toBeCloseTo(1.3, 6);
      // Todas las articulaciones de v2 existen, con la misma posición de anclaje (±1 mm).
      const refJoints = jointNames(ref.model);
      const genJoints = jointNames(gen.model);
      for (const name of refJoints) {
        expect(genJoints, `falta la articulación ${name}`).toContain(name);
        const a = vec(ref.data.xanchor, ref.model.jnt(name).id), b = vec(gen.data.xanchor, gen.model.jnt(name).id);
        for (let k = 0; k < 3; k++) expect(Math.abs(a[k]! - b[k]!), `anclaje de ${name}`).toBeLessThan(1e-3);
      }
      expect(genJoints).toContain("neck_yaw");
      expect(genJoints.length).toBe(refJoints.length + 1);
      // Mismo orden relativo de las articulaciones de v2.
      expect(genJoints.filter((n) => refJoints.includes(n))).toEqual(refJoints);
      expect(gen.data.qpos[2]).toBeCloseTo(ref.data.qpos[2]!, 6);
      expect(gen.data.qpos[2]).toBeCloseTo(0.99, 6);
      expect(gen.model.nu).toBe(0);
      expect(hasGeom(gen.model, "left_hand_geom")).toBe(true);
    } finally {
      ref.data.delete(); ref.model.delete(); gen.data.delete(); gen.model.delete();
    }
  }, 30000);

  it("tiene los sitios de v2 y los de agarre e impacto de la issue", async () => {
    const mj = await loadMujoco();
    const gen = await load(mj, generateHumanoidMJCF({ mass_kg: 75, height_m: 1.76 }));
    try {
      for (const s of [...V2_SITES, ...ISSUE_SITES]) {
        expect(() => gen.model.site(s).id, `sitio ${s}`).not.toThrow();
        expect(gen.model.site(s).id).toBeGreaterThanOrEqual(0);
      }
    } finally {
      gen.data.delete(); gen.model.delete();
    }
  });

  const SIZES: Array<[number, number]> = [[55, 1.55], [75, 1.75], [100, 1.95], [150, 2.0]];
  for (const [mass, height] of SIZES) {
    it(`talla ${mass} kg / ${height} m: masa total, pies sin penetrar y centro de masas entre el 53 % y el 59 %`, async () => {
      const mj = await loadMujoco();
      const gen = await load(mj, generateHumanoidMJCF({ mass_kg: mass, height_m: height }));
      try {
        const total = Array.from(gen.model.body_mass as Float64Array).reduce((a, b) => a + b, 0);
        expect(Math.abs(total - mass) / mass).toBeLessThan(0.001);
        // Los pies tocan el suelo sin penetrar: fondo de la caja del pie.
        for (const side of ["left", "right"]) {
          const gid = gen.model.geom(`${side}_foot_geom`).id;
          const z = gen.data.geom_xpos[3 * gid + 2]! - gen.model.geom_size[3 * gid + 2]!;
          expect(z).toBeGreaterThan(-1e-6);
          expect(z).toBeLessThan(2e-3);
        }
        const comZ = gen.data.subtree_com[5]!;
        const ratio = comZ / height;
        console.log(`talla ${mass}/${height}: centro de masas ${(100 * ratio).toFixed(1)} % de la altura`);
        expect(ratio).toBeGreaterThan(0.53);
        expect(ratio).toBeLessThan(0.59);
        // El modelo se estabiliza sin explotar (0,5 s pasivos).
        for (let i = 0; i < 250; i++) mj.mj_step(gen.model, gen.data);
        expect(Array.from(gen.data.qpos).every(Number.isFinite)).toBe(true);
      } finally {
        gen.data.delete(); gen.model.delete();
      }
    }, 30000);

    it(`talla ${mass} kg / ${height} m: simetría izquierda/derecha`, async () => {
      const mj = await loadMujoco();
      const gen = await load(mj, generateHumanoidMJCF({ mass_kg: mass, height_m: height }));
      try {
        for (const part of ["upperarm", "forearm", "thigh", "shin", "foot"]) {
          const l = gen.model.body(`left_${part}`).id, r = gen.model.body(`right_${part}`).id;
          expect(gen.model.body_mass[l]).toBe(gen.model.body_mass[r]);
          const pl = vec(gen.data.xpos, l), pr = vec(gen.data.xpos, r);
          expect(pl[0]).toBeCloseTo(pr[0], 9);
          expect(pl[1]).toBeCloseTo(-pr[1], 9);
          expect(pl[2]).toBeCloseTo(pr[2], 9);
        }
        for (const s of ["wrist", "triceps", "armpit", "knee_back", "ankle", "sleeve_elbow"]) {
          const l = vec(gen.data.site_xpos, gen.model.site(`left_${s}`).id);
          const r = vec(gen.data.site_xpos, gen.model.site(`right_${s}`).id);
          expect(l[0]).toBeCloseTo(r[0], 9);
          expect(l[1]).toBeCloseTo(-r[1], 9);
          expect(l[2]).toBeCloseTo(r[2], 9);
        }
      } finally {
        gen.data.delete(); gen.model.delete();
      }
    }, 30000);
  }

  it("el hombro admite flexión completa (~3,1 rad) y los rangos de las articulaciones pares son espejo", () => {
    expect(jointRangeRad("left_shoulder_pitch")[0]).toBeLessThan(-3.0);
    expect(jointRangeRad("right_shoulder_pitch")[0]).toBeLessThan(-3.0);
    const l = jointRangeRad("left_hip_roll"), r = jointRangeRad("right_hip_roll");
    expect(l[0]).toBeCloseTo(-r[1], 12);
    expect(l[1]).toBeCloseTo(-r[0], 12);
    expect(() => jointRangeRad("no_existe")).toThrow();
  });

  it("segment_lengths_m manda sobre height_m y la masa sigue sumando", () => {
    const spec = computeHumanoidSpec({ mass_kg: 90, height_m: 1.8, segment_lengths_m: { thigh: 0.5 } });
    expect(spec.lengths_m.thigh).toBe(0.5);
    expect(spec.lengths_m.shin).toBeCloseTo(0.42 * 1.8 / 1.76, 9);
    expect(spec.totalMassKg).toBeCloseTo(90, 9);
  });

  it("con IMC > 27 el 60 % del exceso va al tronco", () => {
    const a = computeHumanoidSpec({ mass_kg: 150, height_m: 2.0 });
    const normal = 27 * 4; // 108 kg
    const excess = 150 - normal;
    const trunk = a.masses_kg.pelvis + a.masses_kg.abdomen + a.masses_kg.chest;
    const trunkIfUniform = 39 * normal / 75 + 39 / 75 * 0.4 * excess;
    expect(trunk).toBeCloseTo(trunkIfUniform + 0.6 * excess, 9);
    expect(a.masses_kg.pelvis).toBeGreaterThan(a.masses_kg.chest); // reparto 0,7 / 0,25 / 0,05 del exceso
  });

  it("parámetros fuera de rango o no finitos lanzan error", () => {
    expect(() => generateHumanoidMJCF({ mass_kg: 49, height_m: 1.76 })).toThrow(RangeError);
    expect(() => generateHumanoidMJCF({ mass_kg: 151, height_m: 1.76 })).toThrow(RangeError);
    expect(() => generateHumanoidMJCF({ mass_kg: 75, height_m: 1.4 })).toThrow(RangeError);
    expect(() => generateHumanoidMJCF({ mass_kg: 75, height_m: 2.1 })).toThrow(RangeError);
    expect(() => generateHumanoidMJCF({ mass_kg: NaN, height_m: 1.76 })).toThrow(RangeError);
    expect(() => generateHumanoidMJCF({ mass_kg: 75, height_m: 1.76, build: 2 })).toThrow(RangeError);
    expect(() => generateHumanoidMJCF({ mass_kg: 75, height_m: 1.76, segment_lengths_m: { thigh: 5 } })).toThrow(RangeError);
  });
});

/** Nombres de las articulaciones de un modelo, en orden, leídos del MJCF por la API de nombres. */
function jointNames(model: { njnt: number; names: ArrayLike<number>; name_jntadr: ArrayLike<number> }): string[] {
  const names = Uint8Array.from(model.names);
  const adr = model.name_jntadr;
  const dec = new TextDecoder();
  const out: string[] = [];
  for (let j = 0; j < model.njnt; j++) {
    let end = adr[j]!;
    while (names[end] !== 0) end++;
    out.push(dec.decode(names.subarray(adr[j], end)));
  }
  return out;
}

function hasGeom(model: { geom(name: string): { id: number } }, name: string): boolean {
  try { return model.geom(name).id >= 0; } catch { return false; }
}
