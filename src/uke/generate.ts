/**
 * Generador del MJCF del humanoide paramétrico (masa, altura, complexión). Con 75 kg y 1,76 m reproduce
 * `public/models/humanoid_step_v2.xml` (jerarquía, nombres, masas, articulaciones) y añade mano, `neck_yaw`,
 * rangos articulares de `rangeOfMotion.ts` y sitios de agarre e impacto. Reglas y aproximaciones: `docs/UKE_MODEL.md`.
 * Sin dependencias de `src/control/` ni de `src/physics/`.
 */
import { worldOptionTag } from "../scenario/worldOptions";
import { rangeAttr } from "./rangeOfMotion";

export const REFERENCE_MASS_KG = 75;
export const REFERENCE_HEIGHT_M = 1.76;
export const MASS_RANGE_KG: readonly [number, number] = [50, 150];
export const HEIGHT_RANGE_M: readonly [number, number] = [1.5, 2.05];
/** Índice de masa corporal a partir del cual el exceso de masa se reparte hacia el tronco. */
export const BMI_THRESHOLD = 27;
/** Fracción del exceso de masa que va a pelvis, abdomen y pecho (regla de partida). */
export const DEFAULT_TRUNK_EXCESS_FRACTION = 0.6;

/** Longitudes de v2 en m. Definiciones en `docs/UKE_MODEL.md`. */
export const V2_SEGMENT_LENGTHS_M = {
  thigh: 0.42,
  shin: 0.42,
  /** Longitud total de la caja del pie. */
  foot: 0.27,
  upper_arm: 0.29,
  forearm: 0.26,
  /** Del origen de la pelvis a la articulación del cuello (0,11 + 0,23 + 0,24). */
  trunk: 0.58,
  /** De la articulación del cuello al centro de la cabeza. */
  neck: 0.065,
} as const;

export type SegmentLengths = { -readonly [K in keyof typeof V2_SEGMENT_LENGTHS_M]: number };

export interface HumanoidParams {
  mass_kg: number;
  height_m: number;
  /** Fracción (0 a 1) del exceso de masa (IMC > 27) que va al tronco. Por defecto 0,6. */
  build?: number;
  /** Longitudes explícitas (m); mandan sobre `height_m`. */
  segment_lengths_m?: Partial<SegmentLengths>;
}

export interface GenerateOptions {
  /** Rozamiento deslizante de las geometrías y del suelo. Por defecto 1,0 (v2). */
  friction?: number;
  /** XML adicional dentro de `<worldbody>`, tras el suelo (props, etc.). */
  extraWorldXml?: string;
  /** Atributos extra de `<option>`. */
  optionAttrs?: string;
  modelName?: string;
  /** Amortiguación de las articulaciones (v2: 1,6). */
  jointDamping?: number;
  /** Amortiguación por articulación (N·m·s/rad); manda sobre `jointDamping`. Se integra de forma implícita. */
  jointDampingByName?: Readonly<Record<string, number>>;
  /** Armadura de las articulaciones (v2: 0,015). */
  jointArmature?: number;
}

/** Masas de v2 por segmento, en kg (una unidad; los pares se duplican). */
const V2_MASS = {
  pelvis: 14, abdomen: 9, chest: 16, head: 5,
  upperarm: 2.1, forearm: 0.85, hand: 0.45, thigh: 7.5, shin: 3.5, foot: 1.1,
} as const;
type Seg = keyof typeof V2_MASS;
const SEGS = Object.keys(V2_MASS) as Seg[];
const PAIRED: ReadonlySet<Seg> = new Set(["upperarm", "forearm", "hand", "thigh", "shin", "foot"]);
const TRUNK: readonly Seg[] = ["pelvis", "abdomen", "chest"];
/**
 * Reparto del exceso de masa del tronco entre pelvis, abdomen y pecho. Repartirlo en proporción a las masas de v2
 * (14:9:16) subía el centro de masas al 60 % de la altura con 150 kg / 2,00 m (fuera del 53-59 % exigido); la grasa
 * se acumula sobre todo en abdomen y caderas, así que se reparte 0,7 / 0,25 / 0,05 (medido con 150 kg / 2,00 m: 60,0 % en proporción a v2, 59,5 % con 0,4/0,4/0,2, 59,2 % con 0,5/0,35/0,15).
 */
export const TRUNK_EXCESS_WEIGHTS: Readonly<Record<"pelvis" | "abdomen" | "chest", number>> = {
  pelvis: 0.7, abdomen: 0.25, chest: 0.05,
};

export interface HumanoidSpec {
  masses_kg: Record<Seg, number>;
  totalMassKg: number;
  /** Factor de grosor `sqrt((m/h)/(75/1.76))`. */
  thickness: number;
  /** Factor de complexión del tronco. */
  trunkBuild: number;
  lengths_m: SegmentLengths;
  pelvisHeightM: number;
}

function check(cond: boolean, msg: string): void {
  if (!cond) throw new RangeError(msg);
}

/** Masas por segmento y factores geométricos. Lanza `RangeError` con parámetros fuera de rango. */
export function computeHumanoidSpec(p: HumanoidParams): HumanoidSpec {
  const { mass_kg: m, height_m: h } = p;
  check(Number.isFinite(m) && m >= MASS_RANGE_KG[0] && m <= MASS_RANGE_KG[1],
    `mass_kg debe estar entre ${MASS_RANGE_KG[0]} y ${MASS_RANGE_KG[1]} kg, no ${m}`);
  check(Number.isFinite(h) && h >= HEIGHT_RANGE_M[0] && h <= HEIGHT_RANGE_M[1],
    `height_m debe estar entre ${HEIGHT_RANGE_M[0]} y ${HEIGHT_RANGE_M[1]} m, no ${h}`);
  const fraction = p.build ?? DEFAULT_TRUNK_EXCESS_FRACTION;
  check(Number.isFinite(fraction) && fraction >= 0 && fraction <= 1, `build debe estar entre 0 y 1, no ${fraction}`);

  const hs = h / REFERENCE_HEIGHT_M;
  const lengths: SegmentLengths = { ...V2_SEGMENT_LENGTHS_M };
  for (const k of Object.keys(V2_SEGMENT_LENGTHS_M) as Array<keyof SegmentLengths>) {
    const v = p.segment_lengths_m?.[k];
    if (v === undefined) {
      lengths[k] = V2_SEGMENT_LENGTHS_M[k] * hs;
    } else {
      const ratio = v / V2_SEGMENT_LENGTHS_M[k];
      check(Number.isFinite(v) && ratio >= 0.5 && ratio <= 2,
        `segment_lengths_m.${k} debe estar entre 0,5 y 2 veces el valor de v2 (${V2_SEGMENT_LENGTHS_M[k]} m), no ${v}`);
      lengths[k] = v;
    }
  }

  // Un cuerpo (no los pares) cuenta una vez; la suma total usa la multiplicidad.
  const mult = (s: Seg): number => (PAIRED.has(s) ? 2 : 1);
  const baseTotal = SEGS.reduce((a, s) => a + V2_MASS[s] * mult(s), 0);
  const bmi = m / (h * h);
  const normalMass = bmi > BMI_THRESHOLD ? BMI_THRESHOLD * h * h : m;
  const excess = m - normalMass;
  const trunkBase = TRUNK.reduce((a, s) => a + V2_MASS[s], 0);
  const masses = {} as Record<Seg, number>;
  for (const s of SEGS) {
    const share = V2_MASS[s] * mult(s) / baseTotal;
    let v = (V2_MASS[s] * normalMass) / baseTotal;
    if (excess > 0) {
      v += (1 - fraction) * excess * share / mult(s);
      if (TRUNK.includes(s)) v += fraction * excess * TRUNK_EXCESS_WEIGHTS[s as "pelvis" | "abdomen" | "chest"];
    }
    masses[s] = v;
  }
  const total = SEGS.reduce((a, s) => a + masses[s] * mult(s), 0);
  const trunkActual = TRUNK.reduce((a, s) => a + masses[s], 0);
  const trunkUniform = trunkBase * m / baseTotal;
  const thickness = Math.sqrt((m / h) / (REFERENCE_MASS_KG / REFERENCE_HEIGHT_M));
  return {
    masses_kg: masses,
    totalMassKg: total,
    thickness,
    trunkBuild: Math.sqrt(trunkActual / trunkUniform),
    lengths_m: lengths,
    pelvisHeightM: 0.08 * hs + lengths.thigh + lengths.shin + 0.07 * hs,
  };
}

/** Formato corto y estable de un número. */
function f(x: number): string {
  return String(Number(x.toFixed(6)));
}

const RGBA = {
  pelvis: "0.50 0.72 0.86 1", abdomen: "0.38 0.65 0.82 1", chest: "0.46 0.77 0.92 1", head: "0.79 0.85 0.88 1",
  upperarm: "0.53 0.74 0.84 1", forearm: "0.68 0.8 0.86 1", hand: "0.79 0.85 0.88 1", thigh: "0.39 0.62 0.78 1",
  shin: "0.51 0.72 0.85 1", foot: "0.28 0.77 0.81 1",
};
const SITE_IMPACT = `rgba="1 0.5 0.1 1"`;
const SITE_GRIP = `rgba="0.9 0.2 0.6 1"`;

/** Devuelve el MJCF completo (suelo incluido) del humanoide con 33+1 grados de libertad de bisagra y raíz libre. */
export function generateHumanoidMJCF(params: HumanoidParams, options: GenerateOptions = {}): string {
  const spec = computeHumanoidSpec(params);
  const { masses_kg: M, lengths_m: L } = spec;
  const hs = params.height_m / REFERENCE_HEIGHT_M;
  const tf = spec.thickness;
  const tt = tf * spec.trunkBuild; // grosor del tronco
  const st = L.trunk / V2_SEGMENT_LENGTHS_M.trunk;
  const sn = L.neck / V2_SEGMENT_LENGTHS_M.neck;
  const sFoot = L.foot / V2_SEGMENT_LENGTHS_M.foot;
  const friction = options.friction ?? 1.0;
  const fr = `${f(friction)} 0.025 0.002`;

  // Pelvis y tronco.
  const pelvisA = [0.14 * tt, 0.15 * tt, 0.11 * st] as const;
  const abR = 0.12 * tt;
  const abOff = 0.11 * st, abLen = 0.23 * st, chOff = 0.23 * st;
  const chA = [0.155 * tt, 0.215 * tt, 0.18 * st] as const;
  const chZ = 0.105 * st;
  const headOff = 0.24 * st;
  const headA = [0.105 * hs, 0.09 * hs, 0.13 * hs] as const;
  const headZ = 0.065 * sn;
  const shoulderY = (0.225 / 0.215) * chA[1];
  const shoulderZ = 0.145 * st;
  // Brazos.
  const uaLen = L.upper_arm, faLen = L.forearm;
  const uaR = 0.048 * tf, faR = 0.041 * tf, handR = 0.04 * tf;
  const handZ = faLen + 0.03 * hs;
  // Piernas.
  const hipY = 0.7 * pelvisA[1];
  const hipZ = -0.08 * hs;
  const thR = 0.072 * tf, shR = 0.053 * tf;
  const footA = [0.135 * sFoot, 0.055 * tf, 0.035 * hs] as const;
  const footPos = [0.065 * sFoot, 0, -0.035 * hs] as const;

  const ell = (a: number, c: number, dz: number): number => a * Math.sqrt(Math.max(0, 1 - (dz / c) ** 2));
  const rng = (j: string): string => {
    const d = options.jointDampingByName?.[j];
    return `range="${rangeAttr(j)}"${d === undefined ? "" : ` damping="${f(d)}"`}`;
  };
  const site = (name: string, x: number, y: number, z: number, kind: "impact" | "grip" | "ref", size = 0.015): string => {
    const color = kind === "impact" ? SITE_IMPACT : kind === "grip" ? SITE_GRIP : `rgba="0.96 0.77 0.3 1"`;
    return `<site name="${name}" pos="${f(x)} ${f(y)} ${f(z)}" size="${f(size)}" ${color}/>`;
  };

  // Posiciones de los sitios sobre la superficie.
  const sternumX = ell(chA[0], chA[2], 0.015 * st) + 0.005 * tt;
  const sternumZ = chZ + 0.015 * st;
  const headFrontDz = 0.04 * hs;
  const headFrontX = 0.005 * hs + ell(headA[0], headA[2], headFrontDz);
  const headFrontZ = headZ + headFrontDz;

  const arm = (side: "left" | "right", sg: 1 | -1): string => `
          <body name="${side}_upperarm" pos="0 ${f(sg * shoulderY)} ${f(shoulderZ)}">
            <joint name="${side}_shoulder_pitch" type="hinge" axis="0 1 0" ${rng(`${side}_shoulder_pitch`)}/>
            <joint name="${side}_shoulder_roll" type="hinge" axis="1 0 0" ${rng(`${side}_shoulder_roll`)}/>
            <joint name="${side}_shoulder_yaw" type="hinge" axis="0 0 1" ${rng(`${side}_shoulder_yaw`)}/>
            ${site(`${side}_shoulder_impact`, 0, 0, -0.04 * hs, "impact")}
            ${site(`${side}_triceps`, -uaR, 0, -0.55 * uaLen, "grip")}
            ${site(`${side}_armpit`, 0, -sg * uaR, -0.06 * hs, "grip")}
            <geom name="${side}_upperarm_geom" type="capsule" fromto="0 0 0 0 0 ${f(-uaLen)}" size="${f(uaR)}" mass="${f(M.upperarm)}" rgba="${RGBA.upperarm}"/>
            <body name="${side}_forearm" pos="0 0 ${f(-uaLen)}">
              <joint name="${side}_elbow" type="hinge" axis="0 1 0" ${rng(`${side}_elbow`)}/>
              ${site(`${side}_forearm_impact`, 0, 0, -0.5 * faLen, "impact")}
              ${site(`${side}_sleeve_elbow`, 0, sg * faR, -0.04 * hs, "grip")}
              ${site(`${side}_wrist`, 0, 0, -faLen, "grip")}
              <geom name="${side}_forearm_geom" type="capsule" fromto="0 0 0 0 0 ${f(-faLen)}" size="${f(faR)}" mass="${f(M.forearm)}" rgba="${RGBA.forearm}"/>
              <geom name="${side}_hand_geom" type="sphere" pos="0 0 ${f(-handZ)}" size="${f(handR)}" mass="${f(M.hand)}" rgba="${RGBA.hand}"/>
            </body>
          </body>`;

  const leg = (side: "left" | "right", sg: 1 | -1): string => `
      <body name="${side}_thigh" pos="0 ${f(sg * hipY)} ${f(hipZ)}">
        <joint name="${side}_hip_pitch" type="hinge" axis="0 1 0" ${rng(`${side}_hip_pitch`)}/>
        <joint name="${side}_hip_roll" type="hinge" axis="1 0 0" ${rng(`${side}_hip_roll`)}/>
        <joint name="${side}_hip_yaw" type="hinge" axis="0 0 1" ${rng(`${side}_hip_yaw`)}/>
        <geom name="${side}_thigh_geom" type="capsule" fromto="0 0 0 0 0 ${f(-L.thigh)}" size="${f(thR)}" mass="${f(M.thigh)}" rgba="${RGBA.thigh}"/>
        <body name="${side}_shin" pos="0 0 ${f(-L.thigh)}">
          <joint name="${side}_knee" type="hinge" axis="0 -1 0" ${rng(`${side}_knee`)}/>
          ${site(`${side}_knee_back`, -shR, 0, -0.05 * hs, "grip")}
          <geom name="${side}_shin_geom" type="capsule" fromto="0 0 0 0 0 ${f(-L.shin)}" size="${f(shR)}" mass="${f(M.shin)}" rgba="${RGBA.shin}"/>
          <body name="${side}_foot" pos="0 0 ${f(-L.shin)}">
            <joint name="${side}_ankle_pitch" type="hinge" axis="0 1 0" ${rng(`${side}_ankle_pitch`)}/>
            <joint name="${side}_ankle_roll" type="hinge" axis="1 0 0" ${rng(`${side}_ankle_roll`)}/>
            <geom name="${side}_foot_geom" type="box" pos="${f(footPos[0])} 0 ${f(footPos[2])}" size="${f(footA[0])} ${f(footA[1])} ${f(footA[2])}" mass="${f(M.foot)}" rgba="${RGBA.foot}"/>
            ${site(`${side}_sole`, 0.065 * sFoot, 0, -0.068 * hs, "ref", 0.012)}
            ${site(`${side}_ankle`, 0, 0, 0.01 * hs, "grip")}
          </body>
        </body>
      </body>`;

  const name = options.modelName ?? "humanoid_generated";
  return `<mujoco model="${name}">
  <!-- Generado por src/uke/generate.ts: ${f(params.mass_kg)} kg, ${f(params.height_m)} m. Reproduce humanoid_step_v2 con 75 kg y 1.76 m. -->
  <compiler angle="radian"/>
  ${worldOptionTag(options.optionAttrs)}
  <default>
    <joint limited="true" damping="${f(options.jointDamping ?? 1.6)}" armature="${f(options.jointArmature ?? 0.015)}"/>
    <geom friction="${fr}" condim="3" solref="0.015 1" solimp="0.9 0.95 0.001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" friction="${fr}" rgba="0.18 0.26 0.32 1"/>
${options.extraWorldXml ?? ""}
    <body name="pelvis" pos="0 0 ${f(spec.pelvisHeightM)}">
      <freejoint name="pelvis_root"/>
      <geom name="pelvis_geom" type="ellipsoid" size="${f(pelvisA[0])} ${f(pelvisA[1])} ${f(pelvisA[2])}" mass="${f(M.pelvis)}" rgba="${RGBA.pelvis}"/>
      <site name="pelvis_origin" pos="0 0 0" size="0.012" rgba="0 1 0 1"/>

      <body name="abdomen" pos="0 0 ${f(abOff)}">
        <joint name="lumbar_pitch" type="hinge" axis="0 1 0" ${rng("lumbar_pitch")}/>
        <joint name="lumbar_roll" type="hinge" axis="1 0 0" ${rng("lumbar_roll")}/>
        <geom name="abdomen_geom" type="capsule" fromto="0 0 0 0 0 ${f(abLen)}" size="${f(abR)}" mass="${f(M.abdomen)}" rgba="${RGBA.abdomen}"/>
        ${site("belt_front", abR, 0, 0.04 * st, "grip")}
        ${site("belt_back", -abR, 0, 0.04 * st, "grip")}
        ${site("abdomen_front", abR, 0, 0.5 * abLen, "impact")}

        <body name="chest" pos="0 0 ${f(chOff)}">
          <joint name="thorax_pitch" type="hinge" axis="0 1 0" ${rng("thorax_pitch")}/>
          <joint name="thorax_roll" type="hinge" axis="1 0 0" ${rng("thorax_roll")}/>
          <joint name="thorax_yaw" type="hinge" axis="0 0 1" ${rng("thorax_yaw")}/>
          <geom name="chest_geom" type="ellipsoid" pos="0 0 ${f(chZ)}" size="${f(chA[0])} ${f(chA[1])} ${f(chA[2])}" mass="${f(M.chest)}" rgba="${RGBA.chest}"/>
          ${site("sternum_impact", sternumX, 0, sternumZ, "impact", 0.022)}
          ${site("sternum", sternumX, 0, sternumZ, "impact", 0.022)}

          <body name="head" pos="0 0 ${f(headOff)}">
            <joint name="neck_pitch" type="hinge" axis="0 1 0" ${rng("neck_pitch")}/>
            <joint name="neck_roll" type="hinge" axis="1 0 0" ${rng("neck_roll")}/>
            <joint name="neck_yaw" type="hinge" axis="0 0 1" ${rng("neck_yaw")}/>
            <geom name="head_geom" type="ellipsoid" pos="${f(0.005 * hs)} 0 ${f(headZ)}" size="${f(headA[0])} ${f(headA[1])} ${f(headA[2])}" mass="${f(M.head)}" rgba="${RGBA.head}"/>
            ${site("nape", -0.06 * hs, 0, 0.02 * hs, "grip")}
            ${site("head_front", headFrontX, 0, headFrontZ, "impact")}
            ${site("head_side_left", 0.005 * hs, headA[1], headZ, "impact")}
            ${site("head_side_right", 0.005 * hs, -headA[1], headZ, "impact")}
          </body>
${arm("left", 1)}
${arm("right", -1)}
        </body>
      </body>

${leg("left", 1)}
${leg("right", -1)}
    </body>
  </worldbody>
  <!-- Segmentos vecinos cuyas geometrías se solapan en las tallas gruesas (cápsula del abdomen contra el muslo: 10 kN
       en 150 kg con las rodillas flexionadas): no colisionan entre sí. -->
  <contact>
    <exclude body1="abdomen" body2="left_thigh"/>
    <exclude body1="abdomen" body2="right_thigh"/>
    <exclude body1="pelvis" body2="left_shin"/>
    <exclude body1="pelvis" body2="right_shin"/>
  </contact>
</mujoco>
`;
}
