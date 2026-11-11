/**
 * Posturas iniciales de Uke: tabla de ángulos articulares (rad, convención de `rangeOfMotion.ts`) y colocación
 * cinemática previa al asentamiento de 0,5 s. Los tobillos no se escriben a mano: se resuelven para que ambos pies
 * queden planos. La separación de los pies sale de los ángulos de cadera (no es un parámetro aparte).
 * La pelvis se baja hasta que el pie más bajo toque el suelo. El centro de masas queda donde lo dejan los ángulos
 * (en `neutral`, sobre los tobillos); el equilibrio de tobillo toma como referencia su posición tras el asentamiento.
 */
import type { MujocoModule } from "../scenario/mujocoModule";
import type { UkeStance } from "../scenario/schema";
import { jointRangeRad } from "./rangeOfMotion";

/** Todas las bisagras del modelo generado, en el orden del MJCF. */
export const UKE_HINGE_JOINTS: readonly string[] = [
  "lumbar_pitch", "lumbar_roll", "thorax_pitch", "thorax_roll", "thorax_yaw", "neck_pitch", "neck_roll", "neck_yaw",
  ...["left", "right"].flatMap((s) => [
    `${s}_shoulder_pitch`, `${s}_shoulder_roll`, `${s}_shoulder_yaw`, `${s}_elbow`,
  ]),
  ...["left", "right"].flatMap((s) => [
    `${s}_hip_pitch`, `${s}_hip_roll`, `${s}_hip_yaw`, `${s}_knee`, `${s}_ankle_pitch`, `${s}_ankle_roll`,
  ]),
];

type Angles = Readonly<Record<string, number>>;

/** Par simétrico: mismo valor en ambos lados (para roll, el lado derecho lleva el signo contrario). */
function sym(name: string, value: number, mirrorSign = false): Record<string, number> {
  return { [`left_${name}`]: value, [`right_${name}`]: mirrorSign ? -value : value };
}

/** Ángulos objetivo por postura (lo que no aparece vale 0). Los tobillos se resuelven al colocar. */
export const STANCE_ANGLES: Readonly<Record<UkeStance, Angles>> = {
  neutral: {},
  // Rodillas algo flexionadas; brazos al frente con los codos doblados, como en una toma de agarre.
  natural_grip: {
    ...sym("hip_pitch", -0.125), ...sym("knee", -0.25),
    ...sym("shoulder_pitch", -0.9), ...sym("shoulder_roll", 0.1, true), ...sym("elbow", -1.3),
  },
  // Base ancha (cadera abierta), caderas y rodillas flexionadas, manos adelantadas.
  defensive: {
    ...sym("hip_pitch", -0.3), ...sym("knee", -0.6), ...sym("hip_roll", 0.22, true), lumbar_pitch: 0.07, thorax_pitch: 0.035,
    ...sym("shoulder_pitch", -0.7), ...sym("shoulder_roll", 0.25, true), ...sym("elbow", -1.5),
  },
  // Un pie adelantado ~0,3 m: pierna delantera con el muslo adelantado y la pierna vertical, la trasera algo más
  // flexionada, de modo que ambas caderas queden a la misma altura.
  staggered_left: {
    left_hip_pitch: -0.35, left_knee: -0.35, right_hip_pitch: 0.1, right_knee: -0.23,
    ...sym("shoulder_pitch", -0.5), ...sym("elbow", -0.9),
  },
  staggered_right: {
    right_hip_pitch: -0.35, right_knee: -0.35, left_hip_pitch: 0.1, left_knee: -0.23,
    ...sym("shoulder_pitch", -0.5), ...sym("elbow", -0.9),
  },
};

type Model = InstanceType<MujocoModule["MjModel"]>;
type Data = InstanceType<MujocoModule["MjData"]>;

export interface StancePlacement {
  /** Objetivo articular por nombre de bisagra (rad), ya con los tobillos resueltos. */
  readonly targets: ReadonlyMap<string, number>;
  /** Distancia entre los centros de los dos pies en el plano XY, en m. */
  readonly footSeparationM: number;
}

/**
 * Coloca el modelo en la postura (cinemática, sin dinámica): fija ángulos, resuelve los tobillos, apoya los pies. Deja `qvel` a cero. Hay que llamar a `mj_forward` después.
 */
export function applyStance(mj: MujocoModule, model: Model, data: Data, stance: UkeStance): StancePlacement {
  const qadr = (name: string): number => model.jnt_qposadr[model.jnt(name).id]!;
  const angles = STANCE_ANGLES[stance];
  for (const name of UKE_HINGE_JOINTS) {
    const [lo, hi] = jointRangeRad(name);
    const v = angles[name] ?? 0;
    data.qpos[qadr(name)] = Math.min(hi - 1e-3, Math.max(lo + 1e-3, v));
  }
  data.qvel.fill(0);

  // Pies planos: la componente vertical de los ejes X e Y del pie debe ser nula.
  for (let iter = 0; iter < 30; iter++) {
    mj.mj_kinematics(model, data);
    for (const side of ["left", "right"]) {
      const o = 9 * model.body(`${side}_foot`).id;
      const r20 = data.xmat[o + 6]!, r21 = data.xmat[o + 7]!;
      data.qpos[qadr(`${side}_ankle_pitch`)] += Math.asin(r20);
      data.qpos[qadr(`${side}_ankle_roll`)] -= Math.asin(r21);
    }
  }
  mj.mj_kinematics(model, data);
  mj.mj_comPos(model, data);

  const feet = ["left", "right"].map((s) => model.geom(`${s}_foot_geom`).id);
  let minBottom = Infinity;
  for (const g of feet) {
    const o = 9 * g;
    const reach = Math.abs(data.geom_xmat[o + 6]!) * model.geom_size[3 * g]! +
      Math.abs(data.geom_xmat[o + 7]!) * model.geom_size[3 * g + 1]! +
      Math.abs(data.geom_xmat[o + 8]!) * model.geom_size[3 * g + 2]!;
    minBottom = Math.min(minBottom, data.geom_xpos[3 * g + 2]! - reach);
  }
  data.qpos[2] += 0.0005 - minBottom;
  mj.mj_kinematics(model, data);
  mj.mj_comPos(model, data);

  const targets = new Map<string, number>();
  for (const name of UKE_HINGE_JOINTS) targets.set(name, data.qpos[qadr(name)]!);
  const separation = Math.hypot(
    data.geom_xpos[3 * feet[0]!]! - data.geom_xpos[3 * feet[1]!]!,
    data.geom_xpos[3 * feet[0]! + 1]! - data.geom_xpos[3 * feet[1]! + 1]!,
  );
  return { targets, footSeparationM: separation };
}
