/**
 * Actitudes de Uke. Interfaz común `UkeController`; el ejecutor llama a `step` antes de cada `mj_step`
 * (y a `mj_subtreeVel` justo antes, como exige `stepActiveStanding`).
 *
 * `tense`: postura objetivo (PD en bisagras internas con ganancias escaladas, `gains.ts`) más el equilibrio
 * de tobillo del baseline. Reutiliza `stepActiveStanding` de `src/control/standingBaseline.ts` sin modificarlo:
 * se construye con `makeStandingController` y se sustituyen sus ganancias, objetivos y límites por los escalados.
 * Los pares van solo a `qfrc_applied` de bisagras internas; nunca a los 6 primeros grados de libertad.
 */
import type { JointPDSettings } from "../control/jointPD";
import {
  makeStandingController, type StandingController, type StandingModel,
  type StandingState,
} from "../control/standingBaseline";
import type { UkeAttitude, UkeFallMode, UkeStance } from "../scenario/schema";
import { createFallReflexController, type ReflexDriver } from "./fallReflex";
import { IMPLICIT_DAMPING_FRACTION, jointGroupOf, scaledGains, type UkeGains } from "./gains";
import { UKE_HINGE_JOINTS } from "./stances";

export interface UkeController {
  /** Escribe `qfrc_applied`. Requiere `mj_subtreeVel` ya llamado en este paso. */
  step(model: StandingModel, data: StandingState, t: number): void;
  status(): Record<string, unknown>;
  /**
   * Solo con reflejos de caída: estado compacto del reflejo para grabarlo por muestra (0 inactivo, 1 detectado,
   * 2 reaccionando, 3 aterrizado). Ausente sin reflejos: la grabación no cambia.
   */
  reflexState?(): number;
}

export interface UkeControllerParams {
  attitude: UkeAttitude;
  stance: UkeStance;
  mass_kg: number;
  height_m: number;
  strength: number;
  /** Objetivo articular por nombre de bisagra (de `applyStance`). */
  targets: ReadonlyMap<string, number>;
  /** Reflejos de caída (`fallReflex.ts`). Ausente o `none`: sin reflejos. */
  fall?: UkeFallMode;
  /** Latencia de reacción de los reflejos, en s (por defecto 0,15). */
  fall_latency_s?: number;
}

/** `tense` con el mando que usan los reflejos de caída para cambiar objetivos y ganancias (`fallReflex.ts`). */
export interface TenseController extends UkeController {
  readonly reflex: ReflexDriver;
}

export function createTenseController(
  params: UkeControllerParams,
  model: StandingModel,
  data: StandingState,
  gains: UkeGains = scaledGains(params.mass_kg, params.height_m, params.strength),
): TenseController {
  const base = makeStandingController(model, data);
  const settings = new Map<number, JointPDSettings>();
  const limitOf = new Map<number, number>();
  const nameOf = new Map<number, string>();
  const feedforwardLimit = new Map<number, number>();
  for (const name of UKE_HINGE_JOINTS) {
    const group = jointGroupOf(name);
    const g = gains.groups[group];
    const id = model.jnt(name).id;
    settings.set(id, {
      targetRad: params.targets.get(name) ?? 0,
      stiffnessNmPerRad: g.kp,
      dampingNmsPerRad: g.kd * (1 - IMPLICIT_DAMPING_FRACTION),
      maxTorqueNm: g.maxNm,
    });
    const dof = model.jnt_dofadr[id]!;
    limitOf.set(dof, group === "ankle" ? gains.ankleLimitNm : g.maxNm);
    if (group === "spine" || group === "neck" || group === "arm") feedforwardLimit.set(dof, g.maxNm);
    nameOf.set(dof, name);
  }
  const controller: StandingController = {
    ...base,
    gains: settings,
    kpX: gains.comKp, kdX: gains.comKd, kpY: gains.comKp, kdY: gains.comKd,
    ankleLimitNm: gains.ankleLimitNm,
  };
  /**
   * Equilibrio de tobillo por pie. `stepActiveStanding` reparte el par de equilibrio a partes iguales entre los dos
   * tobillos, lo que solo vale con los pies a la misma altura de apoyo (simétrico). Con un pie adelantado el pie
   * delantero y el trasero necesitan pares opuestos y el reparto igual levanta el delantero (medido: 100 y 150 kg
   * caían en `staggered_*`). Aquí cada pie recibe su fracción de la carga (proyección del centro de masas sobre la
   * línea entre pies) y el centro de presión de cada pie es el centro de su caja más el desplazamiento común que hace
   * que el centro de presión total coincida con el centro de masas. Con los pies simétricos es la fórmula del
   * baseline (`ankleBalanceTorques`) con el mismo reparto; el PD de postura y los límites son los de
   * `stepNeutralStandingPD` del baseline.
   */
  // PD de postura (la misma ley que `limitedJointTorque` del baseline, sin sus comprobaciones por paso ni asignaciones:
  // con 28 articulaciones x 5000 pasos eran la mayor parte del coste fuera de `mj_step`). Las articulaciones se
  // comprobaron como bisagras al construir los ajustes (`makeStandingController`).
  // `target` y `kpScale` son los mandos de los reflejos de caída (valen el objetivo de la postura y 1 mientras nadie los toque:
  // multiplicar por 1 es exacto, así que sin reflejos el par es bit a bit el de siempre).
  const pdList = Array.from(settings, ([id, g]) => ({
    dof: model.jnt_dofadr[id]!, qadr: model.jnt_qposadr[id]!, g, target: g.targetRad, kpScale: 1,
  }));
  const pdByName = new Map(UKE_HINGE_JOINTS.map((name) => [name, pdList.find((e) => e.dof === model.jnt_dofadr[model.jnt(name).id])!]));
  let balanceScale = 1;
  let feedforwardScale = 1;
  const applyPostureTorques = (d: StandingState): void => {
    d.qfrc_applied.fill(0);
    for (const { dof, qadr, g, target, kpScale } of pdList) {
      const tau = g.stiffnessNmPerRad * kpScale * (target - d.qpos[qadr]!) - g.dampingNmsPerRad * d.qvel[dof]!;
      d.qfrc_applied[dof] = Math.max(-g.maxTorqueNm, Math.min(g.maxTorqueNm, tau));
    }
  };
  const footGeoms = ["left", "right"].map((s) => model.geom(`${s}_foot_geom`).id);
  const clipAnkle = (v: number): number => Math.max(-gains.ankleLimitNm, Math.min(gains.ankleLimitNm, v));
  const stepBalance = (d: StandingState): void => {
    applyPostureTorques(d);
    const gx = (d as StandingState & { geom_xpos: ArrayLike<number> }).geom_xpos;
    const cx = d.subtree_com[3]!, cy = d.subtree_com[4]!;
    const vx = d.subtree_linvel[3]!, vy = d.subtree_linvel[4]!;
    const p = footGeoms.map((g) => [gx[3 * g]!, gx[3 * g + 1]!] as const);
    const dx = p[1]![0] - p[0]![0], dy = p[1]![1] - p[0]![1];
    const len2 = dx * dx + dy * dy;
    const sRight = len2 > 1e-9 ? Math.max(0, Math.min(1, ((cx - p[0]![0]) * dx + (cy - p[0]![1]) * dy) / len2)) : 0.5;
    const share = [1 - sRight, sRight];
    const dX = cx - (share[0]! * p[0]![0] + share[1]! * p[1]![0]);
    const dY = cy - (share[0]! * p[0]![1] + share[1]! * p[1]![1]);
    const fx = controller.kpX * (cx - controller.referenceComXY[0]) + controller.kdX * vx;
    const fy = controller.kpY * (cy - controller.referenceComXY[1]) + controller.kdY * vy;
    for (let i = 0; i < 2; i++) {
      const ankleX = d.xpos[3 * controller.footBodyIds[i]!]!;
      const ankleY = d.xpos[3 * controller.footBodyIds[i]! + 1]!;
      const pitch = balanceScale * (share[i]! * (controller.totalMassKg * 9.81 * (p[i]![0] + dX - ankleX) + fx));
      const roll = balanceScale * (-share[i]! * (controller.totalMassKg * 9.81 * (p[i]![1] + dY - ankleY) + fy));
      const pd = controller.anklePitchDofs[i]!, rd = controller.ankleRollDofs[i]!;
      d.qfrc_applied[pd] = clipAnkle(d.qfrc_applied[pd]! + pitch);
      d.qfrc_applied[rd] = clipAnkle(d.qfrc_applied[rd]! + roll);
    }
    for (let k = 0; k < 6; k++) {
      if (d.qfrc_applied[k] !== 0) throw new Error("Se detectó fuerza no permitida sobre pelvis libre");
    }
  };
  const ffDofs = Array.from(feedforwardLimit.keys());
  const ffLimits = Array.from(feedforwardLimit.values());
  const limDofs = Array.from(limitOf.keys());
  const limLimits = Array.from(limitOf.values());
  let maxRatio = 0;
  let maxNm = 0;
  let maxJoint = "";
  let saturated = 0;
  let steps = 0;
  const reflex: ReflexDriver = {
    tonic: true,
    target: (name) => pdByName.get(name)!.target,
    setTarget: (name, rad) => { pdByName.get(name)!.target = rad; },
    setStiffnessScale: (name, scale) => { pdByName.get(name)!.kpScale = scale; },
    setBalanceScale: (scale) => { balanceScale = scale; },
    setFeedforwardScale: (scale) => { feedforwardScale = scale; },
    maxTorqueNm: (name) => pdByName.get(name)!.g.maxTorqueNm,
  };
  return {
    reflex,
    step(_m, d): void {
      stepBalance(d);
      // Compensación estática de la gravedad de lo que cuelga de cada articulación (tono postural): sin ella el PD
      // cede bajo la carga de los brazos y el tronco se inclina. Solo en tronco, cuello y brazos: lo que cuelga de
      // ellos no toca el suelo. En las piernas el peso lo sostiene el contacto y compensarlo cargaría la pelvis
      // (medido: con 150 kg la pelvis se inclinaba 5° hacia atrás y Uke caía). El resultado respeta el par máximo.
      const bias = (d as StandingState & { qfrc_bias?: ArrayLike<number> }).qfrc_bias;
      if (bias) {
        for (let i = 0; i < ffDofs.length; i++) {
          const dof = ffDofs[i]!, limit = ffLimits[i]!;
          d.qfrc_applied[dof] = Math.max(-limit, Math.min(limit, d.qfrc_applied[dof]! + feedforwardScale * bias[dof]!));
        }
      }
      steps++;
      let sat = false;
      for (let i = 0; i < limDofs.length; i++) {
        const dof = limDofs[i]!;
        const tau = Math.abs(d.qfrc_applied[dof]!);
        const ratio = tau / limLimits[i]!;
        if (ratio > maxRatio) { maxRatio = ratio; maxNm = tau; maxJoint = nameOf.get(dof)!; }
        if (ratio >= 0.999) sat = true;
      }
      if (sat) saturated++;
    },
    status(): Record<string, unknown> {
      return {
        attitude: "tense",
        stance: params.stance,
        steps,
        max_torque_ratio: maxRatio,
        max_torque_nm: maxNm,
        max_torque_joint: maxJoint,
        saturated_steps: saturated,
        ankle_limit_nm: gains.ankleLimitNm,
      };
    },
  };
}

/**
 * `relaxed`: sin objetivos ni equilibrio. No escribe ningún par: solo actúan la amortiguación articular pasiva
 * (`damping` del MJCF, ver `damping.ts`) y los topes de rango del modelo. Cae como un cuerpo sin tono.
 */
export function createRelaxedController(params: UkeControllerParams): UkeController {
  let steps = 0;
  return {
    step(_m, d): void {
      d.qfrc_applied.fill(0);
      steps++;
    },
    status(): Record<string, unknown> {
      return { attitude: "relaxed", stance: params.stance, steps };
    },
  };
}

/**
 * `relaxed` con mando para los reflejos de caída: igual que `relaxed` (sin par alguno) mientras ninguna rigidez sea mayor que 0.
 * Los reflejos dan tono solo a las articulaciones de sus reglas (PD con las rigideces y los pares máximos de `gains.ts`).
 */
export function createRelaxedReflexController(
  params: UkeControllerParams, model: StandingModel,
  gains: UkeGains = scaledGains(params.mass_kg, params.height_m, params.strength),
): UkeController & { readonly reflex: ReflexDriver } {
  const entries = new Map(UKE_HINGE_JOINTS.map((name) => {
    const g = gains.groups[jointGroupOf(name)];
    const id = model.jnt(name).id;
    return [name, { dof: model.jnt_dofadr[id]!, qadr: model.jnt_qposadr[id]!, kp: g.kp, maxNm: g.maxNm, target: 0, kpScale: 0 }];
  }));
  const list = Array.from(entries.values());
  let steps = 0;
  const reflex: ReflexDriver = {
    tonic: false,
    target: (name) => entries.get(name)!.target,
    setTarget: (name, rad) => { entries.get(name)!.target = rad; },
    setStiffnessScale: (name, scale) => { entries.get(name)!.kpScale = scale; },
    setBalanceScale: () => { /* relaxed no tiene equilibrio activo */ },
    setFeedforwardScale: () => { /* ni compensación de la gravedad */ },
    maxTorqueNm: (name) => entries.get(name)!.maxNm,
  };
  return {
    reflex,
    step(_m, d): void {
      d.qfrc_applied.fill(0);
      for (const e of list) {
        if (e.kpScale <= 0) continue;
        const tau = e.kp * e.kpScale * (e.target - d.qpos[e.qadr]!);
        d.qfrc_applied[e.dof] = Math.max(-e.maxNm, Math.min(e.maxNm, tau));
      }
      steps++;
    },
    status(): Record<string, unknown> {
      return { attitude: "relaxed", stance: params.stance, steps };
    },
  };
}

export function createUkeController(
  params: UkeControllerParams, model: StandingModel, data: StandingState,
): UkeController {
  const reflexes = params.fall !== undefined && params.fall !== "none";
  switch (params.attitude) {
    case "tense": {
      const base = createTenseController(params, model, data);
      return reflexes ? createFallReflexController(base, base.reflex, params) : base;
    }
    case "relaxed": {
      if (!reflexes) return createRelaxedController(params);
      const base = createRelaxedReflexController(params, model);
      return createFallReflexController(base, base.reflex, params);
    }
  }
}
