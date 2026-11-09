import { applyHingePD, type JointPDSettings, type MuJoCoJointIndexView, type MuJoCoJointStateView } from "./jointPD";

/** Internal-joint baseline posture controller. This is NOT an approved
 * standing controller: its 10s stability must be measured in physics.
 * No root torque, no pelvis anchor, no motion capture and no RL.
 */
export interface NamedHingeModel extends MuJoCoJointIndexView {
  jnt(name: string): { id: number };
}

export type JointGroup = "spine" | "hip" | "knee" | "ankle" | "arm" | "neck";

export const NEUTRAL_PD_GAINS: Readonly<Record<JointGroup, Readonly<{
  kp: number; kd: number; maxNm: number;
}>>> = {
  spine: { kp: 450, kd: 48, maxNm: 160 },
  hip: { kp: 680, kd: 62, maxNm: 260 },
  knee: { kp: 540, kd: 52, maxNm: 220 },
  ankle: { kp: 700, kd: 64, maxNm: 170 },
  arm: { kp: 75, kd: 14, maxNm: 70 },
  neck: { kp: 60, kd: 12, maxNm: 40 },
};

/** Explicit joint whitelist prevents any accidental force on the root. */
export const NEUTRAL_JOINT_GROUPS: Readonly<Record<JointGroup, readonly string[]>> = {
  spine: ["lumbar_pitch", "lumbar_roll", "thorax_pitch", "thorax_roll", "thorax_yaw"],
  hip: ["left_hip_pitch", "left_hip_roll", "left_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_hip_yaw"],
  knee: ["left_knee", "right_knee"],
  ankle: ["left_ankle_pitch", "left_ankle_roll", "right_ankle_pitch", "right_ankle_roll"],
  arm: ["left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow"],
  neck: ["neck_pitch", "neck_roll"],
};

export function makeNeutralStandingPD(model: NamedHingeModel): ReadonlyMap<number, JointPDSettings> {
  const result = new Map<number, JointPDSettings>();
  for (const group of Object.keys(NEUTRAL_JOINT_GROUPS) as JointGroup[]) {
    const g = NEUTRAL_PD_GAINS[group];
    for (const name of NEUTRAL_JOINT_GROUPS[group]) {
      const id = model.jnt(name).id;
      if (!Number.isSafeInteger(id) || id <= 0 || id >= model.njnt || model.jnt_type[id] !== 3) {
        throw new TypeError("Joint no es una bisagra interna: " + name);
      }
      result.set(id, {
        targetRad: 0,
        stiffnessNmPerRad: g.kp,
        dampingNmsPerRad: g.kd,
        maxTorqueNm: g.maxNm,
      });
    }
  }
  return result;
}

export function stepNeutralStandingPD(
  model: NamedHingeModel,
  data: MuJoCoJointStateView,
  gains: ReadonlyMap<number, JointPDSettings>,
): number {
  data.qfrc_applied.fill(0);
  return applyHingePD(model, data, gains);
}


/** Physics-grounded standing controller for synthetic humanoid v1.
 * Sensor arrays are refreshed by calling mj_subtreeVel(model,data) before
 * this controller, and every MuJoCo integration step follows this call.
 */
export interface StandingModel extends NamedHingeModel {
  geom(name: string): { id: number };
  readonly geom_bodyid: ArrayLike<number>;
  readonly body_mass: ArrayLike<number>;
}

export interface StandingState extends MuJoCoJointStateView {
  readonly subtree_com: ArrayLike<number>;
  readonly subtree_linvel: ArrayLike<number>;
  readonly xpos: ArrayLike<number>;
}

export interface StandingController {
  readonly gains: ReadonlyMap<number, JointPDSettings>;
  readonly footBodyIds: readonly [number, number];
  readonly anklePitchDofs: readonly [number, number];
  readonly ankleRollDofs: readonly [number, number];
  readonly referenceComXY: readonly [number, number];
  readonly totalMassKg: number;
  readonly kpX: number;
  readonly kdX: number;
  readonly kpY: number;
  readonly kdY: number;
  readonly ankleLimitNm: number;
}

/** Required ankle torque based on CoM error, CoM horizontal velocity and
 * gravity moment relative to ankle support. This does not create an
 * unphysical force: torque is delivered only to internal ankle DoFs.
 */
export function ankleBalanceTorques(
  comXY: readonly [number, number],
  comVelXY: readonly [number, number],
  ankleXY: readonly [number, number],
  referenceXY: readonly [number, number],
  totalMassKg: number,
  gains: Pick<StandingController, "kpX" | "kdX" | "kpY" | "kdY" | "ankleLimitNm">,
): readonly [number, number] {
  const values = [...comXY,...comVelXY,...ankleXY,...referenceXY,totalMassKg,
    gains.kpX,gains.kdX,gains.kpY,gains.kdY,gains.ankleLimitNm];
  if (!values.every(Number.isFinite) || totalMassKg <= 0 || gains.ankleLimitNm <= 0 ||
      [gains.kpX,gains.kdX,gains.kpY,gains.kdY].some(g => g < 0)) {
    throw new RangeError("Parámetros físicos inválidos para tobillos");
  }
  const sagittal = totalMassKg * 9.81 * (comXY[0] - ankleXY[0]) +
    gains.kpX * (comXY[0] - referenceXY[0]) + gains.kdX * comVelXY[0];
  const lateral = -(totalMassKg * 9.81 * (comXY[1] - ankleXY[1]) +
    gains.kpY * (comXY[1] - referenceXY[1]) + gains.kdY * comVelXY[1]);
  // Return TOTAL corrective torque: caller shares it between two ankles.
  return [sagittal,lateral];
}

export function makeStandingController(model: StandingModel, data: StandingState): StandingController {
  const footBodyIds = ["left_foot_geom","right_foot_geom"]
    .map(name => model.geom_bodyid[model.geom(name).id]!) as [number,number];
  const ankleDof = (name: string): number => model.jnt_dofadr[model.jnt(name).id]!;
  const totalMassKg = Array.from(model.body_mass).reduce((sum,mass)=>sum+mass,0);
  if (totalMassKg<=0 || !Number.isFinite(totalMassKg)) throw new Error("Masa total inválida");
  return {
    gains: new Map(Array.from(makeNeutralStandingPD(model), ([id,gain]) => [
      id,{ ...gain,
        stiffnessNmPerRad:gain.stiffnessNmPerRad*0.5,
        dampingNmsPerRad:gain.dampingNmsPerRad*0.5,
      },
    ])),
    footBodyIds,
    anklePitchDofs: [ankleDof("left_ankle_pitch"),ankleDof("right_ankle_pitch")],
    ankleRollDofs: [ankleDof("left_ankle_roll"),ankleDof("right_ankle_roll")],
    referenceComXY:[data.subtree_com[3]!,data.subtree_com[4]!],
    totalMassKg,
    kpX:1200, kdX:300, kpY:1200, kdY:300, ankleLimitNm:160,
  };
}

/** Apply neutral joint PD plus COM feedback. Requires mj_subtreeVel to have
 * refreshed subtree_linvel before calling. Does not call mj_step.
 */
export function stepActiveStanding(
  model: StandingModel, data: StandingState, controller: StandingController,
): readonly [number,number] {
  stepNeutralStandingPD(model,data,controller.gains);
  const [left,right]=controller.footBodyIds;
  const xpos=data.xpos;
  const ankleXY: readonly [number,number]=[
    (xpos[left*3]!+xpos[right*3]!)/2,
    (xpos[left*3+1]!+xpos[right*3+1]!)/2,
  ];
  const [pitch,roll]=ankleBalanceTorques(
    [data.subtree_com[3]!,data.subtree_com[4]!],
    [data.subtree_linvel[3]!,data.subtree_linvel[4]!],
    ankleXY,controller.referenceComXY,controller.totalMassKg,controller,
  );
  const clip=(v:number)=>Math.max(-controller.ankleLimitNm,Math.min(controller.ankleLimitNm,v));
  for(const id of controller.anklePitchDofs){
    data.qfrc_applied[id]=clip(data.qfrc_applied[id]!+pitch/2);
  }
  for(const id of controller.ankleRollDofs){
    data.qfrc_applied[id]=clip(data.qfrc_applied[id]!+roll/2);
  }
  if (Array.from(data.qfrc_applied).slice(0,6).some(v=>v!==0)) {
    throw new Error("Se detectó fuerza no permitida sobre pelvis libre");
  }
  return [pitch,roll];
}
