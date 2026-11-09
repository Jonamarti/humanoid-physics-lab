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
