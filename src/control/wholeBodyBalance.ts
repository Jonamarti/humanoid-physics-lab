import type { XYZ } from "./balanceSensors";

/** Opt-in angular-momentum redistribution; NOT an inverse-dynamics solver.
 * MuJoCo computes whole-humanoid subtree angular momentum about its COM.
 * We can change segment momentum with internal joint torques, while the
 * change in *total* angular momentum also depends on external ground forces.
 * No contact => no claim of externally stabilizing angular momentum.
 */
export interface WholeBodyBalanceInput {
  readonly angularMomentumWorldNms: XYZ; // body=1 subtree_angmom, kg m^2/s
  readonly rootAngularVelocityWorldRadS: XYZ; // free-root qvel angular component
  readonly totalMassKg: number;
  readonly solvedFootLoadN: number; // from floor contacts, not number of contacts
}

export interface WholeBodyBalanceCommand {
  readonly active: boolean;
  readonly targetsRad: Readonly<Record<string,number>>;
  readonly normalizedMomentumXYZ: XYZ;
  readonly supportFraction: number;
}

/** Exact whitelist of internal hinges, no free root, toes or unreachable joints. */
export const WHOLE_BODY_JOINTS = [
  "left_shoulder_pitch", "right_shoulder_pitch",
  "left_shoulder_roll", "right_shoulder_roll",
  "left_shoulder_yaw", "right_shoulder_yaw",
  "left_hip_pitch", "right_hip_pitch",
  "left_hip_roll", "right_hip_roll",
  "lumbar_pitch", "thorax_pitch",
  "lumbar_roll", "thorax_roll",
] as const;

const clamp=(n:number,a:number,b:number):number=>Math.max(a,Math.min(b,n));
const deadband=(n:number,epsilon:number):number=>
  Math.abs(n)<=epsilon ? 0 : Math.sign(n)*(Math.abs(n)-epsilon)/(1-epsilon);

/** Experimentally bounded poses, not an optimal LQR/MPC controller.
 * The same-sign arm rotations try to create torso reaction moments.
 * Leg and spine corrections are smaller to preserve foot contacts.
 * No numerical success is implied by a nonzero requested pose.
 */
export function wholeBodyBalanceCommand(input:WholeBodyBalanceInput):WholeBodyBalanceCommand {
  const values=[...input.angularMomentumWorldNms,
    ...input.rootAngularVelocityWorldRadS,
    input.totalMassKg,input.solvedFootLoadN];
  if(!values.every(Number.isFinite) ||
    input.angularMomentumWorldNms.length!==3 ||
    input.rootAngularVelocityWorldRadS.length!==3 ||
    input.totalMassKg<=0 || input.solvedFootLoadN<0){
    throw new RangeError("Momento angular o apoyo inválidos");
  }
  const fraction=input.solvedFootLoadN/(input.totalMassKg*9.81);
  const active=fraction>=0.45; // no pretend-to-recover in free flight
  const v:XYZ=[0,1,2].map(i=>
    clamp(input.angularMomentumWorldNms[i]!/(input.totalMassKg*0.12) +
      input.rootAngularVelocityWorldRadS[i]!*0.25,-1,1)) as unknown as XYZ;
  const normalized:XYZ=v.map(a=>deadband(a,0.025)) as unknown as XYZ;
  if(!active || normalized.every(x=>x===0)){
    return {active:false,targetsRad:{},
      normalizedMomentumXYZ:normalized,supportFraction:fraction};
  }
  // Positive feedback target on limb (a relative joint rotation) creates
  // opposing reaction on its proximal body. This is a heuristic sign
  // choice that must be verified with MuJoCo push trials.
  const [roll,pitch,yaw]=normalized;
  const targets:Record<string,number>={
    left_shoulder_pitch:0.33*pitch,right_shoulder_pitch:0.33*pitch,
    left_shoulder_roll:0.25*roll,right_shoulder_roll:0.25*roll,
    left_shoulder_yaw:0.15*yaw,right_shoulder_yaw:0.15*yaw,
    left_hip_pitch:0.045*pitch,right_hip_pitch:0.045*pitch,
    left_hip_roll:0.035*roll,right_hip_roll:0.035*roll,
    lumbar_pitch:0.050*pitch,thorax_pitch:0.025*pitch,
    lumbar_roll:0.045*roll,thorax_roll:0.020*roll,
  };
  return {active:true,targetsRad:targets,
    normalizedMomentumXYZ:normalized,supportFraction:fraction};
}
