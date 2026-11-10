/** Trial-only joint-target hold after physically verified foot landing.
 * This stabilizes the *requested posture* without constraining the foot,
 * altering friction, adding forces or acting on the free root.
 *
 * A measured joint angle is not a pose keyframe: the normal limited-torque
 * actuator continues to determine whether it can physically track it.
 */
export const LANDING_POSE_JOINTS = [
  "hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll",
] as const;

export function blendMeasuredLandingPose(
  nominal:Readonly<Record<string,number>>,
  measured:Readonly<Record<string,number>>,
  side:"left"|"right",
  timeSinceLandingS:number,
  strength:number,
):Record<string,number> {
  if(!Number.isFinite(timeSinceLandingS) || timeSinceLandingS<0 ||
     !Number.isFinite(strength) || strength<0 || strength>1 ||
     !Object.values(nominal).every(Number.isFinite) ||
     !Object.values(measured).every(Number.isFinite)) {
    throw new RangeError("Pose de apoyo o intensidad inválida");
  }
  const u=Math.min(1,timeSinceLandingS/1.45);
  const fade=u*u*(3-2*u);
  const influence=strength*(1-fade);
  const targets:Record<string,number>={...nominal};
  if(influence===0) return targets;
  for(const part of LANDING_POSE_JOINTS) {
    const name=side+"_"+part;
    const measuredAngle=measured[name];
    if(measuredAngle===undefined || !Number.isFinite(measuredAngle)) {
      throw new RangeError("Falta ángulo físico de aterrizaje: "+name);
    }
    // Neutral targets absent from nominal still get a legitimate
    // limited-torque PD command while the landing pose is held.
    const desired=nominal[name]??0;
    targets[name]=desired*(1-influence)+measuredAngle*influence;
  }
  return targets;
}
