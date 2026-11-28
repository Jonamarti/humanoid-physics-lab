/** Isolated heading-control experiment for the free-root reference.
 * Only hip axial rotation hinges are added. No yaw constraint or root actuation.
 * The stance projection uses each actual joint axis in world coordinates;
 * a commanded internal child moment reacts on the pelvis with opposite sign.
 */
import { clamp,type Pair } from "./simbiconCore";
export type HeadingPolicy="neutral"|"heading";
export const headingJointNames=["lHipYaw","rHipYaw"] as const;
export const headingLimitNm=40;
const wrap=(a:number):number=>Math.atan2(Math.sin(a),Math.cos(a));
export interface HeadingObservation {
  yaw:number;yawRate:number;footYaw:Pair;footYawRate:Pair;
  jointQ:Pair;jointRate:Pair;axisZ:Pair;loadN:Pair;filteredLoadN:Pair;
}
export interface HeadingCommand {hipNm:Pair;desiredRootNm:number;projectedReactionNm:number;residualNm:number}
export function headingCommand(policy:HeadingPolicy,o:HeadingObservation):HeadingCommand {
  if(![o.yaw,o.yawRate,...o.footYaw,...o.footYawRate,...o.jointQ,...o.jointRate,...o.axisZ,
    ...o.loadN,...o.filteredLoadN].every(Number.isFinite)||o.loadN.some(v=>v<0)||o.filteredLoadN.some(v=>v<0)||
    o.axisZ.some(v=>Math.abs(v)>1+1e-8))throw new RangeError("Invalid heading observation");
  const tau:[number,number]=[0,0];
  const w:[number,number]=[0,0];
  for(const i of [0,1] as const){
    const posture=-50*o.jointQ[i]-5*o.jointRate[i];
    // Unloaded limb orientation is a relative joint target plus bounded
    // world-heading feedback. During support, use a relative posture target
    // before distributing the desired pelvis reaction.
    const orient=policy==="heading"&&o.loadN[i]<20?-30*wrap(o.footYaw[i])-3*o.footYawRate[i]:0;
    tau[i]=clamp(posture+orient,-headingLimitNm,headingLimitNm);
    w[i]=o.loadN[i]>=20?o.filteredLoadN[i]:0;
  }
  const total=w[0]+w[1];
  const desired=policy==="heading"&&total>1?clamp(-70*wrap(o.yaw)-8*o.yawRate,-60,60):0;
  const reaction=()=>-tau[0]*o.axisZ[0]-tau[1]*o.axisZ[1];
  if(policy==="heading"&&total>1){
    const denominator=w[0]*o.axisZ[0]**2+w[1]*o.axisZ[1]**2;
    if(denominator>1e-8){
      const error=desired-reaction();
      for(const i of [0,1] as const){
        tau[i]=clamp(tau[i]-error*w[i]*o.axisZ[i]/denominator,-headingLimitNm,headingLimitNm);
      }
    }
    // A clipped hinge can leave a residual. One deterministic pass can
    // allocate feasible remaining effort only to a currently loaded leg.
    for(const i of [0,1] as const)if(w[i]>0&&Math.abs(o.axisZ[i])>.1){
      tau[i]=clamp(tau[i]-(desired-reaction())/o.axisZ[i],-headingLimitNm,headingLimitNm);
    }
  }
  const actual=reaction();
  return {hipNm:tau,desiredRootNm:desired,projectedReactionNm:actual,residualNm:desired-actual};
}
