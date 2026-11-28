/** First coronal experiments on the verified sagittal reference (#114).
 * Uses physical roll hinges; the six root DOFs remain unactuated.
 * Posture-only control is an ablation. Capture mode adds a LIPM foot
 * placement heuristic and bounded ankle CoP control. Neither is GENBICON.
 */
import { clamp,type Pair } from "./simbiconCore";
import { loadedHipReaction,measuredShares } from "./genbiconCore";
export type CoronalPolicy="posture"|"capture";
export const coronalJointNames=["lHipRoll","rHipRoll","lAnkleRoll","rAnkleRoll"] as const;
export const hipLimitNm=100,ankleLimitNm=45;
export interface CoronalObservation {
  swing:0|1;phaseElapsedS:number;lifting:boolean;
  comY:number;comZ:number;velocityY:number;torsoRoll:number;torsoRollRate:number;
  hipQ:Pair;hipRate:Pair;ankleQ:Pair;ankleRate:Pair;
  ankleY:Pair;hipToAnkleJacobianY:Pair;footRoll:Pair;soleY:Pair;loadN:Pair;
  filteredLoadN:Pair;
}
export interface CoronalCommand {
  hipNm:Pair;ankleNm:Pair;captureY:number;targetFootY:number;
  targetHipQ:number;desiredCopY:number;hipResidualNm:number;
}
export function lateralCapturePoint(y:number,velocity:number,height:number):number {
  if(![y,velocity,height].every(Number.isFinite)||height<=0)throw new RangeError("Invalid capture point");
  return y+velocity/Math.sqrt(9.81/height);
}
export function swingHipTarget(q:number,ankleY:number,targetY:number,jacobianY:number):number {
  if(![q,ankleY,targetY,jacobianY].every(Number.isFinite))throw new RangeError("Invalid swing Jacobian");
  return Math.abs(jacobianY)<.15?clamp(q,-.35,.35):clamp(q+clamp((targetY-ankleY)/jacobianY,-.25,.25),-.35,.35);
}
export function coronalCommand(policy:CoronalPolicy,o:CoronalObservation):CoronalCommand {
  const values=[o.phaseElapsedS,o.comY,o.comZ,o.velocityY,o.torsoRoll,o.torsoRollRate,
    ...o.hipQ,...o.hipRate,...o.ankleQ,...o.ankleRate,...o.ankleY,...o.hipToAnkleJacobianY,
    ...o.footRoll,...o.soleY,...o.loadN,...o.filteredLoadN];
  if(!values.every(Number.isFinite)||o.loadN.some(x=>x<0)||o.filteredLoadN.some(x=>x<0))
    throw new RangeError("Invalid coronal observation");
  const s=o.swing,st=1-s as 0|1;
  const h=clamp(o.comZ,.4,1.3),omega=Math.sqrt(9.81/h);
  const cp=lateralCapturePoint(o.comY,o.velocityY,h);
  const remaining=o.lifting?clamp(.3-o.phaseElapsedS,0,.3)+.04:.04;
  // Predict DCM to landing under the current support foot. Limit the
  // requested reach, not the physical state, and keep a small nominal step width.
  const projected=o.soleY[st]+(cp-o.soleY[st])*Math.exp(omega*remaining);
  const footY=o.comY+clamp(projected-o.comY+(s===0?.045:-.045),-.23,.23);
  const hipTarget=policy==="capture"?swingHipTarget(o.hipQ[s],o.ankleY[s],footY,o.hipToAnkleJacobianY[s]):0;
  const base: [number,number]=[0,0];
  const ankle: [number,number]=[0,0];
  for(const i of [0,1] as const) {
    base[i]=clamp(160*((i===s?hipTarget:0)-o.hipQ[i])-14*o.hipRate[i],-hipLimitNm,hipLimitNm);
    // Flatten the sole relative to world when it is unloaded. The roll
    // joint is child-positive, so positive world roll needs negative torque.
    ankle[i]=clamp(-100*o.footRoll[i]-8*o.ankleRate[i],-ankleLimitNm,ankleLimitNm);
  }
  const weights=measuredShares([o.loadN[0]>=20?o.filteredLoadN[0]:0,o.loadN[1]>=20?o.filteredLoadN[1]:0]);
  const hasSupport=weights[0]+weights[1]>0;
  const cop=o.comY+h/9.81*(3*o.comY+4*o.velocityY);
  let residual=0;
  if(policy==="capture"&&hasSupport){
    const rootMoment=-180*o.torsoRoll-20*o.torsoRollRate;
    const hip=loadedHipReaction(base,rootMoment,weights,hipLimitNm);
    base[0]=hip.left;base[1]=hip.right;residual=hip.residualNm;
    const center=weights[0]*o.soleY[0]+weights[1]*o.soleY[1];
    const shift=clamp(cop-center,-.025,.025);
    for(const i of [0,1] as const)if(weights[i]>0) {
      // Upward ground force at +Y produces +X foot moment. The
      // internal ankle torque must oppose it (child-positive convention).
      const f=o.filteredLoadN[i];
      const requested=-f*(o.soleY[i]+shift-o.ankleY[i])-3*o.ankleRate[i];
      const maxMoment=Math.min(ankleLimitNm,f*.035);
      ankle[i]=clamp(requested,-maxMoment,maxMoment);
    }
  }
  return {hipNm:base,ankleNm:ankle,captureY:cp,targetFootY:footY,targetHipQ:hipTarget,desiredCopY:cop,hipResidualNm:residual};
}
