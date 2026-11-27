/** C2: contact-gated lateral transfer and unilateral support.
 * Pure planner; the physics loop remains the only source of motion.
 * All timings/thresholds are declared before the first physical trial.
 */
import { clamp, type Pair, type Side } from "./simbiconCore";

export type UnilateralPhase = "prepare" | "shift" | "lift" | "hold" | "lower" | "recenter" | "done" | "timeout";
export interface UnilateralState {
  phase: UnilateralPhase;
  elapsedS: number;
  qualifiedS: number;
  longestSupportS: number;
  reachedShift: boolean;
  reachedHold: boolean;
  completed: boolean;
}
export interface UnilateralObservation {
  comY: number;
  velocityY: number;
  loadN: Pair;
  clearanceM: Pair;
  supportY: number;
  bodyWeightN: number;
  fallen: boolean;
}
export const smooth = (u: number): number => {
  const x = clamp(u,0,1);
  return x*x*(3-2*x);
};
export function initialUnilateral(): UnilateralState {
  return {phase:"prepare",elapsedS:0,qualifiedS:0,longestSupportS:0,reachedShift:false,reachedHold:false,completed:false};
}
export function advanceUnilateral(prev: UnilateralState, obs: UnilateralObservation, support: Side,
  liftRequested: boolean, dt: number): UnilateralState {
  if (!(dt>0) || !Number.isFinite(dt) || ![obs.comY,obs.velocityY,...obs.loadN,...obs.clearanceM,obs.supportY,obs.bodyWeightN].every(Number.isFinite)
      || obs.bodyWeightN<=0 || obs.loadN.some(v=>v<0)) throw new RangeError("Invalid unilateral observation");
  if (prev.phase==="done" || prev.phase==="timeout") return prev;
  const st=support==="left"?0:1, sw=1-st;
  const shifted = obs.loadN[st]! >= .75*obs.bodyWeightN && Math.abs(obs.comY-obs.supportY)<.045 && Math.abs(obs.velocityY)<.10;
  const supported = obs.loadN[st]! >= .75*obs.bodyWeightN && obs.loadN[sw]!<5 && obs.clearanceM[sw]!>.02;
  const both = obs.loadN[st]!>.20*obs.bodyWeightN && obs.loadN[sw]!>.20*obs.bodyWeightN;
  const next={...prev,elapsedS:prev.elapsedS+dt};
  const go=(phase:UnilateralPhase):UnilateralState=>({...next,phase,elapsedS:0,qualifiedS:0});
  if(obs.fallen) return go("timeout");
  switch(prev.phase) {
    case "prepare": return next.elapsedS>=.5?go("shift"):next;
    case "shift":
      next.qualifiedS=shifted?prev.qualifiedS+dt:0;
      if(next.elapsedS>=1.5 && next.qualifiedS>=.15) {
        next.reachedShift=true;
        return go(liftRequested?"lift":"recenter");
      }
      return next.elapsedS>=4?go("timeout"):next;
    case "lift":
      next.qualifiedS=supported?prev.qualifiedS+dt:0;
      if(next.elapsedS>=.5 && next.qualifiedS>=.10) return go("hold");
      return next.elapsedS>=2?go("timeout"):next;
    case "hold":
      next.qualifiedS=supported?prev.qualifiedS+dt:0;
      next.longestSupportS=Math.max(prev.longestSupportS,next.qualifiedS);
      if(next.qualifiedS>=2) { next.reachedHold=true; return go("lower"); }
      return next.elapsedS>=4?go("timeout"):next;
    case "lower":
      if(next.elapsedS>=.6 && obs.loadN[sw]!>.10*obs.bodyWeightN && obs.clearanceM[sw]!<.015) return go("recenter");
      return next.elapsedS>=2?go("timeout"):next;
    case "recenter":
      next.qualifiedS=both && Math.abs(obs.velocityY)<.08?prev.qualifiedS+dt:0;
      if(next.elapsedS>=1.5 && next.qualifiedS>=.25) {next.completed=true;return go("done");}
      return next.elapsedS>=4?go("timeout"):next;
  }
}
export function unilateralTargets(state: UnilateralState, centerY: number, supportY: number): {refY:number;flexion:number} {
  const phase=state.phase;
  const w=phase==="prepare"||phase==="done"?0:phase==="shift"?smooth(state.elapsedS/1.5):
    phase==="recenter"?1-smooth(state.elapsedS/1.5):1;
  const flexion=phase==="lift"?smooth(state.elapsedS/.5):phase==="hold"?1:
    phase==="lower"?1-smooth(state.elapsedS/.6):0;
  return {refY:centerY+w*(supportY-centerY),flexion};
}

/** Allocate vertical reactions so their weighted sole centers reproduce the
 * requested lateral CoP whenever possible. A residual outside the center
 * interval is handled by local foot pressure shifts; no fictitious force.
 * This is a desired load, not a claim that the contact solver achieved it.
 */
export function lateralLoadTarget(desiredCopY:number,centers:Pair,loaded:readonly [boolean,boolean]):Pair {
  if(![desiredCopY,...centers].every(Number.isFinite)) throw new RangeError("Invalid CoP allocation");
  if(!loaded[0]) return [0,loaded[1]?1:0];
  if(!loaded[1]) return [1,0];
  const span=centers[0]-centers[1];
  if(Math.abs(span)<1e-6) return [.5,.5];
  const left=clamp((desiredCopY-centers[1])/span,0,1);
  return [left,1-left];
}
/** Vertical contact force has joint moment axis dot (lever cross F).
 * Correct measured-vs-requested normal load with the opposite internal knee
 * moment. This is a limited virtual-force correction, not an external push.
 */
export function kneeLoadCorrection(desiredN:number,observedN:number,momentArmM:number):number {
  if(![desiredN,observedN,momentArmM].every(Number.isFinite)||desiredN<0||observedN<0) throw new RangeError("Invalid load correction");
  return clamp(-(desiredN-observedN)*momentArmM,-30,30);
}
