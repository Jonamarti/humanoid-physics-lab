import type { XY } from "./balanceSensors";
import type { StepPhase } from "./experimentalStep";
import type { FootSide } from "./recoveryPlanner";

/** Kinematic intent for a voluntary STEP, not a guaranteed gait or a fake
 * root movement. MuJoCo integrates every joint and enforces floor contacts.
 * World +X is forward, joint hip_pitch -q raises the thigh forward, and
 * the corrected -Y-axis knee bends toward the REAR at negative q.
 */
export interface GaitPosture {
  readonly targetsRad: Readonly<Record<string,number>>;
  readonly forwardComBiasM: number;
  readonly requestedKneeFlexionRad: number;
  readonly requestedTrunkPitchRad: number;
}

const clamp=(n:number,a:number,b:number):number=>Math.max(a,Math.min(b,n));
export const smoothStep = (timeS:number,durationS:number):number => {
  if (!Number.isFinite(timeS) || !Number.isFinite(durationS) || durationS<=0) {
    throw new RangeError("Tiempo/rampa inválidos");
  }
  const u=clamp(timeS/durationS,0,1);
  return u*u*(3-2*u);
};

/** Separately testable articulated 'controlled falling' posture:
 * - SHIFT_WEIGHT: small sagittal COM shift + torso pitch (not a root force).
 * - LIFT: swing hip flexes, knee folds backwards to shorten the leg.
 * - SWING: knee begins extending while thigh advances.
 * - LAND: knee extends smoothly; no instantaneous snapping to neutral.
 * - RECOVER: gradually straighten torso, leave center reference to FSM.
 * Preserves ~10cm ground clearance as geometric intent but does not
 * assert the foot will actually reach it.
 */
export function controlledStepPosture(input: {
  readonly phase: StepPhase;
  readonly elapsedS: number;
  readonly swingSide: FootSide;
  readonly targetDeltaXY: XY;
  readonly intensity?: number;
}): GaitPosture {
  const {phase,elapsedS,swingSide,targetDeltaXY} = input;
  const intensity=input.intensity??1;
  if (![elapsedS,...targetDeltaXY,intensity].every(Number.isFinite) ||
      elapsedS < 0 || intensity < 0 || intensity > 1 ||
      Math.hypot(...targetDeltaXY)>0.38) {
    throw new RangeError("Plan de marcha inválido");
  }
  const dx=targetDeltaXY[0];
  const sagittal = Math.abs(dx)>0.025 ? Math.sign(dx) : 0;
  // Forward lean of the torso is deliberately mild, ~7° total, so the
  // dynamic controller can observe/limit it; not a 500N imitation.
  const stanceRamp=phase==="SHIFT_WEIGHT" ? smoothStep(elapsedS,0.38) :
    phase==="STAND" || phase==="FALL" || phase==="ABORT" ? 0 :
    phase==="RECOVER" ? 1-smoothStep(elapsedS,0.65) :
    phase==="COMPLETE" ? 0 : 1;
  const trunkPitch=0.125*sagittal*stanceRamp*intensity;
  const bias=clamp(dx*0.3,-0.045,0.045)*stanceRamp*intensity;
  const targets:Record<string,number>={};
  if (trunkPitch!==0 || ["SHIFT_WEIGHT","LIFT","SWING","LAND","RECOVER"].includes(phase)) {
    targets.lumbar_pitch=trunkPitch*0.72;
    targets.thorax_pitch=trunkPitch*0.28;
  }
  const flex=phase==="LIFT" ? smoothStep(elapsedS,0.26) :
    phase==="SWING" ? 1-0.28*smoothStep(elapsedS,0.38) :
    phase==="LAND" ? 0.72*(1-smoothStep(elapsedS,0.40))+0.07 :
    phase==="RECOVER" ? 0.07*(1-smoothStep(elapsedS,0.6)) :
    0;
  const hipForward=phase==="LIFT" ? smoothStep(elapsedS,0.28) :
    phase==="SWING" ? 1 :
    phase==="LAND" ? 1-0.42*smoothStep(elapsedS,0.4) :
    phase==="RECOVER" ? 0.58*(1-smoothStep(elapsedS,0.7)) :
    0;
  const kneeFlex=-1.0*flex*intensity;
  if (["LIFT","SWING","LAND","RECOVER","COMPLETE"].includes(phase)) {
    targets[`${swingSide}_knee`]=kneeFlex;
    // Hip pitch -q brings the knee forward, while knee -q now shortens
    // the leg by folding the shin backwards, preventing toe scraping.
    const hip=-sagittal*0.34*hipForward*intensity;
    targets[`${swingSide}_hip_pitch`]=hip;
    const sign=swingSide==="left"?1:-1;
    targets[`${swingSide}_hip_roll`]=clamp(targetDeltaXY[1]/0.65,-0.32,0.32)*hipForward+
      (phase==="LIFT"?sign*0.07*smoothStep(elapsedS,0.25):0);
    targets[`${swingSide}_ankle_pitch`]=0.12*flex*intensity;
  }
  return {targetsRad:targets,forwardComBiasM:bias,
    requestedKneeFlexionRad:kneeFlex,requestedTrunkPitchRad:trunkPitch};
}
