import { capturePointXY, insideConvexHullXY, type XY, type XYZ } from "./balanceSensors";
import type { GroundSupportObservation } from "./footContacts";
import { planRecovery, type FootSide } from "./recoveryPlanner";
import type { StepPhase } from "./experimentalStep";

/** Decision layer only. Foot motion is delegated to the physical MuJoCo
 * ExperimentalStepSequence; no free-root forces or kinematic foot placement.
 */
export interface ReactiveStepObservation {
  readonly timeS: number;
  readonly comWorldM: XYZ;
  readonly comVelocityWorldMps: XYZ;
  readonly totalMassKg: number;
  readonly rootTiltDeg: number;
  readonly pelvisHeightM: number;
  readonly ground: GroundSupportObservation;
}
export interface ReactiveStepIntent {
  readonly timeS: number;
  readonly swingSide: FootSide;
  readonly supportSide: FootSide;
  /** Relative translation of the swing foot, in WORLD XY metres. */
  readonly deltaXY: XY;
  readonly capturePointXY: XY;
}
export type ReactiveRecoveryPhase =
  "MONITORING" | "WARNING" | "NO_SAFE_STEP" | "STEP_ATTEMPT" |
  "STEP_ABORT" | "STEP_COMPLETE" | "FALL";
export interface ReactiveRecoveryStatus {
  readonly phase: ReactiveRecoveryPhase;
  readonly reason: string;
  readonly attempts: number;
  readonly unstableDurationS: number;
  readonly capturePointXY: XY | null;
  readonly lastIntent: ReactiveStepIntent | null;
  readonly physicalStepPhase: StepPhase | null;
}

const finite = (values:readonly number[]) => values.every(Number.isFinite);
const MIN_SPEED_MPS = 0.12;
const PLANNING_DELAY_S = 0.12;
const MAX_REACH_M = 0.34;
const MIN_EFFECTIVE_FORCE_FRACTION = 0.60;

export interface ReactiveStepOptions {
  /** Opt-in research parameter; all physical release/completion gates stay fixed. */
  readonly planningDelayS?: number;
  /** Linear COM forecast used only to anticipate a step request. */
  readonly captureLookaheadS?: number;
  readonly emergencyLateralShiftM?: number;
  readonly emergencyShiftRampS?: number;
  readonly transferHipRollRad?: number;
  readonly transferAnkleDampingScale?: number;
}

/** One automatic attempt per trial. Never retry indefinitely after a
 * failed lift or fall; manual reset required before another trial.
 * Hysteresis avoids responding to one noisy contact/capture sample.
 */
export class ReactiveStepCoordinator {
  constructor(readonly options:ReactiveStepOptions = {}) {
    const delay=options.planningDelayS??PLANNING_DELAY_S;
    if(!Number.isFinite(delay) || delay<0 || delay>0.12)
      throw new RangeError("Latencia experimental fuera de 0..120 ms");
    const lookahead=options.captureLookaheadS??0;
    if(!Number.isFinite(lookahead) || lookahead<0 || lookahead>0.4)
      throw new RangeError("Anticipación experimental fuera de 0..400 ms");
    const shift=options.emergencyLateralShiftM??0.115;
    const ramp=options.emergencyShiftRampS??0.35;
    const hip=options.transferHipRollRad??0;
    const damping=options.transferAnkleDampingScale??1;
    if(!Number.isFinite(shift) || shift<0.05 || shift>0.15 ||
       !Number.isFinite(ramp) || ramp<0.04 || ramp>0.5 ||
       !Number.isFinite(hip) || Math.abs(hip)>0.3 ||
       !Number.isFinite(damping) || damping<0 || damping>1)
      throw new RangeError("Referencia experimental de transferencia fuera de límites");
  }
  private phase:ReactiveRecoveryPhase="MONITORING";
  private reason="Equilibrio con corrección articular; sin paso solicitado";
  private lastSampleS=-Infinity;
  private warningSinceS:number|undefined;
  private unstableDurationS=0;
  private attempts=0;
  private capturePointXY:XY|null=null;
  private lastIntent:ReactiveStepIntent|null=null;
  private physicalStepPhase:StepPhase|null=null;

  snapshot():ReactiveRecoveryStatus {
    return {
      phase:this.phase,reason:this.reason,attempts:this.attempts,
      unstableDurationS:this.unstableDurationS,
      capturePointXY:this.capturePointXY,
      lastIntent:this.lastIntent,
      physicalStepPhase:this.physicalStepPhase,
    };
  }

  observe(input:ReactiveStepObservation):ReactiveStepIntent|null {
    const {timeS,comWorldM:com,comVelocityWorldMps:vel,
      totalMassKg:mass,ground,rootTiltDeg:tilt,pelvisHeightM:pelvis} = input;
    if(!finite([timeS,...com,...vel,mass,tilt,pelvis,
      ground.totalVerticalLoadN]) || mass<=0 || com[2]<=0 ||
      tilt<0 || timeS<this.lastSampleS){
      throw new RangeError("Observaciones de recuperación reactiva inválidas");
    }
    const dt=timeS-this.lastSampleS;
    this.lastSampleS=timeS;
    const velocityXY:XY=[vel[0],vel[1]];
    const speed=Math.hypot(...velocityXY);
    const comXY:XY=[com[0],com[1]];
    const capture=capturePointXY(comXY,velocityXY,com[2]);
    this.capturePointXY=capture;
    // Terminal attempts stay terminal until simulation reset. A FALL can
    // still supersede an earlier ABORT when the body subsequently falls.
    if(pelvis<0.65 || tilt>50){
      this.phase="FALL";
      this.reason="Caída física detectada por altura o inclinación";
      return null;
    }
    if(this.attempts>0 || this.phase==="FALL") return null;

    const lookahead=this.options.captureLookaheadS??0;
    const planningCom:XY=[comXY[0]+vel[0]*lookahead,comXY[1]+vel[1]*lookahead];
    const predictedCapture:XY=[capture[0]+vel[0]*lookahead,capture[1]+vel[1]*lookahead];
    const movingOutward=speed>=MIN_SPEED_MPS &&
      !insideConvexHullXY(predictedCapture,ground.loadedSupportPolygonXY,0.004);
    const supportAvailable=ground.totalVerticalLoadN>=
      mass*9.81*MIN_EFFECTIVE_FORCE_FRACTION;
    if(!movingOutward || !supportAvailable){
      this.warningSinceS=undefined;
      this.unstableDurationS=0;
      this.phase="MONITORING";
      this.reason=!supportAvailable ? "Apoyo medido insuficiente para iniciar un paso" :
        "El punto de captura permanece dentro del apoyo o se mueve lentamente";
      return null;
    }
    if(!Number.isFinite(dt) || dt>0.06) this.warningSinceS=undefined;
    this.warningSinceS??=timeS;
    this.unstableDurationS=timeS-this.warningSinceS;
    if(this.unstableDurationS<(this.options.planningDelayS??PLANNING_DELAY_S)){
      this.phase="WARNING";
      this.reason="Desequilibrio persistente: mantener corrección corporal antes de pisar";
      return null;
    }
    // The existing reachability planner uses solved loaded foot polygons,
    // prevents crossing the legs and tests the future capture region.
    const plan=planRecovery({
      comXY:planningCom,comVelocityXY:velocityXY,comHeightM:com[2],
      ground,excessiveTrunkTilt:tilt>35,
      unstableDurationS:this.unstableDurationS,
      maxStepReachM:MAX_REACH_M,stepReactionDelayS:0.12,
      planningLatencyS:this.options.planningDelayS??PLANNING_DELAY_S,
      captureMarginM:0.002,
    });
    if((plan.decision!=="STEP" && plan.decision!=="SHIFT_WEIGHT") ||
        !plan.swingFoot || !plan.supportFoot || !plan.targetFootXY){
      this.phase="NO_SAFE_STEP";
      this.reason="Sin apoyo/paso alcanzable: "+plan.reason;
      return null;
    }
    const swing=ground[plan.swingFoot];
    const delta:XY=[
      plan.targetFootXY[0]-swing.centerXY[0],
      plan.targetFootXY[1]-swing.centerXY[1],
    ];
    const len=Math.hypot(...delta);
    // Real step is never issued opposite to the current COM velocity.
    if(len<0.04 || len>0.38 || delta[0]*vel[0]+delta[1]*vel[1]<=0){
      this.phase="NO_SAFE_STEP";
      this.reason="Objetivo demasiado cercano, fuera del alcance o contrario al COM";
      return null;
    }
    const intent:ReactiveStepIntent={
      timeS,swingSide:plan.swingFoot,supportSide:plan.supportFoot,
      deltaXY:delta,capturePointXY:capture,
    };
    this.lastIntent=intent;
    this.attempts=1;
    this.phase="STEP_ATTEMPT";
    this.physicalStepPhase="STAND";
    this.reason="Paso reactivo solicitado en dirección del punto de captura; pendiente de despegue real";
    return intent;
  }

  observePhysicalStep(phase:StepPhase,reason:string):void {
    if(this.attempts===0) return;
    this.physicalStepPhase=phase;
    if(phase==="FALL"){
      this.phase="FALL";
      this.reason="Fallo del paso con caída: "+reason;
    } else if(phase==="ABORT"){
      if(this.phase!=="FALL"){
        this.phase="STEP_ABORT";
        this.reason="Intento de paso abortado sin aterrizaje estable: "+reason;
      }
    } else if(phase==="COMPLETE"){
      this.phase="STEP_COMPLETE";
      this.reason="Paso físico completo según las puertas de estabilidad: "+reason;
    }
  }
}
