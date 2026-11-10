import type { XY, XYZ } from "./balanceSensors";
import { limitedJointTorque, type JointPDSettings, type MuJoCoJointStateView } from "./jointPD";
import type { NamedHingeModel } from "./standingBaseline";
import type { FootSide } from "./recoveryPlanner";
import { controlledStepPosture } from "./controlledStepPosture";
import { assessStepStability, type StepStabilityAssessment } from "./stepStabilityDiagnostics";

/** EXPERIMENTAL: physical joints only. No root actuation, foot teleportation
 * or fake contact. An aborted attempt is NOT a simulated fall.
 */
export type StepPhase =
  "STAND" | "SHIFT_WEIGHT" | "LIFT" | "SWING" | "LAND" | "RECOVER" |
  "COMPLETE" | "ABORT" | "FALL";

export interface StepFeedback {
  readonly timeS: number;
  readonly comWorldM: XYZ;
  readonly comVelocityWorldMps: XYZ;
  readonly comProjectionInsideLoadedSupport: boolean;
  readonly capturePointInsideLoadedSupport: boolean;
  readonly pelvisHeightM: number;
  readonly rootTiltDeg: number;
  readonly swingSoleWorldM: XYZ;
  /** Filtered speed from physical integration. Sampled ~20ms apart. */
  readonly swingSoleVelocityWorldMps?: XYZ;
  /** Solver's measured tangential-friction utilization at this sample. */
  readonly swingFrictionUtilization?: number | null;
  readonly supportSoleWorldM: XYZ;
  readonly supportLoadN: number;
  readonly swingLoadN: number;
}

export interface StepCommand {
  readonly phase: StepPhase;
  readonly referenceComXY: XY;
  readonly targetsRad: Readonly<Record<string, number>>;
  /** World-space end-effector target, used by bounded Jacobian feedback. */
  readonly soleTargetWorldM: XYZ | null;
  readonly cartesianBlend: number;
}

export interface StepSnapshot {
  readonly phase: StepPhase;
  readonly reason: string;
  readonly maxSoleRiseM: number;
  readonly maxSoleTravelXYM: number;
  readonly maxTargetProgressM: number;
  readonly currentTargetProgressM: number;
  readonly minPelvisM: number;
  readonly maxRootTiltDeg: number;
  readonly maxSupportLoadFraction: number;
  readonly stepContactConfirmed: boolean;
  readonly maxLandingSlipM: number;
  readonly maxSupportFootSlipM: number;
  readonly targetErrorXYM: number;
  /** Consecutive seconds of verified stability, sampled at physics cadence. */
  readonly settledDurationS: number;
  /** Last unloaded-foot velocity sampled BEFORE the loaded landing frame.
   * This is a ~20ms-cadence proxy, not an exact contact-time impact velocity.
   */
  readonly preLandingTangentialSpeedMps: number | null;
  readonly preLandingDownwardSpeedMps: number | null;
  readonly frictionUtilizationAtLanding: number | null;
  readonly firstLandingTimeS: number | null;
  /** Every physics gate required for sustained step completion. */
  readonly stability: StepStabilityAssessment | null;
}

/** One voluntary step attempt, useful for testing the controller in real
 * MuJoCo before plugging it into the perturbation planner. Phase changes
 * are based on measured contact/load/foot motion, with strict timeouts.
 */
export class ExperimentalStepSequence {
  private phase: StepPhase = "STAND";
  private phaseStartedAtS: number;
  private readonly supportSide: FootSide;
  private originalSole: XYZ | undefined;
  private originalSupportSole: XYZ | undefined;
  private maxSupportFootSlipM = 0;
  private targetErrorXYM = Infinity;
  private lastTimeS = -Infinity;
  private reason = "Esperando inicio";
  private maxSoleRiseM = 0;
  private maxSoleTravelXYM = 0;
  private maxTargetProgressM = 0;
  private currentTargetProgressM = 0;
  private minPelvisM = Infinity;
  private maxRootTiltDeg = 0;
  private maxSupportLoadFraction = 0;
  private stepContactConfirmed = false;
  private firstLandingSole: XYZ | undefined;
  private maxContactSlipM = 0;
  private latestSwingLoadN = 0;
  private settledSinceS: number | undefined;
  private settledDurationS = 0;
  private lastUnloadedFootVelocityWorldMps: XYZ | undefined;
  private firstLandingVelocityEstimate: XYZ | undefined;
  private frictionUtilizationAtLanding: number | null = null;
  private firstLandingTimeS: number | null = null;
  private stability: StepStabilityAssessment | null = null;

  constructor(
    readonly swingSide: FootSide,
    readonly targetDeltaXY: XY,
    readonly startTimeS = 0.8,
    readonly coordinatedGaitProfile = 0,
  ) {
    if (!targetDeltaXY.every(Number.isFinite) ||
      Math.hypot(...targetDeltaXY) > 0.38 ||
      Math.hypot(...targetDeltaXY) < 0.03 || startTimeS < 0 || !Number.isFinite(startTimeS) ||
      !Number.isFinite(coordinatedGaitProfile) || coordinatedGaitProfile < 0 || coordinatedGaitProfile > 1) {
      throw new RangeError("Solicitud de paso fuera de alcance");
    }
    this.supportSide = swingSide === "left" ? "right" : "left";
    this.phaseStartedAtS = startTimeS;
  }

  snapshot(): StepSnapshot {
    return {
      phase: this.phase, reason: this.reason,
      maxSoleRiseM: this.maxSoleRiseM,
      maxSoleTravelXYM: this.maxSoleTravelXYM,
      maxTargetProgressM: this.maxTargetProgressM,
      currentTargetProgressM: this.currentTargetProgressM,
      minPelvisM: this.minPelvisM,
      maxRootTiltDeg: this.maxRootTiltDeg,
      maxSupportLoadFraction: this.maxSupportLoadFraction,
      stepContactConfirmed: this.stepContactConfirmed,
      maxLandingSlipM: this.maxContactSlipM,
      maxSupportFootSlipM: this.maxSupportFootSlipM,
      targetErrorXYM: this.targetErrorXYM,
      settledDurationS: this.settledDurationS,
      preLandingTangentialSpeedMps: this.firstLandingVelocityEstimate
        ? Math.hypot(this.firstLandingVelocityEstimate[0],
          this.firstLandingVelocityEstimate[1]) : null,
      preLandingDownwardSpeedMps: this.firstLandingVelocityEstimate
        ? Math.max(0,-this.firstLandingVelocityEstimate[2]) : null,
      frictionUtilizationAtLanding: this.frictionUtilizationAtLanding,
      firstLandingTimeS: this.firstLandingTimeS,
      stability: this.stability,
    };
  }

  private transition(phase: StepPhase, timeS: number, reason: string): void {
    this.phase = phase;
    this.phaseStartedAtS = timeS;
    this.reason = reason;
  }

  observe(info: StepFeedback): void {
    if (![info.timeS, ...info.comWorldM, ...info.comVelocityWorldMps,
      info.pelvisHeightM, info.rootTiltDeg,
      ...info.swingSoleWorldM, ...info.supportSoleWorldM,
      info.supportLoadN, info.swingLoadN].every(Number.isFinite) ||
      info.supportLoadN < 0 || info.swingLoadN < 0 || info.rootTiltDeg < 0 ||
      typeof info.comProjectionInsideLoadedSupport !== "boolean" ||
      typeof info.capturePointInsideLoadedSupport !== "boolean" ||
      info.timeS < this.lastTimeS ||
      (info.swingSoleVelocityWorldMps !== undefined &&
        (!info.swingSoleVelocityWorldMps.every(Number.isFinite) ||
         info.swingSoleVelocityWorldMps.length !== 3)) ||
      (info.swingFrictionUtilization != null &&
        (!Number.isFinite(info.swingFrictionUtilization) ||
         info.swingFrictionUtilization < 0))) {
      throw new RangeError("Mediciones de paso inválidas o tiempo no monótono");
    }
    const sampleGapS = info.timeS - this.lastTimeS;
    this.lastTimeS = info.timeS;
    this.latestSwingLoadN = info.swingLoadN;
    if (!this.originalSole) this.originalSole = [...info.swingSoleWorldM] as XYZ;
    if (!this.originalSupportSole) {
      this.originalSupportSole = [...info.supportSoleWorldM] as XYZ;
    }
    this.minPelvisM = Math.min(this.minPelvisM, info.pelvisHeightM);
    this.maxRootTiltDeg = Math.max(this.maxRootTiltDeg, info.rootTiltDeg);
    const [x,y,z] = info.swingSoleWorldM;
    this.maxSupportFootSlipM = Math.max(this.maxSupportFootSlipM,
      Math.hypot(info.supportSoleWorldM[0]-this.originalSupportSole[0],
        info.supportSoleWorldM[1]-this.originalSupportSole[1]));
    this.targetErrorXYM = Math.hypot(
      x - (this.originalSole[0] + this.targetDeltaXY[0]),
      y - (this.originalSole[1] + this.targetDeltaXY[1]),
    );
    if (this.firstLandingSole && info.swingLoadN > 40) {
      this.maxContactSlipM = Math.max(this.maxContactSlipM,
        Math.hypot(x-this.firstLandingSole[0],y-this.firstLandingSole[1]));
    }
    const rise = z - this.originalSole[2];
    const travelX = x - this.originalSole[0];
    const travelY = y - this.originalSole[1];
    const travel = Math.hypot(travelX, travelY);
    const desired = Math.hypot(...this.targetDeltaXY);
    const progress = (travelX * this.targetDeltaXY[0] +
      travelY * this.targetDeltaXY[1]) / desired;
    const minProgress = Math.min(0.05, desired * 0.4);
    this.currentTargetProgressM = progress;
    this.maxTargetProgressM = Math.max(this.maxTargetProgressM, progress);
    this.maxSoleRiseM = Math.max(this.maxSoleRiseM, rise);
    this.maxSoleTravelXYM = Math.max(this.maxSoleTravelXYM, travel);
    const loadTotal = info.supportLoadN + info.swingLoadN;
    const supportFraction = loadTotal > 10 ? info.supportLoadN/loadTotal : 0;
    this.maxSupportLoadFraction = Math.max(this.maxSupportLoadFraction,supportFraction);
    if (this.phase === "FALL") return;
    if (info.pelvisHeightM < 0.65 || info.rootTiltDeg > 50) {
      this.transition("FALL",info.timeS,"Pelvis baja o tronco demasiado inclinado");
      return;
    }
    if (this.phase === "ABORT") return;
    if (["LIFT","SWING","LAND"].includes(this.phase) &&
        info.swingLoadN < 25 && info.swingSoleVelocityWorldMps) {
      this.lastUnloadedFootVelocityWorldMps =
        [...info.swingSoleVelocityWorldMps] as XYZ;
    }
    // Any foot which slides away after the first actual landing has failed
    // the planted-foot objective. This prevents spurious COMPLETE.
    if ((this.phase === "RECOVER" || this.phase === "COMPLETE") &&
      this.maxContactSlipM > 0.045) {
      this.transition("ABORT",info.timeS,"El pie apoyado se deslizó más de 4,5 cm");
      return;
    }
    // A transient impact does not establish a step. Require a continuous
    // interval of physically loaded bilateral support and a stationary COM.
    // Missing/late samples invalidate the interval rather than pretending
    // there was support throughout the gap.
    if (this.stepContactConfirmed) {
      this.stability=assessStepStability({
        comProjectionInsideLoadedSupport:info.comProjectionInsideLoadedSupport,
        capturePointInsideLoadedSupport:info.capturePointInsideLoadedSupport,
        comVelocityWorldMps:info.comVelocityWorldMps,
        pelvisHeightM:info.pelvisHeightM,
        rootTiltDeg:info.rootTiltDeg,
        targetErrorXYM:this.targetErrorXYM,
        maxSupportFootSlipM:this.maxSupportFootSlipM,
        maxLandingSlipM:this.maxContactSlipM,
        supportLoadN:info.supportLoadN,
        swingLoadN:info.swingLoadN,
      });
    }
    const stableNow=this.stability?.passes ?? false;
    if ((this.phase === "RECOVER" || this.phase === "COMPLETE") &&
        stableNow && sampleGapS <= 0.06) {
      this.settledSinceS ??= info.timeS;
      this.settledDurationS = info.timeS-this.settledSinceS;
    } else {
      this.settledSinceS = undefined;
      this.settledDurationS = 0;
      if (this.phase === "COMPLETE") {
        this.transition("ABORT",info.timeS,"El apoyo dejó de ser estable tras completarse");
        return;
      }
    }
    const elapsed = info.timeS - this.phaseStartedAtS;
    switch (this.phase) {
      case "STAND":
        if (info.timeS >= this.startTimeS) this.transition("SHIFT_WEIGHT",info.timeS,"Transferencia de peso inicial");
        break;
      case "SHIFT_WEIGHT":
        if (elapsed >= 0.25 && supportFraction >= 0.7 && info.swingLoadN < loadTotal * 0.3) {
          this.transition("LIFT",info.timeS,"Pie de apoyo con al menos 70 % de la carga");
        } else if (elapsed > 1.2) {
          this.transition("ABORT",info.timeS,"No consiguió transferir peso al apoyo");
        }
        break;
      case "LIFT":
        if (rise > 0.03 && info.swingLoadN < 25) {
          this.transition("SWING",info.timeS,"Despegue físico y descarga medidos");
        } else if (elapsed > 0.9) {
          this.transition("ABORT",info.timeS,"No despegó el pie libre");
        }
        break;
      case "SWING": {
        // Late-swing knee extension can lower the foot before the next
        // 20ms observation. A valid loaded touchdown after a verified lift
        // also counts as end of SWING; requiring >2cm clearance *at that
        // same instant* incorrectly marked such steps ABORT.
        const forwardArrival = travel > 0.055 && progress > minProgress * 0.8;
        const stillAirborne = rise > 0.012;
        const alreadyTouching = info.swingLoadN > 40;
        if (forwardArrival && (stillAirborne || alreadyTouching)) {
          this.transition("LAND",info.timeS,
            alreadyTouching ? "Vuelo completado: contacto físico anticipado" :
              "Avance medido hacia el objetivo y pie aún elevado");
        } else if (elapsed > 0.85) {
          this.transition("ABORT",info.timeS,"No avanzó físicamente hacia el destino");
        }
        break;
      }
      case "LAND":
        if (elapsed > 0.05 && progress > minProgress && rise < 0.02 &&
            info.swingLoadN > 40) {
          this.stepContactConfirmed = true;
          this.firstLandingSole = [...info.swingSoleWorldM] as XYZ;
          this.firstLandingVelocityEstimate = this.lastUnloadedFootVelocityWorldMps;
          this.frictionUtilizationAtLanding = info.swingFrictionUtilization ?? null;
          this.firstLandingTimeS = info.timeS;
          this.transition("RECOVER",info.timeS,"Aterrizaje con carga real verificado");
        } else if (elapsed > 0.9) {
          this.transition("ABORT",info.timeS,"Aterrizaje en dirección objetivo no confirmado");
        }
        break;
      case "RECOVER":
        if (progress > minProgress && this.settledDurationS >= 1.0) {
          this.transition("COMPLETE",info.timeS,
            "Apoyo bilateral y destino estables durante al menos un segundo");
        } else if (elapsed > 2.2) {
          this.transition("ABORT",info.timeS,
            "No sostuvo el equilibrio un segundo; fallan: "+
              (this.stability?.failures.join(", ") ?? "sin medidas"));
        }
        break;
    }
  }

  command(timeS: number, referenceComXY: XY): StepCommand {
    if (![timeS,...referenceComXY].every(Number.isFinite)) {
      throw new RangeError("Referencia de paso inválida");
    }
    const elapsed = Math.max(0,timeS - this.phaseStartedAtS);
    const supportSign = this.supportSide === "left" ? 1 : -1;
    const moveSign = this.swingSide === "left" ? 1 : -1;
    const gait = this.coordinatedGaitProfile > 0
      ? controlledStepPosture({
        phase:this.phase, elapsedS:elapsed,
        swingSide:this.swingSide, targetDeltaXY:this.targetDeltaXY,
        intensity:this.coordinatedGaitProfile,
      }) : null;
    const active = ["SHIFT_WEIGHT","LIFT","SWING","LAND"].includes(this.phase);
    const fraction = this.phase === "SHIFT_WEIGHT"
      ? Math.min(1,elapsed/0.35) : active ? 1 : 0;
    const weightShiftRef: XY = [
      referenceComXY[0]+(gait?.forwardComBiasM ?? 0),
      referenceComXY[1]+supportSign*0.075*fraction,
    ];
    let ref: XY = weightShiftRef;
    if (this.firstLandingSole && this.originalSole &&
      (this.phase === "RECOVER" || this.phase === "COMPLETE")) {
      // The support centroid changes when the swing foot lands at a new
      // XY location. A fixed pre-step COM reference pulled the body and
      // the planted foot backwards immediately after landing.
      const deltaX=this.firstLandingSole[0]-this.originalSole[0];
      const deltaY=this.firstLandingSole[1]-this.originalSole[1];
      const newStandingRef:XY=[
        referenceComXY[0]+0.5*deltaX,
        referenceComXY[1]+0.5*deltaY,
      ];
      const u=this.phase==="COMPLETE" ? 1 :
        Math.max(0,Math.min(1,elapsed/0.7));
      const w=u*u*(3-2*u);
      ref=[
        weightShiftRef[0]*(1-w)+newStandingRef[0]*w,
        (referenceComXY[1]+supportSign*0.075)*(1-w)+newStandingRef[1]*w,
      ];
    }
    const joints: Record<string,number> = {};
    if (["LIFT","SWING","LAND","RECOVER","COMPLETE"].includes(this.phase)) {
      const dx = this.targetDeltaXY[0],dy = this.targetDeltaXY[1];
      const lift = this.phase === "LIFT" || this.phase === "SWING";
      joints[`${this.swingSide}_hip_pitch`] = this.phase === "LIFT"
        ? -0.15 : Math.max(-0.5,Math.min(0.5,-dx / 0.65));
      joints[`${this.swingSide}_hip_roll`] = this.phase === "LIFT"
        ? moveSign * 0.09 : Math.max(-0.4,Math.min(0.4,dy / 0.65));
      joints[`${this.swingSide}_knee`] = lift ? -0.6 : -0.05;
      joints[`${this.swingSide}_ankle_pitch`] = lift ? 0.12 : 0;
    }
    // Only experimental gait mode replaces the leg pose and adds spine
    // commands. Neutral standing and legacy step behavior remain separate.
    if (gait) Object.assign(joints,gait.targetsRad);
    const t = (duration: number): number => {
      const u=Math.max(0,Math.min(1,elapsed/duration));
      return u*u*(3-2*u);
    };
    const initial=this.originalSole;
    let soleTargetWorldM: XYZ | null = null;
    let cartesianBlend=0;
    if (initial && ["LIFT","SWING","LAND"].includes(this.phase)) {
      const horizontal = this.phase==="LIFT" ? 0 :
        this.phase==="SWING" ? t(0.45) : 1;
      const vertical = this.phase==="LIFT" ? 0.095*t(0.3) :
        this.phase==="SWING" ? 0.095 :
        this.phase==="LAND" ? 0.095*(1-t(0.35)) : 0;
      soleTargetWorldM=[
        initial[0]+this.targetDeltaXY[0]*horizontal,
        initial[1]+this.targetDeltaXY[1]*horizontal,
        initial[2]+vertical,
      ];
      // Smooth engagement of the task-space controller on first lift.
      // Never keep pressing the planted foot with task-space swing torques.
      // During LAND, smoothly fade the swing motor as its normal load rises.
      cartesianBlend=this.phase==="LIFT" ? t(0.12) :
        this.phase==="SWING" ? Math.max(0,1-this.latestSwingLoadN/25) :
        Math.max(0,1-this.latestSwingLoadN/60);
    }
    return { phase:this.phase, referenceComXY:ref, targetsRad:joints,
      soleTargetWorldM, cartesianBlend };
  }
}

/** Overrides only named internal hinge DOFs. The standing baseline runs
 * first, so it remains responsible for stance and ankle feedback.
 */
export function applyStepJointTargets(
  model: NamedHingeModel, data: MuJoCoJointStateView,
  baseline: ReadonlyMap<number, JointPDSettings>,
  targetsRad: Readonly<Record<string,number>>,
): void {
  for (const [name,targetRad] of Object.entries(targetsRad)) {
    const joint = model.jnt(name).id;
    if (joint <= 0 || joint >= model.njnt || model.jnt_type[joint] !== 3) {
      throw new TypeError("Intento de actuar articulación no interna: " + name);
    }
    const q = model.jnt_qposadr[joint]!;
    const v = model.jnt_dofadr[joint]!;
    const settings = baseline.get(joint);
    if (!settings) throw new TypeError("Articulación sin par limitado: " + name);
    data.qfrc_applied[v] = limitedJointTorque(data.qpos[q]!,data.qvel[v]!,
      { ...settings,targetRad });
  }
  if (Array.from(data.qfrc_applied).slice(0,6).some(v => v!==0)) {
    throw new Error("El paso experimental intentó actuar sobre root/pelvis");
  }
}
