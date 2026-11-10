import type { XYZ } from "./balanceSensors";

/** Independent observable conditions for accepting a *measured* landed step.
 * A passing unit test suite never implies all of these are physically met.
 */
export const STEP_STABILITY_CHECKS = [
  "comProjection", "capturePoint", "comSpeed", "pelvisHeight",
  "trunkTilt", "targetError", "supportFootDrift", "landingFootDrift",
  "supportLoad", "landingLoad",
] as const;

export type StepStabilityCheck = typeof STEP_STABILITY_CHECKS[number];

export interface StepStabilityInputs {
  readonly comProjectionInsideLoadedSupport: boolean;
  readonly capturePointInsideLoadedSupport: boolean;
  readonly comVelocityWorldMps: XYZ;
  readonly pelvisHeightM: number;
  readonly rootTiltDeg: number;
  readonly targetErrorXYM: number;
  readonly maxSupportFootSlipM: number;
  readonly maxLandingSlipM: number;
  readonly supportLoadN: number;
  readonly swingLoadN: number;
}

export interface StepStabilityAssessment {
  readonly passes: boolean;
  readonly comSpeedXYMps: number;
  readonly checks: Readonly<Record<StepStabilityCheck,boolean>>;
  readonly failures: readonly StepStabilityCheck[];
}

/** Stable-step acceptance is intentionally strict and unchanged from the
 * existing ExperimentalStepSequence, but all failed gates are observable.
 * These are engineering acceptance criteria, not biological safety limits.
 */
export function assessStepStability(input:StepStabilityInputs):StepStabilityAssessment {
  const values=[...input.comVelocityWorldMps,
    input.pelvisHeightM,input.rootTiltDeg,
    input.targetErrorXYM,input.maxSupportFootSlipM,input.maxLandingSlipM,
    input.supportLoadN,input.swingLoadN];
  if(!values.every(Number.isFinite) ||
    input.comVelocityWorldMps.length!==3 ||
    input.rootTiltDeg<0 || input.targetErrorXYM<0 ||
    input.maxSupportFootSlipM<0 || input.maxLandingSlipM<0 ||
    input.supportLoadN<0 || input.swingLoadN<0 ||
    typeof input.comProjectionInsideLoadedSupport!=="boolean" ||
    typeof input.capturePointInsideLoadedSupport!=="boolean") {
    throw new RangeError("Estabilidad: observaciones físicas inválidas");
  }
  const speed=Math.hypot(
    input.comVelocityWorldMps[0],input.comVelocityWorldMps[1]);
  const checks:Readonly<Record<StepStabilityCheck,boolean>>={
    comProjection:input.comProjectionInsideLoadedSupport,
    capturePoint:input.capturePointInsideLoadedSupport,
    comSpeed:speed<0.12,
    pelvisHeight:input.pelvisHeightM>0.82,
    trunkTilt:input.rootTiltDeg<10,
    targetError:input.targetErrorXYM<=0.05,
    supportFootDrift:input.maxSupportFootSlipM<=0.02,
    landingFootDrift:input.maxLandingSlipM<=0.02,
    supportLoad:input.supportLoadN>40,
    landingLoad:input.swingLoadN>40,
  };
  const failures=STEP_STABILITY_CHECKS.filter(check=>!checks[check]);
  return {passes:failures.length===0,comSpeedXYMps:speed,checks,failures};
}

export const STEP_STABILITY_LABELS: Readonly<Record<StepStabilityCheck,string>>={
  comProjection:"COM fuera del apoyo",
  capturePoint:"punto de captura fuera del apoyo",
  comSpeed:"centro de masas aún en movimiento",
  pelvisHeight:"pelvis demasiado baja",
  trunkTilt:"inclinación excesiva",
  targetError:"error de colocación del pie",
  supportFootDrift:"deslizamiento del pie de apoyo",
  landingFootDrift:"deslizamiento del pie recién apoyado",
  supportLoad:"pierna de apoyo descargada",
  landingLoad:"pie recién apoyado sin carga",
};
