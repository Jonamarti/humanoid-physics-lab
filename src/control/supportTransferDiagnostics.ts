/** Exact, observational pre-lift gates shared by the v3 physical FSM and
 * the opt-in 50 Hz recovery benchmark. No actuator commands or threshold
 * tuning belong here. An instantaneously true gate is NOT a verified lift.
 */
export interface SupportTransferInput {
  readonly supportLoadN: number;
  readonly swingLoadN: number;
  readonly comVelocityWorldMps: readonly [number, number, number];
  readonly comProjectionInsideLoadedSupport: boolean;
  readonly capturePointInsideLoadedSupport: boolean;
  readonly pelvisHeightM: number;
  readonly rootTiltDeg: number;
}

export interface SupportTransferReadiness {
  readonly totalLoadN: number;
  readonly supportFraction: number;
  readonly comSpeedMps: number;
  readonly gates: Readonly<{
    supportLoad: boolean;
    swingUnloaded: boolean;
    comProjection: boolean;
    capturePoint: boolean;
    pelvisHeight: boolean;
    rootTilt: boolean;
    comSpeed: boolean;
  }>;
  /** True for the instantaneous sample only; the FSM also requires dwell. */
  readonly ready: boolean;
}

/** Preserve the existing voluntary and reactive v3 release conditions.
 * The reactive emergency deliberately does not require capture inside the
 * old support polygon; it DOES require physically solved foot loads.
 */
export function assessSupportTransfer(
  info: SupportTransferInput, reactiveEmergencyStep: boolean,
): SupportTransferReadiness {
  const totalLoadN = info.supportLoadN + info.swingLoadN;
  const supportFraction = totalLoadN > 10 ? info.supportLoadN / totalLoadN : 0;
  const comSpeedMps = Math.hypot(
    info.comVelocityWorldMps[0], info.comVelocityWorldMps[1],
  );
  const gates = {
    supportLoad: reactiveEmergencyStep
      ? totalLoadN > 250 && supportFraction >= 0.72
      : supportFraction >= 0.80,
    swingUnloaded: info.swingLoadN <
      totalLoadN * (reactiveEmergencyStep ? 0.28 : 0.20),
    comProjection: info.comProjectionInsideLoadedSupport,
    capturePoint: reactiveEmergencyStep || info.capturePointInsideLoadedSupport,
    pelvisHeight: !reactiveEmergencyStep || info.pelvisHeightM > 0.82,
    rootTilt: !reactiveEmergencyStep || info.rootTiltDeg < 25,
    comSpeed: comSpeedMps < (reactiveEmergencyStep ? 1.2 : 0.18),
  };
  return {
    totalLoadN, supportFraction, comSpeedMps, gates,
    ready: Object.values(gates).every(Boolean),
  };
}
