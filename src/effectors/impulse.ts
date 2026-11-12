/**
 * Efector `impulse` (issue #75, PR 7): forma cómoda y validada de un `prescribed_force` de medio seno. No aplica fuerzas por su
 * cuenta: `impulseToPrescribedForce` lo traduce y el ejecutor (`src/scenario/run.ts`) usa la misma ruta de aplicación que para
 * cualquier fuerza prescrita (fuerza en `xfrc_applied` más el par `r × F` respecto al centro de masas del cuerpo).
 */
import type { ImpulseEffector, PrescribedForceEffector, Vec3 } from "../scenario/schema";

/**
 * Fuerza pico de un medio seno de duración `T` que entrega el impulso `J`: `∫0^T Fp sin(pi t / T) dt = 2 Fp T / pi = J`,
 * luego `Fp = pi J / (2 T)`.
 */
export function impulsePeakForceN(impulseNs: number, durationS: number): number {
  return (Math.PI * impulseNs) / (2 * durationS);
}

/** Vector unitario de `v` (no nulo y finito). */
export function unitVector(v: Vec3): Vec3 {
  const n = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / n, v[1] / n, v[2] / n];
}

/** Fuerza prescrita equivalente: mismo sitio, marco, instante y duración; `half_sine` con el pico de `impulsePeakForceN` en la dirección dada. */
export function impulseToPrescribedForce(spec: ImpulseEffector): PrescribedForceEffector & { max_force_n?: number } {
  const peak = impulsePeakForceN(spec.impulse_ns, spec.duration_s);
  const u = unitVector(spec.direction);
  const lowered: PrescribedForceEffector & { max_force_n?: number } = {
    id: spec.id,
    kind: "prescribed_force",
    actor: spec.actor,
    body: spec.body,
    site: spec.site,
    local_point_m: spec.local_point_m,
    frame: spec.frame,
    force_n: [u[0] * peak, u[1] * peak, u[2] * peak],
    start_s: spec.start_s,
    duration_s: spec.duration_s,
    waveform: "half_sine",
  };
  if (spec.max_force_n !== undefined) lowered.max_force_n = spec.max_force_n;
  return lowered;
}
