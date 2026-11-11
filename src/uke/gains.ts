/**
 * Ley de escalado de las ganancias de Uke por masa, altura y fuerza. Parte de `NEUTRAL_PD_GAINS`
 * (`src/control/standingBaseline.ts`, ajustadas para 75 kg y 1,76 m) con la misma mitad de rigidez y
 * amortiguación que usa `makeStandingController`. Justificación y mediciones: `docs/UKE_CONTROL.md`.
 *
 * Con `m = mass / 75` y `h = height / 1.76`:
 *  - rigidez articular       `kp  = kp0 · m · h`
 *  - amortiguación articular `kd  = kd0 · m · h^1.5`        (mantiene el amortiguamiento relativo)
 *  - par máximo              `max = max0 · strength · m^(2/3) · h`
 * Realimentación del centro de masas (tobillos), extensión de la misma ley (no está en la issue):
 *  - `kpCom = kpCom0 · m`, `kdCom = kdCom0 · m · h^0.5` y límite de tobillo como `max` de arriba.
 */
import { NEUTRAL_PD_GAINS, type JointGroup } from "../control/standingBaseline";
import { REFERENCE_HEIGHT_M, REFERENCE_MASS_KG } from "./generate";
import { UKE_HINGE_JOINTS } from "./stances";

/**
 * Fracción de la amortiguación `kd` que va al `damping` de la articulación en el MJCF (se integra de forma implícita)
 * en lugar de a la realimentación explícita de velocidad del PD. Con todo el `kd` explícito, el pie y el antebrazo
 * (segmentos ligeros) chasqueaban a 500 Hz en la prueba de escalón (`kd·dt/I > 1`); ver `docs/UKE_CONTROL.md`.
 */
export const IMPLICIT_DAMPING_FRACTION = 1;

export function jointGroupOf(name: string): JointGroup {
  const base = name.replace(/^(left|right)_/, "");
  if (/^(lumbar|thorax)/.test(base)) return "spine";
  if (base.startsWith("neck")) return "neck";
  if (/^(shoulder|elbow)/.test(base)) return "arm";
  if (base.startsWith("hip")) return "hip";
  if (base === "knee") return "knee";
  return "ankle";
}

/** Mitad de rigidez y amortiguación del PD, como en `makeStandingController`. */
const PD_FRACTION = 0.5;
/** Ganancias de la realimentación del centro de masas de `makeStandingController`. */
const COM_KP0 = 1200;
const COM_KD0 = 300;
const ANKLE_LIMIT0 = 160;

export interface GroupGains {
  readonly kp: number;
  readonly kd: number;
  readonly maxNm: number;
}

export interface UkeGains {
  readonly groups: Readonly<Record<JointGroup, GroupGains>>;
  readonly comKp: number;
  readonly comKd: number;
  readonly ankleLimitNm: number;
}

export function scaledGains(massKg: number, heightM: number, strength: number): UkeGains {
  if (![massKg, heightM, strength].every(Number.isFinite) || massKg <= 0 || heightM <= 0 || strength <= 0) {
    throw new RangeError("masa, altura y fuerza deben ser números positivos");
  }
  const m = massKg / REFERENCE_MASS_KG;
  const h = heightM / REFERENCE_HEIGHT_M;
  const maxScale = strength * m ** (2 / 3) * h;
  const groups = {} as Record<JointGroup, GroupGains>;
  for (const g of Object.keys(NEUTRAL_PD_GAINS) as JointGroup[]) {
    const g0 = NEUTRAL_PD_GAINS[g];
    groups[g] = {
      kp: g0.kp * PD_FRACTION * m * h,
      kd: g0.kd * PD_FRACTION * m * h ** 1.5,
      maxNm: g0.maxNm * maxScale,
    };
  }
  return {
    groups,
    comKp: COM_KP0 * m,
    comKd: COM_KD0 * m * h ** 0.5,
    ankleLimitNm: ANKLE_LIMIT0 * maxScale,
  };
}

/** `damping` por articulación para el MJCF (parte implícita de `kd`). */
export function ukeJointDamping(massKg: number, heightM: number, strength: number): Record<string, number> {
  const gains = scaledGains(massKg, heightM, strength);
  return Object.fromEntries(UKE_HINGE_JOINTS.map((name) => [
    name, gains.groups[jointGroupOf(name)].kd * IMPLICIT_DAMPING_FRACTION,
  ]));
}
