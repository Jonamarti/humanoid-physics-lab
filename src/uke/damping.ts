/**
 * Amortiguación articular de Uke por actitud, escrita en `model.dof_damping` EN TIEMPO DE EJECUCIÓN.
 *
 * El `damping` de una articulación del MJCF lo integra MuJoCo de forma implícita (`integrator="implicitfast"`), así que
 * es la forma estable de dar amortiguación a segmentos ligeros (ver `docs/UKE_CONTROL.md`). Cada actitud tiene la suya:
 * `tense` usa toda la `kd` de `gains.ts`; `relaxed` solo la amortiguación pasiva baja de v2 (1,6 N·m·s/rad) escalada con
 * la misma ley (`kd ∝ m · h^1,5`). Escribir `dof_damping` en marcha (comprobado en `tests/unit/ukeRelaxed.test.ts`: el
 * binding de WASM lo permite y tiene efecto en la simulación) deja cambiar la actitud sin recompilar el modelo; lo usarán
 * los reflejos de caída.
 */
import type { UkeAttitude } from "../scenario/schema";
import { REFERENCE_HEIGHT_M, REFERENCE_MASS_KG } from "./generate";
import { ukeJointDamping } from "./gains";
import { UKE_HINGE_JOINTS } from "./stances";

/** `damping` por defecto de las articulaciones de v2, en N·m·s/rad. */
export const RELAXED_BASE_DAMPING_NMS_PER_RAD = 1.6;

/** Amortiguación pasiva de `relaxed` por articulación: `1,6 · m · h^1,5` (misma ley que `kd` en `gains.ts`). */
export function relaxedJointDamping(massKg: number, heightM: number): Record<string, number> {
  if (![massKg, heightM].every(Number.isFinite) || massKg <= 0 || heightM <= 0) {
    throw new RangeError("masa y altura deben ser números positivos");
  }
  const value = RELAXED_BASE_DAMPING_NMS_PER_RAD * (massKg / REFERENCE_MASS_KG) * (heightM / REFERENCE_HEIGHT_M) ** 1.5;
  return Object.fromEntries(UKE_HINGE_JOINTS.map((name) => [name, value]));
}

/** `damping` por articulación de una actitud. */
export function attitudeJointDamping(
  attitude: UkeAttitude, massKg: number, heightM: number, strength: number,
): Record<string, number> {
  return attitude === "relaxed" ? relaxedJointDamping(massKg, heightM) : ukeJointDamping(massKg, heightM, strength);
}

/** Lo mínimo de un `MjModel` que necesita `setJointDamping`. */
export interface DampingModel {
  dof_damping: Float64Array;
  jnt_dofadr: ArrayLike<number>;
  jnt(name: string): { id: number };
}

/**
 * Escribe `model.dof_damping` por nombre de bisagra. Rechaza nombres que no sean bisagras (la raíz libre no se toca) y
 * valores no finitos o negativos.
 */
export function setJointDamping(model: DampingModel, byName: Readonly<Record<string, number>>): void {
  const hinges = new Set(UKE_HINGE_JOINTS);
  for (const [name, value] of Object.entries(byName)) {
    if (!hinges.has(name)) throw new Error(`"${name}" no es una bisagra de Uke: no se escribe su amortiguación`);
    if (!Number.isFinite(value) || value < 0) throw new RangeError(`amortiguación inválida para ${name}: ${value}`);
    model.dof_damping[model.jnt_dofadr[model.jnt(name).id]!] = value;
  }
}
