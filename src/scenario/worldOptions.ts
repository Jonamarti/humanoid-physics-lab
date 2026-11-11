/**
 * Opciones de MuJoCo (`<option>`) para los modelos NUEVOS de escenarios.
 *
 * No se usan en `humanoid_v1.xml`, `humanoid_step_v2.xml` ni `humanoid_gait_v3.xml`:
 * esos modelos conservan sus opciones para no romper las pruebas existentes.
 * Las mediciones que justifican estos valores están en `docs/CONTACT_CALIBRATION.md`.
 */

/** Paso físico en segundos (500 Hz). */
export const PHYSICS_TIMESTEP_S = 0.002;

/** Frecuencia física en Hz, derivada del paso. */
export const PHYSICS_HZ = 500;

/** Valor de `impratio` (relación de impedancia fricción/normal) del cono elíptico. */
export const WORLD_IMPRATIO = 10;

/** Atributos de `<option>` de los modelos de escenarios, sin las etiquetas. */
export const WORLD_OPTION_ATTRS =
  `timestep="${PHYSICS_TIMESTEP_S}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="${WORLD_IMPRATIO}"`;

/** Cadena `<option .../>` lista para insertar en un MJCF de escenario. */
export function worldOptionTag(extraAttrs = ""): string {
  return `<option ${WORLD_OPTION_ATTRS}${extraAttrs ? ` ${extraAttrs}` : ""}/>`;
}
