/**
 * Módulo MuJoCo WASM para el ejecutor de escenarios (solo Node). Se carga UNA vez por proceso:
 * cada llamada a `loadMujoco()` crea un entorno nuevo y pierde memoria.
 * No usa `src/physics/mujoco.ts` (arrastra el visor y los controladores).
 */
import loadMujoco from "@mujoco/mujoco";

export type MujocoModule = Awaited<ReturnType<typeof loadMujoco>>;

let modulePromise: Promise<MujocoModule> | undefined;

export function getMujoco(): Promise<MujocoModule> {
  if (!modulePromise) {
    modulePromise = loadMujoco().catch((error: unknown) => {
      modulePromise = undefined; // permite reintentar tras un fallo transitorio
      throw error;
    });
  }
  return modulePromise;
}
