# MuJoCo WASM runtime (issue #4)

La primera versión utiliza el paquete **oficial** `@mujoco/mujoco@3.17.0` con binding **single-thread**; no necesita COOP/COEP ni `SharedArrayBuffer`.

## Arquitectura
- `src/physics/mujoco.ts`: carga dinámica de WASM usando un URL de asset gestionado por Vite (`mujoco.wasm?url`), una simulación a la vez por instancia, `loadModel`, `step`, `reset`, `state` y `dispose`.
- `src/physics/fixedClock.ts`: acumulador de timestep fijo independiente del framerate; limita pasos tras reanudar una pestaña.
- `src/physics/demoModel.ts`: esfera libre de 1 kg que cae desde 1,5 m sobre un plano. **No es un humanoide, controlador de equilibrio ni simulación de lesiones.**

## Uso previsto
```ts
import { MujocoSimulation } from "../src/physics/mujoco";
import { GRAVITY_DEMO_MJCF } from "../src/physics/demoModel";
const sim = await MujocoSimulation.create(GRAVITY_DEMO_MJCF);
try {
  const result = sim.step(100); // 100 * 0.002s = 0.2s
  console.log(result.time, result.qpos);
  sim.reset();
} finally {
  sim.dispose();
}
```

Siempre llamar `.delete()` sobre los handles de MuJoCo. La API devuelve **copias**, no vistas al heap WASM.

## Verificación
`npm run typecheck && npm test && npm run build`. Ya existe un test de integración en Node que ejecuta **MuJoCo WASM real**, validando caída, contacto con suelo y reset (`tests/unit/mujoco.smoke.test.ts`), junto a una prueba del reloj fijo. **Pendiente de validación:** comprobar la descarga y ejecución del `.wasm` en una página servida por Vite y en una compilación de producción mediante navegador real/Playwright. La compilación por sí sola no demuestra que el asset cargue en el navegador. Si el navegador no admite WASM, la UI presenta un error comprensible. La carga de modelos en memoria está diseñada para soportar reemplazo sin liberar la escena original hasta que la nueva sea válida.

## Límites
No medir 60 FPS en Android sin dispositivo real; no asegurar resultados idénticos entre navegadores/arquitecturas distintas. Motor, runtime y assets quedan fijados en `package-lock.json`.

Referencia de API: https://github.com/google-deepmind/mujoco/blob/main/wasm/README.md
