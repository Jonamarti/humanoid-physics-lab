# Reproducción de grabaciones en el visor web (PR 7 de #74)

El visor web puede reproducir una grabación `hpl.recording.v1` (`recording.json` o `recording.json.gz`, ver [RECORDING_V1.md](RECORDING_V1.md)) generada con `npm run sim`. **No ejecuta física**: nunca llama a `mj_step`.

## Uso

1. Genera una grabación: `npm run sim -- scenarios/cylinder_obstacle_low.json --out out/cyl_low` (o usa `tests/fixtures/cylinder_obstacle_low.recording.json.gz`).
2. Abre la web (`npm run dev`, o la página publicada) y pulsa **Cargar grabación** (debajo de los botones del laboratorio). Elige el `.json` o `.json.gz`.
3. Se entra en el modo reproducción: se desactivan el selector de escenario y «Cargar simulación», empieza a reproducirse y aparecen el escenario (`meta.scenario_id`) y el resultado (`summary.outcome`, dirección y número de muestras).
4. Controles: **Reproducir/Pausa**, **×1**, **×0,5**, **×0,25**, **Anterior** y **Siguiente** (saltan de muestra en muestra y pausan), barra de tiempo (al arrastrarla se pausa) y **Salir del modo reproducción**, que devuelve al laboratorio.
5. Cada efector con fuerza distinta de cero se dibuja con la flecha del visor (`forceArrow.ts`) y un rótulo arriba a la izquierda del visor con su `id` y los newtons del instante.

## Cómo funciona

| Módulo | Función |
|---|---|
| `src/recording/read.ts` | `parseRecording(texto \| bytes)`: acepta `.json` y `.json.gz` (`DecompressionStream` en navegador y Node >= 18; `zlib` de respaldo), valida `schema === "hpl.recording.v1"` y los campos mínimos. Versión desconocida → error claro. |
| `src/recording/player.ts` | `RecordingPlayer`, clase pura (sin Three.js, DOM ni MuJoCo): `sampleAt(t)`, `seek`, `advance`, `play/pause`, `setSpeed`, `stepNext/stepPrev`. Interpolación lineal de posiciones y ángulos y **slerp con corrección de signo** de los cuaterniones (`quaternionStarts` los localiza con `jnt_type`). En el instante de una muestra devuelve exactamente esa muestra; `t` fuera de rango se recorta. Los contactos no se interpolan (se toma la muestra anterior). |
| `src/recording/replayKinematics.ts` | Carga `model.mjcf` en MuJoCo WASM **solo** para geometrías y cinemática directa: escribe `qpos`, anula `qvel` y llama a `mj_forward`. |
| `src/renderer/replayView.ts` | Interfaz y bucle de animación del modo reproducción. `main.ts` solo lo instancia (13 líneas). |

Flujo por fotograma: `RecordingPlayer.current()` → `ReplayKinematics.setQpos()` (`mj_forward`) → `BallViewer.setArticulatedPoses()` y `setMultiForces()`.

## Notas y límites

- Las flechas usan la escala no lineal de `forceArrowLengthM` (la del laboratorio), no «1 m = 1000 N» del vídeo en Python: la dirección es exacta, la longitud es indicativa. El valor exacto está en el rótulo. Observado: `BallViewer.refreshMultiForceVisuals` normaliza el vector antes de pedir su longitud, así que las flechas múltiples del visor miden siempre ~0,25 m (defecto previo del visor, no tocado aquí).
- Se interpola `qpos` (posiciones, ángulos y cuaterniones), no `qvel`. Las fuerzas de efectores se interpolan linealmente entre muestras: es solo visual.
- El modelo de la grabación se vuelve a compilar en el navegador; si el MJCF usa mallas externas, no se verán (los escenarios actuales usan primitivas).
- Pruebas: `tests/unit/recordingPlayer.test.ts`, `recordingRead.test.ts` y `recordingReplayMujoco.test.ts` (WASM real: `mj_forward` desde el `qpos` grabado da las mismas `xpos` que la simulación, error < 1e-6; con la grabación sin redondear, < 1e-9). Grabación de referencia: `tests/fixtures/cylinder_obstacle_low.recording.json.gz` (~34 kB).
- La interfaz no tiene prueba automática de navegador; ver el estado de la verificación visual en la PR.
