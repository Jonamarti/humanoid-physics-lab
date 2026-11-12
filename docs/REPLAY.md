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

## Galería de grabaciones de ejemplo

Para revisar los resultados desde el móvil, sin tener la grabación en el dispositivo, la web incluye una galería: junto a **Cargar grabación** aparece el selector **Grabaciones de ejemplo**. Al elegir una se descarga (`.json.gz`, decenas de kB) y se reproduce con el mismo código que una grabación cargada a mano (`ReplayView.load`); se muestran su título, su descripción y su veredicto (resultado, dirección y si cumple `expect`). Si `recordings/index.json` no existe o falla la descarga, la web sigue funcionando y el selector lo dice («No disponible») con el motivo.

**Enlace directo.** `?rec=<id>` abre esa grabación nada más cargar, por ejemplo `https://jonamarti.github.io/humanoid-physics-lab/?rec=uke_tense_75_175_push_sternum`. Al elegir una grabación, la barra de direcciones se actualiza con ese parámetro y el bloque de la galería muestra el «Enlace para compartir». Un id que no está en el índice se avisa y no se carga nada.

**Dónde está cada cosa.** Las grabaciones de referencia se confirman en git en `public/recordings/` (`<id>.json.gz` más `index.json`) y Vite las copia a `dist/recordings/`; se piden con una ruta relativa a la base de Vite (`import.meta.env.BASE_URL`, `/humanoid-physics-lab/` en Pages). La lista de lo que se publica es `scenarios/gallery/gallery.json` (no está en `scenarios/` a propósito: `npm run sim -- --batch scenarios` y `tests/unit/scenarios.test.ts` leen todos los `.json` de esa carpeta como escenarios).

| Módulo | Función |
|---|---|
| `src/recording/gallery.ts` | Sin DOM: validación del índice (`hpl.gallery.v1`), URL con base, `?rec=`, descarga con `fetch`, veredicto legible. |
| `src/recording/galleryBuild.ts` | Solo Node: lee `gallery.json`, ejecuta cada escenario con `runScenario`, escribe las grabaciones y el índice. |
| `scripts/build_gallery.ts` | Punto de entrada de `npm run gallery` (mismo mecanismo que `sim`: `vite build --ssr` y `--configLoader runner`). |
| `src/renderer/galleryView.ts` | Interfaz (selector, descripción, veredicto, enlace, mensajes). `main.ts` solo la instancia. |

### Añadir una grabación a la galería

1. Elige un escenario de `scenarios/` (por ejemplo `scenarios/mi_escenario.json`, con su `expect`).
2. Añade una entrada a `scenarios/gallery/gallery.json`: `{ "id": "mi_escenario", "title": "…", "description": "Una o dos frases en español." }`. El `id` es el del escenario y solo admite letras, cifras, «_» y «-». Opcional: `"record_hz": 50` abarata **solo la copia de la galería** (debe dividir 500; el escenario no se toca).
3. Ejecuta `npm run gallery` desde la raíz (en esta máquina, `npm.cmd run gallery`). Reescribe `public/recordings/`: una `.json.gz` por entrada, `index.json` y borra las que ya no están en la lista.
4. Confirma `scenarios/gallery/gallery.json` y todo `public/recordings/`. Sin marcas de tiempo ni rutas: mismo escenario y mismo motor dan los mismos bytes, así que un cambio en el diff es un cambio real.

Hay que volver a ejecutar `npm run gallery` siempre que cambie la física, un escenario de la lista o la versión de `package.json` (queda en `meta.engine_version`). `tests/unit/galleryPublished.test.ts` lo detecta: lee cada grabación con `parseRecording`, la compara con su entrada del índice y con `gallery.json`, limita el conjunto a 1 MB y regenera dos grabaciones (un cilindro y un humanoide) para compararlas con las confirmadas. La comparación es primero byte a byte; si los bytes difieren (otra versión de Node/zlib u otra plataforma) se comparan los JSON descomprimidos con tolerancia numérica (1e-4 absoluta + 1e-3 relativa; `qvel` y la lista de contactos no se comparan elemento a elemento, porque un contacto puede cruzar el umbral de 1e-6 N en otro entorno).

Tamaño actual (5 grabaciones, versión 0.1.0): 34,4 + 33,7 + 195,3 + 175,3 + 141,2 kB = 580 kB comprimidos (más el índice, ~2,6 kB; 582 518 bytes en `public/recordings/`, bien por debajo del límite de 1 MB). `summary.artifacts` y `diagnostics` (#75 PR 5) añadieron entre 0,2 y 0,6 kB a cada una. Dos de las cinco (`uke_stand_75_175_neutral`, 10 s a 50 Hz; y `uke_relaxed_75_175_collapse`) concentran el 64 %. Las tres del humanoide crecieron un 6-9 % con el equilibrio por muestra (`com`, `xcom`, `support_polygon`; #75 PR 4); las del cilindro no cambian.
