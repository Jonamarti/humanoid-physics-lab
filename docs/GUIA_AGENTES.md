# Guía rápida para agentes

Una página. Antes de tocar nada, lee el [plan de derribos](PLAN_DERRIBOS.md), sobre todo las secciones 9 (reglas) y 12 (revisión crítica). Trabajo en curso: issues #73 a #80, etiquetas `derribos` y `fase-N`. Las issues con etiqueta `aparcada` no se tocan.

## Entorno en esta máquina (Windows)

Rutas relativas a la raíz del repositorio; la caché de npm está en la carpeta hermana `../humanoid-physics-lab-npm-cache`.

```sh
npm.cmd ci --cache ../humanoid-physics-lab-npm-cache
npm.cmd run typecheck
npm.cmd run lint
npm.cmd test -- --configLoader runner --maxWorkers=2
npm.cmd run build -- --configLoader runner
```

- Con Vite y Vitest hay que pasar siempre `--configLoader runner`: esbuild no puede recorrer el directorio de usuario en este sandbox.
- Un ensayo manual concreto: `npm.cmd exec -- vitest run <archivo> --configLoader runner --maxWorkers=1 --no-file-parallelism`.
- `npm ci` borra paquetes instalados con `--no-save` (por ejemplo Playwright); reinstálalos si los necesitas.
- Usa cachés y carpetas temporales fuera del directorio de usuario y del repositorio. Hay poco disco: borra los fotogramas intermedios tras codificar y verificar un vídeo.

## Vídeo de una grabación (sin navegador)

Detalles en [VIDEO.md](VIDEO.md). Comando comprobado en esta máquina (no hay que instalar nada global):

```sh
uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow python scripts/render_recording.py out/cyl_low/recording.json.gz out/cyl_low/video.mp4 --view side --sheet out/cyl_low/hoja.png
```

Prueba del script: `uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow python scripts/test_render_recording.py`. Grabación de ejemplo: `scripts/make_sample_recording.py`.

## Reglas (sección 9 del plan)

- CI en verde no es prueba de física: la prueba son las métricas y el vídeo.
- No relajar un umbral para que algo pase; si un criterio es incorrecto, se discute y se cambia aparte.
- Un cambio por comparación, siempre contra la misma batería de casos.
- Los resultados negativos se registran una vez y no se repiten.
- Nada de barridos ciegos de ganancias: antes de un barrido, escribir la hipótesis y qué resultado la refutaría.
- Sin fuerzas artificiales sobre Uke fuera de los efectores declarados.
- Este repositorio es público: sin nombres de técnicas en el núcleo, sin datos ni modelos de personajes privados, sin código copiado del repositorio consumidor.
- Cada lección que afecte a los dos repositorios se anota en ambos.

## Paquete de evidencias (sección 8)

Cada PR que cambie física entrega:

1. Vídeo frontal y lateral, a velocidad real y a ×0,25, con flechas de fuerza, centro de masas y apoyo.
2. Hoja de fotogramas clave en una imagen.
3. Tabla de métricas, con los casos que fallan.
4. Tres o cuatro frases: qué se ve bien, qué no es natural y qué no se ha comprobado.

## Arquitectura de carpetas (sección 12.6)

```
src/scenario/    esquema, validación, intención → escenario, ejecutor
src/effectors/   mano virtual, obstáculo, ariete, presupuesto de Tori
src/uke/         generador MJCF, actitudes, reflejos de caída, paso
src/metrics/     equilibrio, resultado, artefactos
src/recording/   grabador, esquema de grabación, lectura
scenarios/       escenarios JSON con su expectativa
scripts/         sim.mjs (comando), render_recording.py (vídeo)
```

## Flujo de PR

- Rama desde `origin/main`, un tema por PR. El repositorio fusiona automáticamente las PR que no son borrador, sin la etiqueta `no-merge` y con CI en verde ([AUTO_MERGE.md](AUTO_MERGE.md)): abre como borrador cualquier PR que aún no deba fusionarse.
- No cambies `.github/workflows/` sin fusión humana.
- Más contexto: [CONTRIBUTING.md](../CONTRIBUTING.md) y [ARCHITECTURE_BOUNDARY.md](ARCHITECTURE_BOUNDARY.md).
