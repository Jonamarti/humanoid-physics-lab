# Evidencia: dos humanoides articulados

Base: PR #106, commit 1d10c0643a038534d82941c1cfeda7e1b01e87c8. Huella de fuentes normalizadas: `c79159c5f616c688d4ffb34860433aa84d2a444c700d1377ec7b4b5aa47e6027`. Manifiesto: [validation.json](validation.json).

- [metrics.csv](metrics.csv): 24 filas (12 escenas, ambos actores), incluidos los casos defectuosos.
- [metrics.json](metrics.json): controladores, eventos de agarre, trabajos, diagnósticos y resultados por actor.
- [browser-check.json](browser-check.json): selección de Tori, pausa/tiempo/avance y capas en escritorio/móvil emulado.
- [mutation-check.json](mutation-check.json): fallan las pruebas al quitar la reacción física o borrar los pares del otro actor.

La suite completa da 736 correctas, 12 omitidas y 3 pendientes. El banco manual se ejecutó aparte: 12 escenas como prueba de observación, sin ocultar defectos físicos. Typecheck, lint, build y siete pruebas del render pasan. La CLI estricta devuelve 4 por los defectos de Tori mientras Uke sigue de pie.

Vídeos, hojas, grabaciones y logs completos quedan localmente en `out/articulated/`; el índice está en `out/articulated/delivery/index.html`. No se suben grabaciones ni vídeos a Git. Frontal y lateral de carga y caída pasiva, más oblicua del agarre, primero x1 y después x0,25. La vista lateral de la pareja lado a lado tiene oclusión, por eso se añadió la oblicua. Los picos del vídeo usan muestras a 50 Hz; los de la tabla se miden a 500 Hz (el pico de agarre de 11,04 N no queda entero en el muestreo visual).

Se revisaron hojas frontal/oblicua de carga, frontal de caída y capturas del navegador; no todos los fotogramas de los vídeos. La caída pasiva sigue dando 10,94 kN y 1,28 cm de penetración. La protección opcional la empeora a 31,21 kN y 1,86 cm. No se han relajado los umbrales.

Ver [ARTICULATED_ACTORS.md](../../ARTICULATED_ACTORS.md) para arquitectura, reproducción y siguiente hito.
