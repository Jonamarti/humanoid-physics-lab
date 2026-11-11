# Calibración del contacto (prueba de la rampa)

Parte de la issue #74 (PR 1) y del plan de derribos (`docs/PLAN_DERRIBOS.md`, sección 12.1, fallo 6).
Todo lo medido aquí se obtuvo con MuJoCo WASM (`@mujoco/mujoco`) ejecutado en Node mediante
`tests/unit/contactRamp.test.ts`, el 11-11-2026.

## Qué se mide

Un bloque cúbico de 75 kg (arista 0,4 m) reposa sobre un plano inclinado un ángulo `θ`. Plano y bloque tienen
el mismo rozamiento deslizante `µ` (torsional 0,005; de rodadura 0,0001) y los parámetros `solref="0.015 1"`
y `solimp="0.9 0.95 0.001"` de `humanoid_step_v2.xml`. Un bloque rígido con rozamiento de Coulomb no debería
moverse mientras `θ < atan(µ)`. Se simulan 2 s de asentamiento y se mide la velocidad media a lo largo de la
bajada entre t = 2 s y t = 3 s.

Se comparan dos conjuntos de opciones:

- **v2 (antiguo)**: `<option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>` (cono piramidal por defecto).
- **Nuevo** (`src/scenario/worldOptions.ts`): `timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="10"`.

Las opciones nuevas son **solo para los modelos nuevos de escenarios**. No se ha modificado ningún XML de
`public/models/`.

## Resultados (WASM, velocidad a lo largo de la bajada, mm/s)

| µ | ángulo | v2 (piramidal) | nuevo (elíptico, impratio 10) |
|---|---|---|---|
| 0,5 | atan(µ)−8° = 18,57° | 1,168 | 0,062 |
| 0,8 | atan(µ)−8° = 30,66° | 3,990 | 0,143 |
| 1,0 | atan(µ)−8° = 37,00° | 6,275 | 0,194 |
| 0,5 | atan(µ)+5° = 31,57° | 2385,8 | 2394,3 |
| 0,8 | atan(µ)+5° = 43,66° | 3126,1 | 3829,0 |
| 1,0 | atan(µ)+5° = 50,00° | 7587,8 | 6797,1 |
| 1,0 | 30° | 2,542 | 0,122 |
| 1,0 | 40° | 8,065 | 0,224 |

Lectura:

- Con las opciones nuevas, por debajo del ángulo de rozamiento la fluencia es de 0,06–0,19 mm/s, por debajo del
  umbral de la issue (0,5 mm/s), y por encima desliza a más de 2 m/s (umbral: 100 mm/s). Ambas condiciones se
  afirman en la prueba, sin relajar el umbral y sin necesidad de `noslip_iterations`.
- Con las opciones antiguas de v2, el bloque fluye entre 1,2 y 6,3 mm/s por debajo del ángulo de rozamiento (20 a
  30 veces más que con las nuevas). Estas cifras **solo se anotan**; la prueba no las afirma. Son evidencia para
  la issue #49 (deslizamiento de suelas); que la causa del deslizamiento observado allí sea esta es una hipótesis
  no comprobada.
- La referencia medida en MuJoCo nativo (issue #74) daba 2,5 mm/s a 30° y 6,7 mm/s a 40° con v2. En WASM salen
  2,54 mm/s a 30° (coincide) y 8,07 mm/s a 40° (algo más; la ventana y el tamaño del bloque pueden diferir de
  la medición nativa, que no se ha reproducido aquí). Con las opciones nuevas, 0,12 y 0,22 mm/s frente al
  intervalo nativo de 0,10–0,15 mm/s: sube algo a 40°, pero queda por debajo de 0,5 mm/s.
- No se probó `noslip_iterations`: no hizo falta para cumplir el umbral.

## Coste en tiempo

10 s de simulación (5000 pasos de 2 ms) del humanoide `humanoid_step_v2.xml` pasivo (sin controlador, cae al
suelo), en WASM sobre Node, cambiando solo la etiqueta `<option>`. Dos pasadas tras un calentamiento:

| opciones | pasada 1 | pasada 2 |
|---|---|---|
| v2 antiguo | 358 ms | 348 ms |
| nuevo | 384 ms | 402 ms |

El coste del cono elíptico es aproximadamente un 10–15 % mayor en esta medición (poca muestra, en un único PC;
la medición es de orden de magnitud). 10 s del humanoide pasivo cuestan unos 0,35–0,40 s en WASM, apto para
búsquedas por simulación (plan, fallo 24).

## Qué no se ha comprobado

- El efecto de las opciones nuevas sobre el humanoide con controlador (por eso no se tocan los modelos actuales).
- Otras geometrías de contacto (cápsulas, cilindros, contactos en línea) ni cargas distintas de 75 kg.
- Que el deslizamiento de suelas de la issue #49 se deba al cono piramidal.
