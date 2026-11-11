# Grabación `hpl.recording.v1` y ejecutor de escenarios

Issue #74, PR 3. Tipos: `src/recording/schema.ts`. Ejecutor: `src/scenario/run.ts` (`runScenario(scenario)`).
Serialización: `src/recording/write.ts`. Veredicto: `src/metrics/outcome.ts`; comparación con `expect`: `src/scenario/expect.ts`.
El ejecutor solo funciona en Node (usa `node:crypto` y carga MuJoCo WASM una vez por proceso, `src/scenario/mujocoModule.ts`).

## Qué hace el ejecutor

1. Construye el MJCF del escenario (`src/scenario/buildModel.ts`): suelo, props (cajas fijas, un cuerpo estático por `id`)
   y el actor (cilindro libre, cuerpo `cylinder`). Opciones de `worldOptionTag()` más `ccd_tolerance="1e-14" ccd_iterations="100"`
   (ver «Simetría»). Parámetros de contacto de las geometrías: los de `humanoid_step_v2.xml`
   (`solref="0.015 1" solimp="0.9 0.95 0.001"`), rozamiento de `ground.friction` en suelo, props y actor. Solo se admite un actor.
2. Asienta 0,5 s sin efectores. **El tiempo del escenario empieza después** (`t = 0`).
3. Bucle a 500 Hz. Antes de cada `mj_step`: `xfrc_applied` a cero y, por cada efector, fuerza en el mundo y par `r × F`
   respecto al centro de masas del cuerpo. La fuerza del paso `[t, t + dt]` se evalúa en el punto medio.
   Marcos: `world`; `actor_initial` (ejes del actor en `t = 0`, tras el asentamiento); `actor_current` (ejes en ese paso).
4. Muestrea cada `500 / record_hz` pasos: `qpos`, `qvel`, efectores y contactos (tras `mj_forward`, con el wrench ya aplicado).
   La última muestra es el instante `duration_s`.
5. A 500 Hz acumula `peaks` (no a `record_hz`): módulo máximo, impulso `∫ F dt` y trabajo `∫ F · v_punto dt`
   (velocidad del punto desde `cvel` y regla del trapecio en cada paso).

## Contenido

- `meta`: `scenario_id`, `scenario_sha256` (del escenario validado), `engine_version` (`package.json`), `mujoco_version`,
  `physics_hz`, `record_hz`, `frame` (`x_forward_y_left_z_up`), `quat_order` (`wxyz`).
- `model`: `mjcf` completo, `bodies` y `joints` en el orden de índices de MuJoCo. Cuerpos: `world`, los props por `id` y `cylinder`.
- `samples[]`: `t`, `qpos`, `qvel`, `effectors[] {id, point_m, force_n}`, `contacts[] {body_a, body_b, point_m, force_n}`.
  La fuerza de contacto se decodifica de `efc_force` (`src/physics/contactForces.ts`) y se rota al mundo con el marco del contacto
  (convención de MuJoCo: fuerza de `body_a` sobre `body_b`; comprobado: la suma sobre el cilindro en reposo es su peso). Solo
  contactos con fuerza normal > 1e-6 N. El suelo es el cuerpo `world`.
- `peaks.effectors.<id>`: `max_force_n`, `impulse_ns`, `work_j`.
- `summary`: `outcome`, `direction_deg` (o `null` con inclinación final < 5°), `t_outcome_s` (cuándo adoptó su valor final y
  ya no cambió, a 500 Hz), `final_tilt_deg`, `max_tilt_deg` (a 500 Hz), `final_com_height_m`, `slide_m`,
  `expect` (`pass`, `fail` o `none`) y `expect_diffs`.

Veredicto del cilindro: inclinación del eje < 15° `standing`; > 60° `fall`; en otro caso `tilted`. Dirección: hacia donde apunta el eje
(0 = +X, 90 = +Y).

En memoria (`Recording`) los números conservan toda la precisión; al escribir (`serializeRecording`, `gzipRecording`) se redondean
a 6 cifras significativas y se comprime con gzip nivel 9 sin marca de tiempo (mismo resultado byte a byte en el mismo entorno).

## Simetría

La prueba de simetría (reflejar fuerza y obstáculo en Y debe reflejar `qpos`) falló al principio con error 1,1e-4 a 1 s en los
casos con obstáculo, y 1e-13 sin obstáculo. La causa es la tolerancia de la colisión convexa de MuJoCo (caja-cilindro,
`ccd_tolerance` por defecto 1e-6). Con `ccd_tolerance="1e-14" ccd_iterations="100"` el error baja a ~6e-8, por debajo del 1e-6 exigido.
La tolerancia de la prueba no se ha relajado.

## Límites

- Determinismo exacto solo dentro de Node, no entre Node y navegador.
- Un único actor y solo fuerzas prescritas; sin humanoide, agarres ni controladores (issues #75 y #76).
