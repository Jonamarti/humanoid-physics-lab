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

## Humanoide

Con un actor `humanoid` el cuerpo principal es `pelvis` y el veredicto sale de `classifyHumanoidTrajectory` (`src/metrics/outcome.ts`; ver
la sección siguiente). `final_com_height_m` es la altura
del centro de masas de todo el cuerpo. `summary.uke` (opcional, solo humanoide): `max_trunk_tilt_deg`, `initial_pelvis_height_m`,
`min_pelvis_height_m`, `settle_com_speed_mps`, `initial_com_height_m`, `max_root_generalized_force`, `controller` (`UkeController.status()`)
y los bloques de equilibrio `balance`, `fall`, `step` y `unbalance`. Detalle del control en [UKE_CONTROL.md](UKE_CONTROL.md).

## Veredicto y métricas de equilibrio del humanoide

Issue #75, PR 4. Funciones puras (sin MuJoCo) en `src/metrics/balance.ts` y `src/metrics/outcome.ts`; el ejecutor solo las alimenta.
Suelo en `z = 0`. Todo lo siguiente es opcional en la grabación: **una grabación anterior sin estos campos se sigue leyendo** (el lector
solo exige `summary.outcome` como cadena) y el cilindro no los lleva.

### Por muestra (`samples[]`, solo humanoide)

| Campo | Contenido |
|---|---|
| `com` | Centro de masas de todo el actor `[x, y, z]`, en m (`subtree_com` de la pelvis). |
| `xcom` | Punto de captura `xi = c + v / w0` en el plano `[x, y]`, con `w0 = sqrt(g / z_c)`, `g = 9,81`, `z_c` altura del centro de masas (mínimo 0,1 m) y `v` su velocidad (`subtree_linvel`). |
| `support_polygon` | Envolvente convexa antihoraria de los puntos de contacto suelo-pie con carga normal > 20 N (componente vertical de la fuerza del contacto), `[[x, y], ...]` en m. Cuenta cada contacto por separado, no cada pie. Vacío si ningún contacto supera 20 N; con una línea de contactos (talón levantado) sale un segmento de dos puntos. Los vértices alineados (desvío < ~3 nm) se descartan. |
| `capture_margin_m` | Margen con signo de `xcom` al polígono: distancia al borde más cercano, positiva dentro y negativa fuera. Con un polígono de uno o dos puntos nunca está «dentro» (valor `<= 0`); `null` con el polígono vacío. |

### Resultado (`summary.outcome` del humanoide)

Orden de evaluación: `fall`, después `standing` o `stepped`, y si no, `unresolved`.

- **`fall`**: cabeza (`head`), pecho (`chest`), abdomen (`abdomen`) o `pelvis` tocan el suelo (a 500 Hz), o la pelvis baja del 50 % de su altura
  inicial (a `record_hz`). Se cumple en cualquier instante y no se revierte.
- **Estable**: en una muestra, `capture_margin_m >= 0` y velocidad del centro de masas < 0,1 m/s. «Se mantiene estable» = lo es en **todas las
  muestras de los últimos 2 s** de la grabación.
- **`stepped`**: la suela de algún pie (sitios `left_sole` / `right_sole`) se ha desplazado en el plano horizontal más de 5 cm de su posición
  inicial en algún momento (aunque vuelva), han pasado al menos 2 s desde el primer instante en que lo hizo y se mantiene estable.
- **`standing`**: ningún pie se movió más de 5 cm y se mantiene estable (la grabación dura al menos 2 s).
- **`unresolved`**: nada de lo anterior al acabar el tiempo (por ejemplo, grabación de menos de 2 s, o todavía oscilando).

`summary.t_outcome_s`: la caída (`fall.t_s`); en `standing` o `stepped`, el inicio del tramo estable final (en `stepped`, no antes del paso);
en `unresolved`, el final. `summary.direction_deg` (mundo, 0 = +X): en caída, `uke.fall.direction_world_deg`; si no, la dirección del tronco en su
inclinación máxima (`null` si < 5°). `expect.outcome` admite ahora también `stepped` y `unresolved`; el cilindro conserva `standing`, `tilted` y `fall`.

**Interpretaciones** (la issue es literal en «2 s después»): se exige estabilidad sostenida durante los últimos 2 s en lugar de comprobarla en un
solo instante, y el desplazamiento de un pie se mide en el máximo a lo largo de la grabación, no solo al final. Quitar `tilted` del humanoide
sustituye al veredicto mínimo anterior (`classifyHumanoid`, retirado): «de pie» ya no pide tronco < 3°.

### `summary.uke`

- `balance`: `min_capture_margin_m`, `final_capture_margin_m` (`null` si ninguna muestra / la última no tiene apoyo), `final_com_speed_mps`,
  `max_foot_displacement_m {left, right}` y `stable_since_s` (inicio del tramo estable que llega hasta el final, o `null`).
- `fall` (`null` si no cae): `t_s`, `trigger` (`contact:<cuerpo>` o `pelvis_height`), `direction_deg` (respecto al frente inicial de Uke, 0 = delante,
  +90 = izquierda, 180 = atrás; la del eje del tronco `chest` en la primera muestra desde `t_s` en que se inclina ≥ 5°, `null` si nunca),
  `direction_world_deg`, `direction_name` (una de 8), `first_part` y `first_part_t_s` (primera parte que no es un pie en tocar el suelo),
  `contact_order` (primeros contactos de cada parte con el suelo, en orden) e `impact` (`head`, `chest`, `pelvis`: `t_s`, `speed_mps`,
  `normal_speed_mps`, o `null` si esa parte no toca). El cuerpo `chest` hace de «espalda»: no hay un cuerpo de espalda aparte. La velocidad es
  la del punto de contacto (con rotación) en el paso anterior al contacto, a 500 Hz.
- `step` (`null` si ningún pie se mueve más de 5 cm): `t_s` y `foot` (`left` / `right`) del primer desplazamiento.
- `unbalance` (`null` si el punto de captura nunca sale del apoyo): `t_s` (primera muestra con margen negativo o sin apoyo), `direction` y
  `direction_deg` (una de 8, múltiplo de 45° respecto al frente inicial; la del vector desde el centroide del último polígono en que estuvo dentro
  hasta `xcom`; con el talón levantado el polígono se reduce a una línea y su centroide salta, por eso no se usa el actual), `min_margin_m` y
  `recovered` (¿vuelve a estar dentro en la última muestra?).

Medido (12-11-2026, Uke `tense`, 75 kg, 1,75 m, impulso de medio seno de 0,1 s en `sternum` hacia atrás): 6 N·s `standing`; 25 N·s `fall`
hacia atrás con `unbalance` hacia atrás; el umbral queda entre 12 y 13 N·s. `tense` no da pasos: `stepped` solo está probado con trayectorias sintéticas.

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
(0 = +X, 90 = +Y). Se mide cuando el eje cruza los 60° y no al final porque un cilindro tumbado tiene el mismo
aspecto con la tapa hacia +X o hacia -X: en el escenario de 80 cm, que pasa por encima del obstáculo, la dirección del
eje final salía a 180° aunque la caída empezó hacia +X.

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
