# Control de Uke: escalado de ganancias, actitud `tense` y posturas

Issue #75, PR 2. Código: `src/uke/gains.ts`, `src/uke/attitudes.ts`, `src/uke/stances.ts`; ejecutor: `src/scenario/run.ts`.
Pruebas: `tests/unit/ukeControl.test.ts`, `ukeStances.test.ts`, `ukeScenario.test.ts`. Escenarios: `scenarios/uke_stand_*.json`.

## Qué hace

- Actor `kind: "humanoid"` en el esquema de escenario ([SCENARIO_V1.md](SCENARIO_V1.md)): el ejecutor construye el modelo con
  `generateHumanoidMJCF` ([UKE_MODEL.md](UKE_MODEL.md)), coloca la postura, asienta 0,5 s con el controlador activo y llama a
  `UkeController.step(model, data, t)` antes de cada `mj_step` (precedido de `mj_subtreeVel`). Los pares van a `qfrc_applied` solo
  de bisagras internas; el ejecutor lanza un error si algún grado de libertad de la raíz recibe fuerza y registra el máximo en
  `summary.uke.max_root_generalized_force` (medido: 0 en todos los casos).
- `tense` = objetivos articulares de la postura (PD) + equilibrio de tobillo por pie + compensación de la gravedad en tronco, cuello y brazos.
- Veredicto (`src/metrics/outcome.ts`, `classifyHumanoidTrajectory`): `fall`, `stepped`, `standing` o `unresolved`, con métricas de
  equilibrio (`src/metrics/balance.ts`); definiciones en [RECORDING_V1.md](RECORDING_V1.md#veredicto-y-métricas-de-equilibrio-del-humanoide).
  `summary.uke` añade inclinación máxima del tronco, alturas de pelvis y
  centro de masas, velocidad del centro de masas al acabar el asentamiento y estado del controlador (par máximo y su razón con el límite).

## Ley de escalado final (`gains.ts`)

Con `m = masa/75`, `h = altura/1,76`, parte de `NEUTRAL_PD_GAINS` con la mitad de rigidez y amortiguación (como `makeStandingController`):

| Magnitud | Ley |
|---|---|
| Rigidez articular | `kp = kp0 · m · h` |
| Amortiguación articular | `kd = kd0 · m · h^1,5` |
| Par máximo | `max = max0 · strength · m^(2/3) · h` |
| Realimentación del centro de masas (extensión, no estaba en la issue) | `kpCom = 1200 · m`, `kdCom = 300 · m · h^0,5` |
| Límite de par de tobillo | `160 · strength · m^(2/3) · h` |

**La ley es la de la issue. La única diferencia es dónde se aplica la amortiguación:** toda la `kd` va al `damping` de cada articulación del
MJCF (que MuJoCo integra de forma implícita, `IMPLICIT_DAMPING_FRACTION = 1`) y la realimentación explícita de velocidad del PD queda a 0.

Por qué: la prueba de estabilidad por articulación (escalón de 0,2 rad con la raíz fijada) con la `kd` explícita **falló en las
cuatro tallas**: en tres, los tobillos (pie ligero, 1,1 kg) chasqueaban a 500 Hz (444 a 1342 cambios de signo en 3 s) y en 100 kg el hombro crecía,
la trampa que avisa la issue (`kd·dt/I > 1` con segmentos ligeros). Con la `kd` implícita el máximo es de 2 cruces en todas las tallas y la
velocidad pico decrece. Efecto colateral medido: con la `kd` implícita las posturas que antes caían en 100 y 150 kg (pie adelantado, base
ancha) se sostienen.

## Prueba de estabilidad por articulación

`ukeControl.test.ts`: en cada talla, modelo sin articulación libre (con la pelvis a 2,5 m, piernas colgando, excluidos los contactos de la
pelvis con sus hijos: sin articulación MuJoCo la funde con el mundo y deja de filtrarlos) y PD de las 28 bisagras; se da un escalón de 0,2 rad
(o −0,2 si el rango no lo admite) al objetivo de cada una, 3 s. Criterios: como máximo 3 cambios de signo del error respecto al valor final
(con banda muerta de 2 mrad) y que la velocidad pico de la ventana 2,25–3 s no supere la de 0,5–1,5 s. Medido tras el cambio: **máximo 2
cruces y velocidad pico decreciente (−5,1 a −6,5 mrad/s) en las cuatro tallas**.

## Posturas (`stances.ts`)

Tabla de ángulos; los tobillos se resuelven para que los pies queden planos y la pelvis se baja hasta que el pie más bajo toque el suelo.
La separación de los pies sale de los ángulos de cadera (no es un parámetro aparte). `neutral` (todo a 0), `natural_grip` (rodillas
flexionadas 0,25 rad, hombros 0,9 rad y codos 1,3 rad), `defensive` (caderas 0,3 rad, rodillas 0,6 rad, cadera abierta 0,22 rad, algo de
inclinación del tronco), `staggered_left` y `staggered_right` (un pie adelantado unos 0,3 m).

## Resultados de pie, 10 s (medidos)

`neutral`, `tense`, `strength` 1. Centro de masas: 58,1 % de la altura (55, 75 y 100 kg) y 58,5 % (150 kg).

| Talla | Masa total | CM / altura | Inclinación máx. del tronco | Pelvis mín. (% inicial) | Par máx. usado / límite | Asentamiento |
|---|---|---|---|---|---|---|
| 55 kg / 1,55 m | 55 kg | 58,1 % | 0,17° | 100,00 % | 0,2 N·m (cuello) / 114,6 N·m de tobillo | 0,15 cm/s |
| 75 kg / 1,75 m | 75 kg | 58,1 % | 0,16° | 100,00 % | 0,3 N·m / 159,1 N·m | 0,15 cm/s |
| 100 kg / 1,95 m | 100 kg | 58,0 % | 0,15° | 100,00 % | 0,4 N·m / 214,8 N·m | 0,16 cm/s |
| 150 kg / 2,00 m | 150 kg | 58,5 % | 0,14° | 100,00 % | 0,5 N·m / 288,6 N·m | 0,15 cm/s |

El par es minúsculo porque en `neutral` el centro de masas queda sobre los tobillos y los brazos cuelgan: el equilibrio casi no pide par.
Las posturas piden más (rodilla de 150 kg en `defensive`: 135 N·m, 34 % de su límite). Ningún par llegó al 100 % de su límite en estas
pruebas. Cinco posturas × cuatro tallas, medido con `ukeStances.test.ts`:

| Postura | Inclinación máx. del tronco (55 / 75 / 100 / 150 kg) | Pelvis mín. | Asentamiento |
|---|---|---|---|
| `natural_grip` | 1,7 / 1,7 / 1,6 / 2,9° | > 99,7 % | 1,1 / 1,2 / 1,2 / 2,02 cm/s |
| `defensive` | 3,6 / 4,0 / 4,3 / 3,7° (final 0,1–1,5°) | > 98,3 % | 4,7 / 4,9 / 5,2 / 6,6 cm/s |
| `staggered_left` y `_right` | 1,5 / 1,4 / 1,4 / 2,7° | > 99,8 % | 1,0 / 1,1 / 1,1 / 2,04 cm/s |

**Criterios de la issue que NO se cumplen:** `defensive` supera los 3° de tronco en un transitorio de los primeros segundos (acaba en
0,1–1,5°) y su asentamiento de 0,5 s acaba a 4,7–6,6 cm/s (límite 2 cm/s) en las cuatro tallas; 150 kg en `natural_grip` y
`staggered_*` acaba el asentamiento a 2,02–2,04 cm/s (por poco). Siguen de pie y estables. No se han relajado umbrales: esas combinaciones
están como `it.fails` en `ukeStances.test.ts`.

## Resultados negativos (no repetir)

- **Cambiar la fracción de rigidez del PD a 1** (`NEUTRAL_PD_GAINS` completo) en todas las articulaciones: caen las seis posturas
  probadas. Solo en cadera y rodilla: caen 75 y 100 kg en `neutral` y 150 kg se inclina 13°. Coincide con `STANDING_10S.md`: la mitad es necesaria.
- **Compensación de gravedad con `qfrc_bias` − `qfrc_constraint` en cadera y rodilla:** caen todas (realimentación positiva con la fuerza de
  contacto del paso anterior). La compensación con `qfrc_bias` solo en tronco, cuello y brazos sí funciona (en piernas cargaría la pelvis:
  con 150 kg la pelvis se inclinaba 5° hacia atrás y caía). Sin ella, `natural_grip` se inclinaba 19° (la flexión lumbar cede bajo la carga de los brazos).
- **Par máximo `∝ m·h` en vez de `m^(2/3)·h`:** no arregla las caídas de 100 y 150 kg y empeora 150 kg en `natural_grip`. Las caídas no
  eran por saturación sino por la `kd` explícita.
- El reparto igual del par de equilibrio entre los dos tobillos de `stepActiveStanding` falla con un pie adelantado: el pie delantero necesita el
  par opuesto al trasero. Se replica la parte mínima en `attitudes.ts` (cada pie recibe su fracción de la carga) en vez de modificar el baseline.
- Dos `contact exclude` en el generador (abdomen-muslos, pelvis-piernas): con 150 kg y las rodillas flexionadas la cápsula del abdomen
  chocaba con los muslos (10 kN).

## Actitud `relaxed` y amortiguación en tiempo de ejecución (issue #75, PR 3)

Código: `src/uke/damping.ts`, `createRelaxedController` en `src/uke/attitudes.ts`. Pruebas: `tests/unit/ukeRelaxed.test.ts`.

- `relaxed` no tiene objetivos ni equilibrio y no escribe ningún par: solo actúan la amortiguación articular pasiva y los topes de
  rango del MJCF (`limited="true"`). Amortiguación de partida: la de v2, 1,6 N·m·s/rad, escalada con la ley de `gains.ts`
  (`1,6 · m · h^1,5`, igual en todas las bisagras). Un cuerpo relajado no puede tener la amortiguación de uno tenso (`tense` a 75 kg y 1,76 m: de 6 N·m·s/rad en el
  cuello a 32 en la cadera y el tobillo).
- **La amortiguación se escribe en `model.dof_damping` en tiempo de ejecución** (`setJointDamping`), no solo en el MJCF. Comprobado
  con MuJoCo WASM real: el binding permite escribir `model.dof_damping` (la lectura devuelve lo escrito) y el cambio tiene efecto en la
  simulación (codo lanzado a 3 rad/s: tras 0,1 s queda a 0,171 rad/s con 1,6 y a 0,001 rad/s con 200). Los reflejos de caída podrán
  cambiar la amortiguación en marcha. El MJCF generado lleva también la amortiguación de la actitud (para que la grabación sea
  coherente) y el ejecutor la reescribe al empezar; con `tense` el valor es el mismo de siempre, así que los resultados no cambian
  (las pruebas de las cuatro tallas no se tocaron).
- Un Uke `relaxed` **asienta 0,5 s con la actitud `tense`** (si no, se desplomaría durante el asentamiento y el escenario empezaría ya cayendo)
  y pasa a `relaxed` (controlador y `dof_damping`) en `t = 0`.

### Pruebas de física pasiva (medidas)

| Prueba | Resultado |
|---|---|
| Energía (`mj_energyPos` + `mj_energyVel`, llamadas directas tras el `mj_forward` de cada muestra a 125 Hz; el flag de energía no hace falta) | `relaxed` 75 kg cae desde de pie: 747,3 J al inicio, 116,8 J al final; mayor subida sobre el máximo anterior 0,0000 % (límite 1 %) |
| Péndulo (cuerpo sin bisagras, colisiones desactivadas, ambos tobillos clavados al mundo con `connect`: un solo grado de libertad; sin amortiguación) | 5° a 45°: 0,9920 s simulado frente a 0,9924 s del péndulo físico (`I θ'' = m g d sin θ`, `I` = 81,94 kg·m² desde las inercias del modelo, `d` = 0,952 m, RK4): error 0,04 % (límite 10 %) |
| Simetría (`tense` 75 kg, 150 N·s hacia delante, medio seno de 0,1 s, en `sternum`) | desvío lateral máximo del CM 0,000 cm y giro máximo de la pelvis (ángulo entre su eje Y y el inicial) 0,000° durante los 3 s (límites 2 cm y 3°): el resultado es exactamente simétrico en Y |
| Robustez (mismo empujón con el punto desplazado ±1 cm en Y) | cae en los tres casos; dirección −0,0° centrado, 0,2° con +1 cm y −0,2° con −1 cm (límite ±15°); giro de pelvis 0,85° con +1 cm |

Los efectores admiten `site` (sitio con nombre del humanoide, p. ej. `sternum`) como alternativa a `body` + `local_point_m`
([SCENARIO_V1.md](SCENARIO_V1.md)). La prueba de simetría usa 75 kg / 1,75 m; no se ha probado en las otras tallas ni con posturas distintas de `neutral`.

## Qué no se ha comprobado

- Que el controlador aguante empujones: solo se ha medido el equilibrio sin perturbación. No hay reflejos de caída, golpes, agarres ni paso de recuperación (otras PR).
- Que el modelo parezca natural a un observador: ver la evidencia de la PR (hojas de fotogramas).
- La fuerza real de una persona: el factor `strength` solo escala el par máximo.
- El determinismo entre Node y navegador.
