# Detector de artefactos

Issue #75, PR 5; sección 12.5 del [plan](PLAN_DERRIBOS.md). Toda grabación se comprueba automáticamente: si algo falla, no es candidata para enseñarse. Código: `src/metrics/artifacts.ts` (evaluación, función pura sobre una grabación), `src/scenario/artifactProbe.ts` (medidas por paso físico, solo Node) y la integración en `src/scenario/run.ts`. Los umbrales son puntos de partida a calibrar con los primeros clips y **no se relajan para que un escenario pase**.

## Resultado

- `summary.artifacts`: lista de artefactos. **Vacía = limpio en todo lo evaluado.** Cada uno: `type`, `t_s`, `where` (cuerpo, par de cuerpos, articulación o efector), `value` (medido), `threshold` (umbral superado) y `detail` (texto).
- `summary.artifacts_not_evaluated`: detectores que **no pudieron evaluarse** y el motivo (`type`, `reason`). Un detector que no se evalúa nunca cuenta como limpio: «limpio» solo se puede afirmar si esta lista no contiene nada relevante.
- `diagnostics` (en la raíz de la grabación): las medidas por paso físico (picos con su instante y elemento) que el detector necesita y no se pueden sacar de las muestras. Los umbrales viven en el detector, no en la grabación, así que se puede reevaluar con otros umbrales sin volver a simular.
- Una grabación anterior (sin `diagnostics` ni `summary.artifacts`) se sigue leyendo y reproduciendo; `detectArtifacts` sobre ella devuelve lista vacía y todo «no evaluado» (`tests/unit/artifacts.test.ts` lo comprueba con la grabación de referencia del cilindro).

`npm run sim -- <escenario> --strict` (o con `--batch`) sale con **código 4** si hay algún artefacto. Sin `--strict` los artefactos solo se imprimen y se guardan (`batch.csv` lleva una columna final `artifacts`). Ninguna prueba existente usa `--strict`.

### Códigos de salida (decisión)

El encargo pedía el 3, pero en `scripts/sim.ts` el **3 ya significa «el ejecutor lanzó una excepción»** y `GUIA_AGENTES.md` lo documentaba. Reutilizarlo habría hecho indistinguible un fallo del motor de una grabación con artefactos, y habría cambiado un contrato que otros pueden estar usando. Por eso: **0** correcto, **1** `expect` incumplido, **2** escenario inválido o uso incorrecto, **3** excepción del ejecutor (sin cambios), **4** `--strict` y hay artefactos. Precedencia: 2 > 1 > 4 (con `--strict` solo se convierte en 4 lo que habría sido 0; un `expect` incumplido sigue dando 1 y los artefactos se imprimen igualmente).

## Detectores, datos y dónde se miden

«Por paso» = a 500 Hz dentro del ejecutor (cada `mj_step`, desde `t = 0`, sin el asentamiento); «por muestra» = en cada muestra de `record_hz` tras `mj_forward`. Un pico puede perderse entre muestras, así que lo que es un pico se mide por paso. Medido en los 13 escenarios (ver tabla): el pico por paso fue **hasta 1,9 veces mayor** que el máximo de las muestras en la fuerza de contacto (`cylinder_three_phase`: 22 040 N por paso frente a 11 380 N en las muestras) y 2,5 % mayor en la velocidad articular (`uke_tense_75_175_push_sternum`: 14,59 frente a 14,24 rad/s). Con `record_hz` de 50 (los escenarios de pie) sería peor.

| Tipo | Umbral | Medida | Datos | Cuándo |
|---|---|---|---|---|
| `ground_penetration` | > 1 cm | `-dist` del contacto entre una geometría del actor y el suelo (`floor`) | contactos de MuJoCo | por paso |
| `segment_penetration` | > 2 cm | `-dist` entre geometrías de dos cuerpos distintos del actor | contactos de MuJoCo | por paso |
| `joint_speed` | > 25 rad/s | máx. \|`qvel`\| de las articulaciones internas (bisagras del actor; no la raíz libre) | `qvel` | por paso |
| `contact_force` | > 10 kN | fuerza normal de un contacto del actor (suelo, props o él mismo), de `efc_force` | solución del solver | por paso |
| `foot_slip` | > 2 cm | ver abajo | contactos, `cvel` | por paso |
| `tremor` | > 10 cambios de sentido en 1 s | ver abajo | `qpos` | por paso |
| `energy_gain` | exceso > 1 % de `m·g·z_cm` | ver abajo | `mj_energyPos/Vel`, `qfrc_applied`, trabajo de efectores | energía por muestra; trabajo por paso |
| `effector_over_limit` | `max_force_n` declarado | máx. de la fuerza del efector (`peaks.effectors.<id>.max_force_n`, 500 Hz) | `peaks` y `limit_n` | por paso |
| `root_force` | > 1e-9 | máx. \|`qfrc_applied`\| en los 6 grados de la raíz, en cada paso | `qfrc_applied` | por paso |

Los contactos que se leen tras `mj_step` son los del estado al inicio del paso (resueltos con las fuerzas de ese paso): el mismo estado en que se mide la velocidad articular.

### Pie que patina («sin que el escenario lo provoque»)

Un pie está **cargado** si la suma de las fuerzas normales de sus contactos con el suelo supera el **20 % del peso del actor** (el plan no lo numera: elección de esta PR; no es el umbral de 20 N del polígono de apoyo, que no escala con la masa). Mientras el pie está cargado, el desplazamiento horizontal neto de sus puntos de contacto (media ponderada por la carga de la velocidad del punto de contacto, integrada en el tiempo) se acumula en un tramo continuo; el tramo se reinicia si el pie deja de estar cargado o si el rozamiento se satura. El rozamiento está **saturado** si algún contacto del pie tiene utilización |F_t| / (μ·F_n) > 0,9 (`frictionUtilization` de `decodeSolvedContactForce`; sin dato cuenta como saturado). Definición: **si el rozamiento está saturado, la fuerza (la del escenario o el propio movimiento del cuerpo) vence al rozamiento y el deslizamiento es físico; si el contacto dice «pegado» y aun así el punto se mueve, es fluencia numérica del contacto blando**, que es lo que se busca (la prueba de la rampa del plan 12.1 midió 2,5 mm/s). No se mira si hay un efector sobre el pie: un tirón lateral por debajo del rozamiento que mueva el pie también sería un artefacto. Medir en el punto de contacto y no en la suela evita contar como patinazo el giro del pie sobre el talón.

### Temblor

Se sigue `qpos` de cada bisagra con histéresis: un cambio de sentido solo cuenta si la articulación recorre al menos `TREMOR_MIN_AMPLITUDE_RAD` = **0,005 rad (0,29°)** en sentido contrario desde el último extremo. Se cuenta el máximo de cambios de sentido en cualquier ventana de 1 s. **La amplitud no la fija el plan: es una elección de esta PR**, y el resultado depende de ella. Sensibilidad medida (máximo de cambios de sentido por segundo, escenarios humanoides, `src/scenario/artifactProbe.ts` con la amplitud cambiada):

| Amplitud (rad) | 0,0002 | 0,0005 | 0,001 | 0,002 | 0,005 | 0,01 |
|---|---|---|---|---|---|---|
| `uke_relaxed_75_175_collapse` | 22 | 10 | 10 | 9 | 7 | 7 |
| `uke_tense_75_175_push_sternum` | 28 | 18 | 10 | 8 | 6 | 6 |
| de pie (6 escenarios) | 1-2 | 1 | 0-1 | 0-1 | 0-1 | 0-1 |

Elegí 0,005 rad como la amplitud visible (0,29° son ~2 mm en la punta de un antebrazo) y no por los resultados; observa, sin embargo, que **con amplitudes de 0,001 rad o menos las dos caídas sí superarían el umbral de 10**: hay una vibración de menos de 0,06° en las articulaciones durante las caídas que este detector no considera temblor. Se anota como hallazgo, no se oculta. Con 0,002 rad el resultado era 9 y 8, a un paso del umbral: 0,005 da más margen. Limitación probada (`artifactProbeMujoco.test.ts`): una oscilación de 0,002 rad a 20 Hz no se cuenta.

### Energía (decisión)

La energía mecánica del sistema `E = cinética + potencial` (`mj_energyPos` + `mj_energyVel`, que incluye los resortes de MuJoCo; no incluye la energía elástica de los contactos blandos ni de los topes articulares) no puede subir por encima de lo aportado: `exceso_k = (E_k − E_0) − (W_pares,k + W_efectores,k)`. `W_pares` es el trabajo con signo de los pares internos, `Σ τ·q̇ dt` por paso con `qfrc_applied` (regla del trapecio con la velocidad antes y después del paso), y `W_efectores` es la suma de `peaks.effectors.*.work_j` (`∫ F·v_punto dt`, 500 Hz). El amortiguamiento y los contactos solo restan energía, así que sin una fuente oculta el exceso nunca debería ser positivo salvo la tolerancia.

**Por qué vale también para Uke `tense` y no solo para `relaxed`:** como el trabajo de los pares se cuenta *con signo* (positivo cuando empujan en el sentido del movimiento, negativo cuando frenan), un `tense` cuyos pares aportan energía no dispara el detector: esa energía aparece en `W_pares`. Lo que dispara es energía que no sale de ningún par, efector ni de la condición inicial. No hace falta limitarse a `relaxed`, y el detector es el mismo para el cilindro (sin pares, solo efectores). Verificado con MuJoCo real (`artifactProbeMujoco.test.ts`): un efector de 200 N que sube un cuerpo 57 J no dispara si se declara su trabajo y sí (energía de +57 J sin explicar) si no se declara; un par de 3 N·m en una rodilla (+14 J) no dispara y su trabajo calculado aparte coincide con el aumento de energía en un 4 %; un salto de velocidad de 3 m/s que nadie declara sí dispara. **En los escenarios existentes el término de los pares no se ejercita** (su trabajo neto es ~0: en `uke_tense_75_175_push_sternum`, quitarlo del balance solo sube el exceso máximo de 0 a 0,00005 J), así que esa parte solo está comprobada por la prueba de juguete. Quitar el trabajo de los efectores sí hace saltar el detector en los escenarios reales (250 J en el empujón al esternón, 34 J en `cylinder_obstacle_low`).

**Tolerancia:** el exceso puede llegar al `ENERGY_GAIN_TOLERANCE_FRACTION` = **1 %** de `m·g·z_cm` (energía potencial del centro de masas del actor tras el asentamiento; ~7 J para 75 kg de pie), la misma regla del 1 % de la prueba de energía de `relaxed`. Cubre la energía elástica de los contactos al liberarse y el residuo del integrador (el trabajo por trapecio no es exacto cuando cambia la matriz de inercia: 4 % del trabajo en la prueba del par de 3 N·m a 2 ms). Medido en los 13 escenarios: el mayor exceso es 0,17 J (`cylinder_obstacle_high`, tolerancia 6,4 J); los humanoides dan ≤ 1e-6 J. **Límite**: con trabajos de pares de varios cientos de julios, un 4 % de residuo podría rozar la tolerancia (falso positivo); no ocurre con los escenarios actuales y se revisará con los primeros derribos reales. La energía se evalúa solo en las muestras (necesita un `mj_forward` coherente): una ganancia que aparezca y desaparezca entre muestras no se vería.

### Fuerza sobre la raíz y topes de efectores

- `root_force`: **solo** lo que el controlador escribe en `qfrc_applied` de los 6 primeros grados de libertad (debe ser 0; el ejecutor ya aborta si no lo es: la medida del detector es independiente y llega al fichero). Una fuerza de un efector declarado aplicada a la pelvis **no** es un artefacto: es un efector (la regla del plan es «sin fuerzas fuera de los efectores declarados»).
- `effector_over_limit`: compara el pico a 500 Hz con `limit_n`, que el ejecutor copia del campo `max_force_n` del efector **si lo tiene**. Hoy el esquema de escenarios no lo admite (lo traerán los efectores `impulse` y `ram` de la PR 7 y la mano virtual al integrarla): hasta entonces todos los efectores de los escenarios están «sin tope declarado» y el detector lo dice. `t_s` es el instante de la muestra con más fuerza (aproximado: el pico a 500 Hz puede caer entre muestras).

## Cuándo un detector NO se evalúa

Se informa en `artifacts_not_evaluated`:

| Situación | Detectores |
|---|---|
| Grabación sin `diagnostics` (anterior a esta PR) | todos los de `diagnostics` |
| Actor sin articulaciones internas (cilindro) | `joint_speed`, `tremor` |
| Actor de un solo cuerpo (cilindro) | `segment_penetration` |
| Actor sin pies (cilindro) | `foot_slip` |
| Actor que no es humanoide | `root_force` |
| Efector sin `max_force_n` | `effector_over_limit`, con los `id` |
| Grabación de menos de 1 s | `tremor` (la ventana es de 1 s) |
| No se midió la energía | `energy_gain` |

Huecos conocidos, **no** marcados como no evaluados porque el detector sí corre pero no lo cubre: (1) contactos de un par de cuerpos excluidos en el MJCF (abdomen-muslo y pelvis-espinilla, `src/uke/generate.ts`) y los pares padre-hijo, que MuJoCo no colisiona: no pueden dar penetración medida; (2) cilindro contra un obstáculo no cuenta como «segmentos», pero su fuerza de contacto sí entra en `contact_force`; (3) bisagras solamente: sin articulaciones esféricas ni deslizantes (el humanoide actual solo tiene bisagras).

## Resultados en los escenarios actuales (12-11-2026)

`npm run sim -- --batch scenarios` (13 escenarios, un único proceso, entre 0,1 y 1,1 s cada uno). Lo medido; **no se ha tocado ningún umbral, `expect` ni la física**:

| Escenario | Resultado | Artefactos | Penetración suelo (cm) | Fuerza contacto (N) | Vel. articular (rad/s) | Temblor (cambios/s) | Patinazo pie (mm) | Exceso energía (J) |
|---|---|---|---|---|---|---|---|---|
| `cylinder_no_obstacle` | fall | `contact_force`, `ground_penetration` | **2,63** | **19 460** | n/e | n/e | n/e | 0,04 |
| `cylinder_obstacle_above_com` | standing | ninguno | 0,05 | 4 833 | n/e | n/e | n/e | 0,08 |
| `cylinder_obstacle_high` | fall | `contact_force`, `ground_penetration` | **3,56** | **23 180** | n/e | n/e | n/e | 0,17 |
| `cylinder_obstacle_low` | fall | `contact_force`, `ground_penetration` | **3,03** | **21 760** | n/e | n/e | n/e | 0,04 |
| `cylinder_three_phase` | fall | `contact_force`, `ground_penetration` | **2,84** | **22 040** | n/e | n/e | n/e | ~0 |
| `uke_relaxed_75_175_collapse` | fall | `ground_penetration` | **1,21** (pecho) | 7 835 | 11,3 | 7 | 2,5 | ~0 |
| `uke_stand_55_155_neutral` | standing | ninguno | 0,06 | 100 | 0,004 | 0 | 0,001 | 0 |
| `uke_stand_75_175_neutral` | standing | ninguno | 0,06 | 136 | 0,004 | 0 | 0,001 | 0 |
| `uke_stand_75_175_defensive` | standing | ninguno | 0,07 | 156 | 0,13 | 1 | 2,6 | 0 |
| `uke_stand_75_175_natural_grip` | standing | ninguno | 0,06 | 127 | 0,03 | 1 | 0,006 | 0 |
| `uke_stand_100_195_neutral` | standing | ninguno | 0,07 | 181 | 0,003 | 0 | 0,001 | 0 |
| `uke_stand_150_200_neutral` | standing | ninguno | 0,07 | 272 | 0,003 | 0 | 0,0004 | 0 |
| `uke_tense_75_175_push_sternum` | fall | `contact_force`, `ground_penetration` | **1,95** (pecho) | **15 490** | 14,6 | 6 | 6,7 | 0 |

(n/e = no evaluado; `segment_penetration` no supera 1,2 cm en ningún caso, el máximo es 1,196 cm entre `chest` y `right_thigh` en el colapso relajado.)

**Hallazgo (no es un fallo del detector):** los 5 cilindros que caen y los dos humanoides que caen (el relajado y el empujado) superan al menos un umbral en el instante en que el cuerpo golpea el suelo; los 6 de pie, y el cilindro que no vuelca, salen limpios. Es el contacto blando de los modelos de escenarios (`solref="0.015 1"`, `solimp="0.9 0.95 0.001"`): un impacto de un cuerpo de 75 kg se frena con 2-3,5 cm de penetración y 15-23 kN, valores que un suelo real no daría. O el criterio de 1 cm / 10 kN del plan es demasiado estricto para impactos de caída, o el contacto del suelo es demasiado blando para caídas: es una decisión de Jon y no se ha cambiado nada aquí. Hasta entonces, ninguna caída de estas puede salir con `--strict` en verde.

## Cómo medirlo de nuevo

```sh
npm run sim -- --batch scenarios --out out/batch      # imprime artefactos y no evaluados de cada escenario
npm run sim -- scenarios/x.json --out out/x --strict  # código 4 si hay artefactos
```

Pruebas: `tests/unit/artifacts.test.ts` (evaluación sobre grabaciones sintéticas: una que dispara y otra que no por detector, y los casos no evaluados), `tests/unit/artifactProbeMujoco.test.ts` (las medidas por paso con MuJoCo real sobre modelos de juguete), `tests/unit/artifactsRun.test.ts` (ejecutor completo) y `tests/unit/simCommand.test.ts` (`--strict` y códigos de salida).

## Efectores `ram` (PR 7)

Un ariete es un cuerpo libre con energía cinética inicial y se retira (se desactiva) tras el contacto: cuenta como energía del sistema que sale. Al integrar la PR 7 hay que descontar del balance la energía del ariete (la inicial y la que se lleva al retirarse) o excluir sus cuerpos de `mj_energy*`; esta PR no lo hace porque aún no existen. Sus contactos no cuentan como del actor salvo contra el propio actor (`ArtifactProbe` solo mira contactos en que interviene un cuerpo de `ACTOR_BODIES`).
