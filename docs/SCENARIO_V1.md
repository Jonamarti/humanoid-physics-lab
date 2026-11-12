# Escenario `hpl.scenario.v1`

Referencia del formato de escenario (issue #74, PR 2). Tipos: `src/scenario/schema.ts`.
Validador: `src/scenario/validate.ts` (`validateScenario(json, { fileName? })`).

Convenciones: unidades SI (m, kg, s, N, grados donde se indica); ejes X adelante, Y izquierda, Z arriba;
claves en inglés y `snake_case`. **Se rechaza cualquier campo desconocido** y cualquier número no finito.
El validador se detiene en el primer fallo y lanza `ScenarioValidationError` con la ruta del campo y el motivo,
por ejemplo `effectors[1].force_n[0]: debe ser un número finito, no NaN`.

## Ejemplo completo

```json
{
  "schema": "hpl.scenario.v1",
  "id": "cylinder_obstacle_low",
  "duration_s": 3.0,
  "record_hz": 125,
  "ground": { "friction": 0.6 },
  "actors": [
    { "id": "uke", "kind": "rigid_cylinder", "mass_kg": 75, "height_m": 1.75, "radius_m": 0.18 }
  ],
  "props": [
    { "id": "obstacle", "kind": "fixed_box", "size_m": [0.06, 0.4, 0.05], "pos_m": [0.26, 0, 0.05] }
  ],
  "effectors": [
    { "id": "pull", "kind": "prescribed_force", "actor": "uke",
      "body": "cylinder", "local_point_m": [0, 0, 0.525],
      "frame": "world", "force_n": [600, 0, 180],
      "start_s": 0.3, "duration_s": 0.15, "waveform": "half_sine" }
  ],
  "expect": { "outcome": "fall", "direction_deg": 0, "tolerance_deg": 30 }
}
```

## Campos

### Raíz

| Campo | Tipo | Unidad | Regla |
|---|---|---|---|
| `schema` | cadena | | Exactamente `hpl.scenario.v1`. Se comprueba antes que el resto. |
| `id` | cadena | | Solo `[a-z0-9_]`, no vacía. Si se pasa `fileName` al validador, igual al nombre del fichero sin `.json`. |
| `duration_s` | número | s | > 0 y <= 30. |
| `record_hz` | número | Hz | Entre 30 y 500 y divisor exacto de la frecuencia física (500 Hz): 50, 100, 125, 250, 500, etc. |
| `ground` | objeto | | Obligatorio. |
| `actors` | lista | | Al menos un actor. |
| `props` | lista | | Puede estar vacía. |
| `effectors` | lista | | Puede estar vacía. |
| `obstacles` | lista | | Opcional. Obstáculos con fuerza limitada (ver abajo). Si falta, el escenario validado no lleva la clave y su hash no cambia. |
| `expect` | objeto | | Opcional. |

Los `id` de actores, props y efectores siguen la misma regla `[a-z0-9_]` y no pueden repetirse dentro de su lista.

### `ground`

| Campo | Unidad | Regla |
|---|---|---|
| `friction` | adimensional | >= 0. Rozamiento deslizante. |

### `actors[]` (`kind: "rigid_cylinder"`)

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | Ver arriba. |
| `kind` | | `rigid_cylinder` (único tipo en esta fase). |
| `mass_kg` | kg | > 0. |
| `height_m` | m | > 0. |
| `radius_m` | m | > 0. |

Cuerpos referenciables por los efectores: `rigid_cylinder` tiene `cylinder`.

### `actors[]` (`kind: "humanoid"`)

Uke humanoide paramétrico ([UKE_MODEL.md](UKE_MODEL.md), [UKE_CONTROL.md](UKE_CONTROL.md)). Cuerpo principal (el que se mide): `pelvis`.
Cuerpos referenciables por los efectores: `pelvis`, `abdomen`, `chest`, `head`, `left/right_upperarm`, `left/right_forearm`,
`left/right_thigh`, `left/right_shin`, `left/right_foot`.

| Campo | Unidad | Regla |
|---|---|---|
| `id`, `kind` | | `kind` es `humanoid`. |
| `mass_kg` | kg | Obligatorio, de 50 a 150. |
| `height_m` | m | Obligatorio, de 1,50 a 2,05. |
| `strength` | | Opcional, de 0,5 a 1,5 (por defecto 1). Escala el par máximo. |
| `build` | | Opcional, de 0 a 1: fracción del exceso de masa (IMC > 27) que va al tronco (por defecto 0,6). |
| `segment_lengths_m` | m | Opcional. Claves `thigh`, `shin`, `foot`, `upper_arm`, `forearm`, `trunk`, `neck`; cada una entre 0,5 y 2 veces el valor de v2. |
| `attitude` | | Opcional (por defecto `tense`): `tense` o `relaxed` (sin objetivos, solo amortiguación pasiva; ver [UKE_CONTROL.md](UKE_CONTROL.md)). |
| `stance` | | Opcional (por defecto `neutral`): `neutral`, `natural_grip`, `defensive`, `staggered_left`, `staggered_right`. |

Con un humanoide, el asentamiento de 0,5 s ya lleva el controlador activo y el veredicto es el del humanoide: `fall`, `stepped`, `standing` o `unresolved`
(definiciones en [RECORDING_V1.md](RECORDING_V1.md#veredicto-y-métricas-de-equilibrio-del-humanoide)). `summary.uke` lleva medidas adicionales.
El campo `fall` del ejemplo de la issue #75 todavía no existe (es de otra PR) y se rechaza como campo desconocido.

### `props[]` (`kind: "fixed_box"`)

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | Ver arriba. |
| `kind` | | `fixed_box` (único tipo en esta fase). |
| `size_m` | m | Tres aristas `[x, y, z]`, todas > 0. |
| `pos_m` | m | Centro `[x, y, z]` en el mundo. |

### `effectors[]` (`kind: "prescribed_force"`)

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | Ver arriba. |
| `kind` | | `prescribed_force` (único tipo en esta fase). |
| `actor` | | `id` de un actor existente. |
| `body` | | Cuerpo existente de ese actor. Obligatorio salvo que se indique `site`. |
| `site` | | Alternativa a `body` (solo humanoide): sitio con nombre (`sternum`, `nape`, `belt_front`, `head_front`, etc.; lista en `HUMANOID_SITE_BODY`, `src/scenario/schema.ts`). Excluye `body`; el validador rellena `body` con el cuerpo del sitio. |
| `local_point_m` | m | Con `body`: punto de aplicación `[x, y, z]` en el sistema local del cuerpo (obligatorio). Con `site`: desplazamiento opcional (por defecto 0) respecto al sitio, en los ejes del cuerpo. |
| `frame` | | `world`, `actor_initial` (ejes del actor en t=0) o `actor_current`. |
| `force_n` | N | Fuerza `[x, y, z]` expresada en `frame`. Valor máximo de la forma de onda. |
| `start_s` | s | >= 0, desde el comienzo del escenario (tras el asentamiento). |
| `duration_s` | s | > 0 y <= 30. |
| `waveform` | | `half_sine` o `constant`. |

### `effectors[]` (`kind: "hand"`)

Mano virtual de Tori ([VIRTUAL_HAND.md](VIRTUAL_HAND.md)): un punto objetivo unido al agarre por muelle y amortiguador, con
tope de fuerza, de potencia y de alcance. Tori no tiene cuerpo: `shoulder_m`, los `offset_m` y `axis` se dan en el marco
`actor_initial` (origen y ejes del cuerpo principal del actor en t = 0, tras el asentamiento; +X delante de Uke, +Z arriba).

| Campo | Unidad | Regla |
|---|---|---|
| `id`, `actor`, `body`, `site`, `local_point_m` | | Como en `prescribed_force`: punto del actor que se agarra. |
| `link` | | `rope` (solo tracción), `clamp` (tracción y empuje) o `push` (solo empuje). |
| `shoulder_m` | m | Hombro de Tori: centro de la esfera de alcance. |
| `close_s` | s | >= 0. Instante en que la mano se cierra; el agarre en ese instante es el origen de los desplazamientos. |
| `release_s` | s | Opcional, >= `close_s`. Suelta programada. |
| `phases[]` | | `start_s`, `duration_s` (> 0) y `offset_m`: desplazamiento ACUMULADO del objetivo respecto al origen, alcanzado con mínimo tirón al final del tramo. Sin solapes y no antes de `close_s`. Puede estar vacía. |
| `axis` | | Opcional, solo `push`: dirección del empuje. |
| `k_n_per_m`, `c_ns_per_m`, `slack_m`, `max_force_n`, `max_power_w`, `reach_m`, `slip_after_s` | | Opcionales; por defecto 3000 N/m, 200 N·s/m, 0,03 m (`rope`), 500 N, 600 W, 0,75 m y 0,4 s. Reglas de `resolveHandParams`. |

La mano resbala (suceso `slip`) si su demanda pasa `slip_after_s` seguidos en el tope de fuerza. El validador no rellena
los valores por defecto: el hash del escenario solo cambia con lo que el autor escribe.

### `effectors[]` (`kind: "pull"`)

Tirón guiado (`src/effectors/pull.ts`): una mano cuya **fuerza** sigue una curva en el tiempo. La mano `hand` guía la
posición de un objetivo y la dirección de la fuerza depende de dónde esté el agarre (en `hip_drive` llega a cambiar de
sentido); `pull` fija la dirección, que es lo que hace falta para un tirón en arco.

| Campo | Unidad | Regla |
|---|---|---|
| `id`, `actor`, `body`, `site`, `local_point_m`, `frame` | | Como en `prescribed_force`. |
| `keys[]` | | Al menos dos puntos `{t_s, force_n}` con `t_s` estrictamente creciente. Entre puntos se interpola linealmente el vector; antes del primero y desde el último la fuerza es nula. |
| `max_force_n` | N | Opcional, por defecto 500. Ningún punto puede superarlo. |
| `max_power_w` | W | Opcional, por defecto 600. Si `F · v` del agarre lo supera, la fuerza se reduce. |

No tiene alcance ni resbalón: no hay hombro ni muelle. La viabilidad del conjunto la juzga `tori`.

### `tori`

Opcional. `{ "mass_kg": 75 }` más `friction` (por defecto `ground.friction`), `total_pull_max_n` (1100) y
`total_power_max_w` (1500). Con él la grabación lleva `summary.tori_budget` (ver [RECORDING_V1.md](RECORDING_V1.md)).

### `obstacles[]`

Obstáculo con masa y fuerza limitada ([LIMITED_OBSTACLE.md](LIMITED_OBSTACLE.md)): la cadera, la pierna o el pie de Tori en
abstracto. Es un cuerpo del modelo, llamado como su `id`, montado sobre tres deslizaderas y gobernado por un seguimiento con
topes: si Uke carga más que el tope, cede. **Sus coordenadas están en el mundo**, no en `actor_initial`: el cuerpo se crea
antes del asentamiento. El actor nace en el origen mirando a +X, así que para un actor sin girar ambos marcos solo difieren
en la altura de la pelvis. Durante el asentamiento el obstáculo se sostiene en `start_m`.

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | `[a-z][a-z0-9_]*`, único entre obstáculos y distinto de los `id` de `props` y de los cuerpos del actor. |
| `shape` | | `bar` (cápsula transversal, eje Y), `post` (cápsula vertical), `pad` o `foot` (cajas). |
| `tori_mass_kg` | kg | > 0. Fija los valores por defecto: masa del obstáculo (la mitad), tope horizontal (0,8 pesos) y vertical (2,0 pesos). |
| `start_m` | m | Centro de la geometría en el mundo. |
| `phases[]` | | Como en la mano, con `offset_m` en el mundo y respecto a `start_m`. Puede estar vacía (obstáculo quieto). |
| `mass_kg`, `length_m`, `radius_m`, `size_m`, `surface` (`cloth` o `skin`), `friction`, `kp_n_per_m`, `damping_ratio`, `max_force_n`, `max_vertical_force_n`, `max_speed_mps` | | Opcionales; reglas de `resolveObstacleParams`. Se rechaza una combinación de `kp_n_per_m` y masa inestable a 500 Hz. |

No se admite todavía la bisagra (`rotation`) del módulo del obstáculo.

### `expect`

| Campo | Unidad | Regla |
|---|---|---|
| `outcome` | | `standing`, `tilted`, `fall`, `stepped` o `unresolved` (el cilindro solo da `standing`, `tilted` o `fall`; el humanoide, `standing`, `stepped`, `fall` o `unresolved`). |
| `direction_deg` | grados | Opcional. Solo con `outcome: "fall"`. 0 = +X, 90 = +Y. Exige `tolerance_deg`. |
| `tolerance_deg` | grados | Opcional, solo junto a `direction_deg`. > 0 y <= 180. |

## Escenarios incluidos

`scenarios/*.json`: los escenarios del cilindro (issue #74, PR 4) y `uke_stand_<talla>_<postura>.json` (humanoide de pie 10 s, `expect: standing`); `tests/unit/scenarios.test.ts` valida y ejecuta cada uno y exige
que cumpla su `expect`. Todo escenario de esa carpeta debe declarar `expect`. Resultados y umbrales medidos:
[CYLINDER_BENCHMARK.md](CYLINDER_BENCHMARK.md). Salida de la ejecución: [RECORDING_V1.md](RECORDING_V1.md).
Las tres fases de un derribo se escriben como tres efectores `prescribed_force` consecutivos (`phase_a`, `phase_b`, `phase_c`).

`uke_tense_75_175_hip_tip.json` es el primer escenario con manos y obstáculo: dos manos `rope` (manga derecha y esternón)
tiran delante y arriba 0,4 s y después delante y abajo, con una barra a la altura de la cadera delante de Uke (centro a
0,90 m; la pelvis está a 0,98 m). Medido con Uke `tense` de 75 kg y 1,75 m:

| Tope por mano | Resultado | Inclinación máxima del tronco | Carga máxima sobre la barra | La barra cede |
|---|---|---|---|---|
| 100 N | sigue de pie, sin resolver (las manos resbalan) | 16° | 546 N | 0,7 cm |
| 200 N | sigue de pie, sin resolver (las manos resbalan) | 31° | 1013 N | 2,0 cm |
| 250 N | cae, sin pasar por encima de la barra | 84° | 1200 N | 3,6 cm |
| 385 N (500/1,3, margen de la prueba) | cae | | | |
| 500 N (escenario) | gira por encima de la barra y cae de espaldas | 162° | 2192 N | 7,6 cm |
| 500 N, sin barra | cae de frente, arrastrado | 87° | | |

Con 500 N la mano de la manga llega a 398 N y la del esternón está 0,14 s en el tope; ninguna resbala. La barra pasa
0,56 s saturada. Al aterrizar el detector marca 11,1 kN en la pelvis y 4,4 cm de penetración de un pie: es el impacto
sin reflejos de caída descrito en [ESTADO_DERRIBOS_20261112.md](ESTADO_DERRIBOS_20261112.md). Uke gira rígido como una
tabla porque `tense` no cede en la cintura. Barrido hecho a mano con `npm run sim`; solo las filas de 385 N y 500 N con
barra están cubiertas por pruebas.

`uke_tense_75_175_hip_drive.json` es la variante en la que la cadera empuja en vez de ceder. Mismas manos y misma barra, con
tres cambios: la barra tiene topes más altos (1100 N horizontal y 1840 N vertical, 1,5 y 2,5 pesos de Tori) y entre 0,6 y
1,0 s se desplaza 25 cm hacia Uke y 20 cm hacia arriba, cuando Uke ya está cargado sobre ella; y las manos añaden un tercer
tramo (1,1 a 1,5 s) que lleva el agarre abajo y hacia Tori, y sueltan a 1,6 s. Medido:

| | Barra que cede (`hip_tip`) | Barra que empuja (`hip_drive`) |
|---|---|---|
| Inclinación máxima del tronco | 162° | 168° |
| Instante de la caída | 2,13 s | 1,68 s |
| Primera parte en el suelo | cabeza | brazo izquierdo |
| Fuerza de contacto por encima de 10 kN | sí (11,1 kN, pelvis) | no |
| Carga máxima sobre la barra | 2192 N | 2193 N |
| La barra se separa de su objetivo | 7,6 cm | 10,6 cm |
| Tiempo con la barra saturada | 0,56 s | 0,24 s |
| Trabajo de la barra | −85 J (absorbe) | +313 J (aporta) |

Sigue marcando penetración al aterrizar (5,4 cm de un pie, 2,9 cm entre antebrazo y muslo). Con el mismo empuje de la barra
y las manos soltando a 1,3 s sin el tercer tramo, Uke no completa el giro y cae de cabeza (24 kN): el acompañamiento de las
manos es lo que termina la rotación.

`uke_tense_75_175_hip_roll.json` conserva la barra de `hip_drive` y sustituye las manos por dos tirones guiados iguales
(manga derecha y esternón), en el marco `actor_initial`: 400 N a 45° sobre la horizontal (0,3 a 0,6 s), 300 N horizontales
(0,8 s), 300 N a 45° bajo la horizontal (1,0 s) y 200 N a 110° bajo la horizontal, es decir, abajo y hacia Tori (1,2 s);
cero a 1,4 s. Declara `tori` de 75 kg. Medido:

- Uke gira 170°, cae a 1,42 s sobre el brazo y la parte alta de la espalda (7,8 kN) y la cabeza no golpea. Sin artefacto
  de fuerza de contacto.
- La barra toca la pelvis de 0,65 a 1,15 s (pico 1403 N); no toca los muslos.
- Tori viable: usa el 85 % de su rozamiento (a 1,02 s), la carga de sus pies va de 373 a 1757 N, tiro total 800 N.
- El tope de potencia (600 W) recorta la mano de la manga durante 0,27 s.

Lo que rompe el volteo, medido con el mismo escenario:

| Cambio | Resultado |
|---|---|
| Módulo constante de 400 N en todo el arco | Cae de cabeza (19 kN) y Tori inviable: al tirar hacia abajo pierde rozamiento |
| Arco 0,15 s más lento | Cae de cabeza (21 kN): Uke ya ha pasado cuando llega el tirón hacia abajo |
| Rozamiento de la barra 0,5 en vez de 0,8 | El contacto pasa de la pelvis al muslo y se pierde; cae de cabeza (21 kN) |
| Rozamiento de la barra 0,2 | La barra solo toca el muslo 0,08 s; Uke se escurre y cae de cabeza (21 kN). Cubierto por una prueba |
| Barra 18 cm más baja (a la altura del muslo), rozamiento 0,8 | Contactos de 0,03 s en los muslos; no llega a cargarlo y cae de cabeza (8 kN) |

Es decir: el volteo depende de que la cadera agarre la pelvis de Uke. Sobre los muslos, o con poco rozamiento, la
barra resbala al subir en vez de empujar. Salvo la fila indicada, el barrido está hecho a mano con `npm run sim`.

## Notas y límites

- **`record_hz` y el ejemplo de la issue #74.** El ejemplo de la issue usa `record_hz: 120`, pero su propia regla
  exige que `record_hz` sea divisor exacto de 500 Hz, y 500/120 no es entero. El validador aplica la regla
  (rechaza 120) y el ejemplo de este documento usa 125. Si se prefiere admitir 120 Hz habrá que cambiar la regla
  (por ejemplo, muestrear al primer paso físico posterior a cada instante de grabación) y decidirlo aparte.
- Reglas añadidas por esta PR más allá de la lista de la issue, por coherencia: unicidad de `id` dentro de cada
  lista, al menos un actor, `ground.friction` >= 0, `start_s` >= 0, aristas > 0 y las reglas de `expect` de
  arriba. Los valores admitidos de `waveform` distintos de `half_sine` (`constant`) también son una elección de
  esta PR.
- No se comprueba que el efector termine antes del fin del escenario ni que los props no se solapen con los actores.
- Las fases siguientes añadirán tipos de actor, efector y prop; entonces se ampliarán `schema.ts`, `ACTOR_BODIES`
  y este documento.
