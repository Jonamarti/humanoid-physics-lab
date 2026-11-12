# Cuerpo de Tori

Código: `src/effectors/toriBody.ts`. Pruebas: `tests/unit/toriBody.test.ts`. Escenario de ejemplo:
`scenarios/uke_tense_75_175_tori_body.json` (en la galería de la web, `?rec=uke_tense_75_175_tori_body`).

Tori es opcional. Un escenario puede llevar:

- **Nada de Tori**: sobre Uke actúan fuerzas directas (`prescribed_force`: impactos y fuerzas constantes) y obstáculos. Ejemplo:
  `scenarios/uke_tense_75_175_side_impact_post.json`.
- **Tori sin cuerpo** (`tori` sin `body`): solo el presupuesto, como hasta ahora.
- **Tori con cuerpo** (`tori.body`): se ve ejecutando una secuencia de posturas y su cuerpo hace de obstáculo. Con
  `tori.body.enabled: false` se simula sin él sin borrar las posturas, para comparar.

## Qué es

Catorce segmentos sueltos, cada uno un cuerpo libre con una cápsula (la cabeza, una esfera): pelvis, tronco, hombros, cabeza,
brazos, antebrazos, muslos, piernas y pies. No hay articulaciones entre ellos. Se mantienen unidos porque los objetivos de todos
salen del mismo esqueleto.

Cada segmento sigue su objetivo con un muelle y un amortiguador (40 rad/s, amortiguación crítica) más su propio peso, y la orden
se recorta a un tope de fuerza horizontal, uno vertical y uno de par. Es el mismo criterio que el obstáculo de
[LIMITED_OBSTACLE.md](LIMITED_OBSTACLE.md): si Uke empuja más que el tope, el segmento cede, y el informe dice cuánto.

| Segmento | Masa (fracción de Tori) | Tope horizontal | Tope vertical | Choca con Uke |
|---|---|---|---|---|
| pelvis | 0,14 | 0,8 pesos | 2,0 pesos | sí |
| tronco | 0,30 | 0,8 | 2,0 | sí |
| hombros | 0,08 | 0,6 | 1,5 | sí |
| cabeza | 0,07 | 0,2 | 0,3 | sí |
| brazo, antebrazo (cada uno) | 0,03 y 0,025 | 0,3 | 0,4 | no |
| muslo (cada uno) | 0,10 | 0,6 | 1,5 | sí |
| pierna (cada una) | 0,05 | 0,5 | 1,5 | sí |
| pie (cada uno) | 0,015 | 0,5 | 2,0 | sí |

Las fracciones se normalizan para que sumen la masa de Tori. El tope de par es el horizontal por 0,25 m. Todos son puntos de
partida sin calibrar.

Los brazos no chocan con nada: lo que une a Tori con Uke son los agarres.

## Agarres

Cada mano puede agarrar un sitio de Uke (`grips.left`, `grips.right`): la manga, la solapa, una muñeca... El agarre es una
cuerda con muelle (2500 N/m, 200 N·s/m, 2 cm de holgura) entre el punto donde la postura pone la mano y el sitio agarrado:

- Solo tira, nunca empuja, y se recorta a `max_force_n` (500 N por defecto).
- El tirón sale del movimiento de Tori. No hay curva de fuerzas: si Tori gira y se lleva las manos, Uke va detrás.
- En t = 0 la mano está en el punto agarrado, sin fuerza. De ahí va a la trayectoria de las posturas, que alcanza en la
  segunda postura.
- Entre dos posturas la mano va en línea recta por el mundo.
- La muñeca se dibuja en el punto agarrado mientras el brazo llega; si no llega, el brazo queda estirado hacia él.
- Con `release_s` la mano suelta en ese instante.

En la grabación cada agarre es un efector más (`tori_left_hand`, `tori_right_hand`): se ve su flecha en el visor, sale en el
informe con su máximo y su tope, y cuenta en el presupuesto de Tori como mano.

## Qué no es

- No tiene equilibrio propio ni apoya el peso en los pies: cada segmento se sostiene con su orden. Si Tori podría hacer esas
  fuerzas con sus pies lo sigue juzgando el presupuesto (`summary.tori_budget`), que ahora suma también el contacto de todo el
  cuerpo sobre Uke.
- Los topes son por segmento, no del cuerpo entero: la suma de lo que empujan tronco, pelvis y hombros puede superar lo que
  una persona haría. El presupuesto lo detecta por el rozamiento y la carga de los pies, no lo impide.
- Un empujón a un segmento no se transmite a los demás.
- La reacción de los agarres no actúa sobre los brazos de Tori: solo se suma en el presupuesto.
- Los dedos no existen: un agarre no se rompe por fuerza, solo se suelta a la hora que se le diga.
- Con Tori activo, todos los efectores cuentan como manos suyas en el presupuesto, también una fuerza directa añadida a mano.

## Formato (`tori.body`)

| Campo | Unidad | Notas |
|---|---|---|
| `height_m` | m | Obligatorio, de 1,40 a 2,20. La masa es `tori.mass_kg` (de 40 a 200 kg con cuerpo). |
| `enabled` | | Opcional, por defecto `true`. |
| `friction` | | Opcional, por defecto 0,8. Rozamiento de su tela contra Uke. |
| `grips` | | Opcional: `{ "left": { "site", "max_force_n"?, "release_s"? }, "right": { ... } }`. `site` es un sitio del humanoide. |
| `keys[]` | | Obligatorio, al menos una postura, con `t_s` creciente. |

Cada postura de `keys[]`, en coordenadas del MUNDO (Uke empieza en el origen mirando a +X; +Y es su izquierda):

| Campo | Unidad | Notas |
|---|---|---|
| `t_s` | s | Instante de la postura. |
| `pelvis_m` | m | Punto medio entre las caderas. De pie, a 0,91 m × altura / 1,76. |
| `yaw_deg` | ° | Hacia dónde mira la pelvis: 0 = +X, 90 = +Y. No se reduce: de 180 a 360 es media vuelta a su izquierda. |
| `lean_deg` | ° | Opcional. Tronco hacia delante: 0 vertical, 90 horizontal. |
| `side_lean_deg` | ° | Opcional. Tronco hacia su izquierda. |
| `twist_deg` | ° | Opcional. Giro de los hombros respecto a la pelvis. |
| `left_foot_m`, `right_foot_m` | m | Apoyo de cada pie, bajo el tobillo; z es la altura de la planta. |
| `left_hand_m`, `right_hand_m` | m | Opcional. Dónde pone cada mano, en el marco del pecho: `[delante, a su izquierda, hacia la cabeza]` desde el punto medio entre los hombros. Con el tronco horizontal, «delante» apunta al suelo. |

Entre posturas se interpola con mínimo tirón; antes de la primera y después de la última, Tori se queda en ellas. Las rodillas y
los codos salen por cinemática inversa: la rodilla se dobla hacia donde mira la pelvis; el codo, hacia abajo y hacia fuera.
Durante el asentamiento Tori se mantiene en la postura de t = 0.

## En la grabación

Los cuerpos `tori_<segmento>` van al final de `model.bodies`, detrás de los obstáculos, y sus articulaciones libres al final
de `qpos`; la reproducción los mueve sin cambios. `peaks.tori_body` trae el contacto total máximo sobre Uke, su impulso y, por
segmento, contacto máximo, tiempo en contacto, cuánto cedió y tiempo recortado por un tope. El informe del editor lo resume en
la línea «Cuerpo de Tori».

El trabajo de las órdenes de los segmentos entra en el balance de energía del detector de artefactos.

## El escenario de ejemplo

`uke_tense_75_175_tori_body`: Tori de 75 kg y 1,75 m frente a un Uke igual, en tensión. No hay curvas de fuerza. La mano
izquierda de Tori agarra la manga derecha de Uke (sitio `right_sleeve_elbow`) y la derecha la solapa (`sternum`), con 300 N de
tope cada una; la solapa se suelta a 1,05 s. La secuencia es la de `defaultToriKeys`: de frente, paso de entrada, media vuelta
a su izquierda, cadera baja de espaldas a Uke con el tronco a 40°, y piernas estiradas con el tronco a 100° mientras la mano de
la manga baja hacia su rodilla izquierda.

Medido con el motor de esta versión:

- Las dos manos empiezan sin fuerza y llegan a su tope de 300 N.
- Uke avanza centrado y queda cargado sobre la espalda de Tori: el tronco lo sostiene 0,59 s (2,8 kN de pico); cargan
  también los hombros, la cabeza y la pelvis de Tori.
- Uke gira 155°.

Lo que no funciona:

- **La proyección no está resuelta.** Uke queda colgado sobre la espalda y se desliza de cabeza: la cabeza toca el suelo a
  1,70 s antes que el resto, con un pico de 12 kN (el informe del editor, que muestrea a 50 Hz, enseña 2,8 kN). En la referencia, Uke da la vuelta completa y cae de espaldas.
- **Tori no es viable** en la entrada: durante 0,26 s necesita el 160 % del rozamiento de sus pies.
- Con 400 N por mano Uke sale antes de cargar del todo y cae de pecho (116° de giro).
- Tarda 1,50 s del primer tirón al suelo; la referencia, 1,45 s. La coincidencia no dice mucho mientras la caída sea otra.

Antes de los agarres se probó el mismo cuerpo con dos tirones de curva fija (los de los escenarios anteriores): con ellos Uke
rodaba 164° sobre la espalda y caía sobre la pelvis sin tocar con la cabeza, y Tori era viable. Se descartó porque las manos no
unían a los dos: la fuerza seguía su curva aunque Tori estuviera en otro sitio.

## Tempos de referencia

Medidos sobre una demostración a velocidad real grabada de perfil a 25 fps (56 fotogramas, un fotograma cada 0,04 s). No se
publica el vídeo. Tiempos desde que empieza el desequilibrio:

| Fase | Empieza | Dura |
|---|---|---|
| Desequilibrio y paso de entrada | 0,00 s | 0,55 s |
| Giro hasta dar la espalda | 0,55 s | 0,25 s |
| Carga, con las rodillas flexionadas | 0,80 s | 0,20 s |
| Proyección: piernas estiradas y tronco inclinado | 1,00 s | 0,35 s |
| Uke en el suelo | 1,45 s | |

Son de un solo clip y de una sola pareja; el error de lectura es de un fotograma (0,04 s) por fase. Las grabaciones largas de
la misma fuente mezclan tomas a velocidad real y a cámara lenta desde varios ángulos; solo se han medido tiempos en la toma de
perfil a velocidad real.

En esa toma el apoyo de Uke es la parte alta de la espalda y el hombro de Tori, no la cadera: Tori queda con las rodillas a
unos 90° y la cadera por delante y por debajo de la de Uke. En la simulación el tronco también es el segmento que más carga.
