# Obstáculo con masa y fuerza limitada

Issue #76, PR 3. Código: `src/effectors/obstacle.ts`. Pruebas: `tests/unit/obstacleEffector.test.ts` (MuJoCo WASM real).
Plan: [PLAN_DERRIBOS.md](PLAN_DERRIBOS.md) (secciones 3, 9 y 12.1, fallos 2, 5 y 6).

Esta PR entrega el módulo **autónomo**: no está conectado todavía al formato de escenario ni al ejecutor (ver «Qué queda»). Solo importa
`minimumJerk` de `hand.ts` y la decodificación de contactos de `src/physics/contactForces.ts`; no toca `src/scenario/`, `src/uke/`,
`src/metrics/` ni `src/recording/`.

## Qué es

Un obstáculo es un cuerpo con masa (cadera, hombro, pierna o pie de Tori, en abstracto) montado sobre tres deslizaderas (X, Y, Z) y, si gira,
una bisagra. Sigue una trayectoria objetivo con fuerza **limitada**: nunca es un cuerpo fijo ni `mocap`. Si Uke empuja más que el tope, el
obstáculo cede y se mide cuánto. No simula el cuerpo de Tori: es un indicador de qué hace Tori a Uke, como la mano virtual.

## API

Funciones puras y deterministas; el estado es inmutable. Todos los vectores están en el mundo (+X delante, +Y izquierda, +Z arriba).

```ts
// 1. Crear el fragmento MJCF de una barra (o post, pad, foot)
const params = resolveObstacleParams({
  name: "hipbar", shape: "bar", tori_mass_kg: 75, start_m: [x, y, z],        // centro de la barra, en el mundo
  phases: [{ start_s: 0.2, duration_s: 0.3, offset_m: [0.05, 0, 0] }],        // opcional; desplazamientos ACUMULADOS desde start_m
});                                                                            // lanza RangeError con el campo y el motivo
const xml = buildObstacleMjcf(params);          // un <body> con joints hipbar_x/_y/_z, geom hipbar_geom
const mjcf = appendToWorldbody(ukeMjcf, xml);   // inserta antes del último </worldbody>: DESPUÉS de Uke (ver «Orden en el MJCF»)

// 2. Resolver direcciones una vez, con el modelo compilado
const handles = resolveObstacleHandles(model, params);
let state = createObstacleState();

// 3. Avanzar su control, UNA vez por paso físico, antes de mj_step y DESPUÉS del controlador de Uke
const k = readObstacleKinematics(params, handles, data);                       // { position_m, velocity_mps, (angle_rad, angular_velocity_rad_s) }
const r = stepObstacle(params, state, { t, dt, ...k, contact_force_on_uke_n }); // t = instante del comienzo del paso
state = r.state;
applyObstacleCommand(handles, data, r);          // escribe qfrc_applied SOLO en los grados de libertad del obstáculo
mj_step(model, data);

// 4. Leer la fuerza de contacto sobre Uke (tras mj_step; es la del paso que acaba de ejecutarse) y pasarla al paso siguiente
const c = readObstacleContactForce(model, handles, data, { pyramidal, isOtherBody });   // { force_on_other_n, count }
const contact_force_on_uke_n = c.force_on_other_n;

// 5. Muestra, eventos y totales
r.sample   // ObstacleSample (abajo)
r.events   // [{ t, kind: "saturated" | "recovered" }]
r.reaction_n          // = −F, reacción de Tori
state.totals          // trabajo, impulsos, picos, tiempo saturado
state = settleObstacleWork(state, velocity)   // al terminar: cierra el trabajo del último paso
```

`ObstacleSample` (lo que hay que grabar por muestra):

| Campo | Significado |
|---|---|
| `force_n` | orden aplicada al obstáculo (con la compensación de su peso, tras limitar) |
| `demand_force_n` | orden antes de limitar |
| `contact_force_on_uke_n` | fuerza de contacto del obstáculo sobre Uke, tal como la pasó el llamante |
| `target_m`, `position_m` | objetivo y posición real |
| `deviation_m`, `deviation_magnitude_m` | `posición − objetivo` |
| `saturated` | algún tope (horizontal, vertical o de par) recortó la orden: el seguimiento está saturado |
| `horizontal_saturated`, `vertical_saturated`, `torque_saturated`, `speed_limited` | qué tope fue |
| `reaction_n` | `−force_n` |
| `torque_nm`, `angle_deviation_rad` | solo con `rotation` |

Otras funciones: `obstacleTarget(params, t)` (posición, velocidad y aceleración objetivo, sin estado), `plannedPeakSpeedMps(phases)`,
`obstacleStability(params, dt)`, `obstacleInertia(geometría, masa)`. Constantes: `OBSTACLE_DEFAULTS`, `OBSTACLE_FRICTION`,
`OBSTACLE_SHAPE_DEFAULTS`, `GRAVITY_MPS2`.

La fuerza de contacto sobre Uke no la conoce el módulo puro: la mide `readObstacleContactForce` (suma las fuerzas resueltas, `efc_force`, de los
contactos entre la geometría del obstáculo y los cuerpos que acepte `isOtherBody`; por defecto, cualquiera que no sea el suelo ni el obstáculo) y el
llamante la pasa en `contact_force_on_uke_n` del paso siguiente (se registra tal cual). Para el ejecutor basta con `mj_forward` en el instante de
muestreo, como hace con los demás contactos.

## Orden en el MJCF y en el paso (IMPORTANTE para integrarlo)

1. **El obstáculo debe ir después del cuerpo de Uke.** `generateHumanoidMJCF(..., { extraWorldXml })` inserta ANTES de la pelvis: el obstáculo
   pasaría a ser el cuerpo 1 y sus deslizaderas los grados de libertad 0 a 2. El controlador `tense` (`standingBaseline.ts`) lee `subtree_com[3]`,
   `subtree_linvel[3]` (supone que el cuerpo 1 es la pelvis) y comprueba `qfrc_applied[0..5]` (supone que son la raíz libre); el ejecutor hace
   lo mismo con `rootForceMax`. Por eso existe `appendToWorldbody`. Comprobado en la prueba «insertarlo tras Uke…» (con `extraWorldXml` el cuerpo 1
   es el obstáculo). *Sin comprobar:* los props de `buildModel.ts` (`extraWorldXml`) son cuerpos estáticos que también quedan antes de la pelvis y,
   por la misma lectura del código, desplazan el cuerpo 1; no lo he ejecutado, es un posible fallo existente que habrá que mirar.
2. **La orden se escribe en `qfrc_applied` después del controlador de Uke**, que pone todo `qfrc_applied` a cero (`attitudes.ts`). El ejecutor
   actual llama a los efectores antes que a `runUke`: hay que añadir una llamada tras `runUke` (y antes de `mj_step`). `applyObstacleCommand` asigna
   (no suma) y solo toca los grados de libertad del obstáculo.
3. Ninguna fuerza va a la raíz de Uke: el obstáculo actúa sobre Uke solo por contacto.

## Modelo y fórmulas

**Seguimiento.** Con `e = p_t − p`, `ė = v_t − v`, masa `m`, `kp` y `c = 2·ζ·sqrt(kp·m)` (ζ = 1: amortiguación crítica), la orden es

```
F = kp·e + c·ė + m·a_t + m·g·ẑ
```

con `a_t` la aceleración del objetivo (alimentación directa, `feedforward`) y `m·g·ẑ` la compensación del peso (`gravity_compensation`). Después:

1. **Limitador de velocidad** (si `max_speed_mps`): si la aceleración que daría `F` (con la gravedad real) lleva `|v|` por encima del tope en el paso
   siguiente, se sustituye `F` por la que deja `|v_siguiente| = max_speed_mps`. Puede frenar (si algo lo empuja por encima del tope), pero no es un
   freno rígido: una fuerza externa mayor que el tope de fuerza sí puede superarlo.
2. **Tope horizontal:** módulo de `(Fx, Fy)` ≤ `max_force_n`.
3. **Tope vertical:** `|Fz|` ≤ `max_vertical_force_n`.

`saturated` es verdadero si 2 o 3 (o el par de la bisagra) recortaron la orden. La trayectoria objetivo (`phases`) es la de `hand.ts`: desplazamientos
acumulados con mínimo tirón `10τ³ − 15τ⁴ + 6τ⁵`, con sus derivadas analíticas para la alimentación directa.

**Bisagra** (opcional, `rotation`): mismo PD en ángulo, `kp_rot = kp·0,15²`, amortiguación crítica con la inercia de la geometría respecto al eje,
tope de par `max_force_n·0,15`. Eje fijo en el mundo (las deslizaderas están antes en la cadena y no giran el cuerpo). Eje por defecto Y.

**Reacción de Tori.** `reaction_n = −F` (incluye la compensación del peso propio, que Tori también sostiene). `state.totals` acumula impulso, reacción,
trabajo (`∫F·v dt`, trapecio), picos, tiempo saturado y tiempo con el limitador de velocidad. No hay presupuesto aquí (PR 4).

### Identidad de la fuerza de contacto

Para el obstáculo, `m·a = F + F_c + m·g` con `F_c` la fuerza de contacto sobre él. Luego sobre el otro cuerpo actúa `−F_c = F − m·(a − g)`, y

```
|F_h| ≤ tope_h + m·|a_h|        |F_v| ≤ tope_v + m·|a_z + g|
```

El «término inercial» es `m·|a − g|` (aceleración respecto a la caída libre), no solo `m·a`: con la compensación del peso dentro de la orden, el peso
propio aparece en el balance vertical. La prueba comprueba el límite en cada paso y, además, la identidad completa (residuo 0,0 N): valida a la vez el
tope y el lector de contactos. **El límite es ajustado, no holgado**: cuando la orden está en el tope, la fuerza de contacto vale exactamente `tope + m·a`
(relación medida 1,000). Un obstáculo pesado que Uke acelera genera fuerzas de contacto grandes aunque su tope sea pequeño (ver la tabla).

## Decisión: `qfrc_applied` o actuadores de posición con `forcerange`

La issue pide «actuadores de posición con `forcerange`». Se ha elegido **escribir fuerzas generalizadas limitadas en `qfrc_applied`** (como el controlador
de Uke), no `<actuator>`, por estas razones:

- **Tope horizontal en módulo.** `forcerange` limita cada actuador por separado: con X e Y independientes el módulo llegaría a `√2` veces el tope en
  diagonal. Aquí el tope horizontal es el módulo de `(Fx, Fy)` y el vertical se limita aparte.
- **Alimentación directa** de velocidad y aceleración del objetivo, que un actuador de posición (`kp·(ctrl − q) − kv·q̇`) no puede dar. Sin ella el PD
  tiene un retardo `c·v/kp`: con la trayectoria de la prueba, 7,0 mm de error frente a 0,34 mm con ella (el criterio es < 5 mm).
- **Limitador de velocidad y banderas** (`saturated`, `speed_limited`) y reacción acumulada se calculan en el mismo sitio; un actuador no informa de
  que `forcerange` ha recortado.
- **Coherencia con el repositorio:** los modelos no tienen `<actuator>` (`nu = 0`) y `ctrl` no se usa; añadir actuadores cambia `nu` y obliga a tocar el
  ejecutor y el registro.
- **Por grado de libertad y no con `xfrc_applied`:** una fuerza cartesiana en el centro de masas se acopla por el jacobiano con la bisagra si su eje no
  pasa por él; `qfrc_applied` por articulación no.

Lo que se pierde: MuJoCo no hace cumplir el tope; si el llamante no invoca `stepObstacle`/`applyObstacleCommand` en un paso, el obstáculo cae por su
peso (comprobado: 1,2 m en 0,5 s). El control es explícito (la orden usa la posición del comienzo del paso): estable con `a = c·dt/m < 2` y
`b = kp·dt²/m < 4 − 2a` (`obstacleStability`); con 37,5 kg y 2 ms, `a = 0,09`; con 1 kg, `a = 0,57`; con 0,1 kg inestable.

## Decisión: gravedad del propio obstáculo

La orden **incluye** `m·g·ẑ` y por tanto **consume tope vertical**: con 75 kg de Tori y 37,5 kg de obstáculo, 368 N de 1471,5 N (25 %). No se usa
`gravcomp` de MuJoCo (sería fuerza gratis fuera del tope). Medido: sin compensación, el obstáculo cuelga `m·g/kp = 1,84 cm` bajo su objetivo; con ella,
la orden vertical en reposo es `m·g` y el error 0. El peso máximo que sostiene de más es `tope_v − m·g` (1103 N con los valores por defecto). Un
`max_vertical_force_n` menor que el peso propio se rechaza al resolver los parámetros.

## Decisión: colisiones

El obstáculo choca con Uke, el suelo y los props, **no con otros obstáculos**, mediante `contype="2" conaffinity="1"` (MuJoCo hace chocar dos geometrías
si `(contype_a & conaffinity_b) | (contype_b & conaffinity_a) ≠ 0`):

- Contra Uke, suelo y props (valores 1/1 por defecto): `1 & 1 = 1` → chocan.
- Entre obstáculos: `2 & 1 = 0` en los dos sentidos → no chocan.
- No se modifica ninguna geometría de Uke ni del suelo: sus contactos no cambian (prueba: mismo asentamiento de Uke, `qpos` igual a 9 decimales y mismo
  número de contactos, con y sin obstáculo).

Se prefirió esto a `<contact><exclude>` porque este exige un par de nombres de cuerpo por cada pareja de obstáculos (crece con n²) y que quien añada
un obstáculo conozca a los anteriores; con los bits basta el fragmento propio. Coste: toda geometría nueva que deba chocar con un obstáculo necesita
`conaffinity` con el bit 1 (es el valor por defecto).

**Rozamiento.** MuJoCo toma por defecto el MAYOR de los dos rozamientos de un contacto; Uke y el suelo tienen 1,0, así que `cloth` (0,8) y `skin` (0,5)
no tendrían efecto. El obstáculo lleva `priority="1"`: manda su rozamiento (y su `solref`/`solimp`, que son los de siempre). Medido con un bloque de
rozamiento 1,0 sobre una almohadilla: con `cloth` agarra a 0,7·N (0,45 cm en 1 s) y desliza a 0,9·N; con `skin` desliza a 0,7·N y agarra a 0,4·N.
Efecto secundario: también manda contra el suelo.

## Formas y valores por defecto

Todos son puntos de partida de la issue **sin calibrar**; las pruebas solo verifican que el módulo respeta sus propios topes con ellos.

| Parámetro | Valor | Origen |
|---|---|---|
| `kp_n_per_m` | 20 000 N/m | Issue #76 |
| `damping_ratio` | 1 (crítica) | Issue #76 |
| `mass_kg` | 0,5 × masa de Tori | Issue #76 |
| `max_vertical_force_n` | 2,0 × peso de Tori | Issue #76 |
| `max_force_n` (horizontal) | 0,8 × peso de Tori | Issue #76 |
| `surface` | `cloth` 0,8; `skin` 0,5 | Issue #76 |
| `bar` | tramo recto de 0,40 m (a lo largo de Y), radio 0,11 m | Issue: «40 cm y 10–12 cm de radio». **Interpretación:** los 40 cm son el tramo recto, sin las tapas semiesféricas (longitud total 0,62 m) |
| `post` | tramo recto 0,5 m (vertical), radio 0,05 m | Mío, sin fuente |
| `pad` | caja 0,10 × 0,40 × 0,40 m (X, Y, Z) | Mío, sin fuente |
| `foot` | caja 0,24 × 0,10 × 0,08 m | Mío, sin fuente |
| bisagra | `kp·0,15²`, tope `max_force_n·0,15` | Mío, sin fuente |

`max_force_n` es el tope **horizontal** (módulo en XY); el vertical es `max_vertical_force_n`. Segado y barrido son un `post` o un `foot` con masa,
`max_force_n`, `max_vertical_force_n` y `max_speed_mps` propios (la masa por defecto, la mitad de la de Tori, es excesiva para una pierna o un pie:
dala explícita).

## Medidas (MuJoCo WASM, Tori de 75 kg, 2 ms)

| Prueba | Resultado |
|---|---|
| Seguimiento sin Uke (avance 15 cm en 0,4 s, subida 5 cm, desvío lateral 10 cm; pico 0,70 m/s) | error máx **0,34 mm** (criterio < 5 mm). Sin alimentación directa: 7,04 mm |
| Bisagra (0,5 rad en 0,4 s y −0,2 rad) | error de ángulo máx 0,0017 rad; par máx 9,6 N·m (tope 88,3) |
| Carga de 3 × tope vertical (4415 N) | se hunde 82 cm en 0,5 s, orden vertical = tope (1471,5 N), saturado |
| Carga de 0,5 × tope vertical (736 N) | cede **36,8 mm** en régimen (= F/kp), 40,7 mm de máximo. **Criterio «< 1 cm» NO cumplido** |
| Misma carga con `kp = 100 000` N/m | 8,2 mm de máximo (dato: hace falta `kp ≥ F/1 cm` = 73 575 N/m en régimen) |
| Choque de un bloque de 100 kg a 2 m/s contra la almohadilla | pico de contacto 7808 N; fuerza / (tope + m·\|a − g\|) máx **1,000**; residuo de Newton 0,0 N |
| Empuje de 883 N (1,5 × tope) sobre un trineo de 100 kg contra la almohadilla | satura a los 0,058 s; cede 50,6 cm en 0,6 s; orden horizontal máx 588,6 N; fuerza en régimen entre el tope y el empuje |
| `post` de 8 kg, trayectoria planificada a 6 m/s | sin límite 6,71 m/s; con `max_speed_mps = 2`: **2,0000 m/s** |
| Simetría en Y (choque oblicuo y su reflejo) | asimetría 4e-17 m (obstáculo), 2e-16 m (bloque) |
| Uke `tense` (75 kg, 1,75 m) contra `bar`, empuje de 80 N en el esternón | desvío lateral del CM y de la pelvis < 0,001 mm; la barra cede 4,7 mm; contacto máx 113 N (0,146 del límite) |
| Mismo montaje con empuje lateral de 20 N | CM 5,8 mm, pelvis 3,2 mm. Con rozamiento 0,05: pelvis 7,6 mm |
| Barra con tope 100 N y empuje de 250 N | satura a los 0,28 s; cede 1,55 m en 1,5 s; contacto 2893 N máx (relación 1,000); orden ≤ 100 N |

### Uke `tense` apoyado contra la barra (montaje exacto)

Uke `tense` (75 kg, 1,75 m, postura `neutral`, sin controlador modificado: `createUkeController` con las ganancias y la amortiguación de `tense`,
como el ejecutor), de cara a +X. La barra (`bar`, tori 75 kg, valores por defecto) va delante de la pelvis, a la altura de su centro, a 1 mm del
contacto. Tras 0,5 s de asentamiento sin empuje, una fuerza horizontal en +X sobre el sitio `sternum` (aplicada con `applyForceAtPoint` al cuerpo `chest`,
nunca a la raíz; efector declarado) sube a 80 N en 0,3 s y se mantiene 2 s. Esa fuerza es lo que empuja a Uke contra la barra: la pelvis se apoya y la
barra cede 4,7 mm. Resultados en la tabla. **Cuidado al leerlo:** con un empuje sagital y un Uke simétrico el desvío lateral es exactamente 0 por
simetría; la prueba no demuestra por sí sola que el contacto no resbale. La que sí lo exige es la del empuje con componente lateral de 20 N (el 25 %,
por debajo de µ = 0,8): 3,2 mm de la pelvis, y un control con µ = 0,05 la desvía 7,6 mm (mismas condiciones). Aun así, esa componente lateral la
absorben también los pies de Uke, no solo el rozamiento de la barra. Y la forma de la barra no queda probada: con una barra de 1 mm de tramo recto
(casi un punto) el resultado de las pruebas de Uke es el mismo (la colisión cápsula–elipsoide de MuJoCo da un solo punto de contacto), así que no se ha
demostrado que el «contacto en línea» aporte algo con esta geometría de pelvis.

## Criterios de la issue

| Criterio | Estado |
|---|---|
| 3 × tope vertical lo hunde | cumplido (82 cm en 0,5 s) |
| 0,5 × tope lo mueve < 1 cm | **NO cumplido** con `kp = 20 000`: 3,7 cm (un muelle puro cede F/kp). Prueba `it.fails` con el número en el nombre. Opciones (no tomadas aquí): `kp ≥ 73 575` (con 100 000, 8,2 mm), un término integral acotado, o reinterpretar el criterio |
| Sin Uke, error < 5 mm | cumplido (0,34 mm), gracias a la alimentación directa |
| Contacto ≤ tope + m·a (±10 %) | cumplido con el término inercial `m·\|a − g\|` (relación máx 1,000) |
| Uke `tense` contra `bar`: desvío lateral < 2 cm en 2 s | cumplido (< 0,001 mm, trivial por simetría; con empuje lateral, 3,2 mm) |
| Segado: `post` con `max_speed_mps` | cumplido (2,0000 m/s con tope 2) |
| Simetría en Y | cumplido (4e-17 m) |

## Qué queda para PR posteriores

- **Integración en el escenario** (`schema.ts`, `validate.ts`, `buildModel.ts`, `run.ts`, grabación): efector `obstacle`, inserción tras Uke, llamada tras
  el controlador de Uke, grabar `ObstacleSample` y eventos, `summary` de picos. Otras PR editan esos ficheros.
- **Presupuesto de Tori y pista de Tori (PR 4):** sumar `reaction_n`; objetivos tomados de la pista en lugar de `phases`.
- **Resolver el criterio de 0,5 × tope** (rigidez, integral acotada o criterio), y **calibrar** todos los valores con datos y con los primeros derribos.
- **Estabilidad con Uke de otras tallas y con obstáculos ligeros** (pie, poste de pocos kg): solo se ha medido con 75 kg de Uke y la masa por defecto.
- **Contacto en línea real:** comprobar si la barra aporta algo frente a un punto (por ejemplo con una pelvis de más de un contacto, `multiccd`).
- **Mejora de segado/barrido contra una pierna de Uke** (solo se ha probado el límite de velocidad en el aire).
- Posible fallo existente de los props en `buildModel.ts` (ver «Orden en el MJCF»).
