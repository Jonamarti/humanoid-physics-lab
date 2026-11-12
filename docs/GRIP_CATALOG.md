# Catálogo de agarres

Issue #76, PR 2. Código: `src/effectors/gripCatalog.ts`. Pruebas: `tests/unit/gripCatalog.test.ts` (modelo generado en las cuatro tallas y Uke `relaxed` en MuJoCo WASM real).
Plan: [PLAN_DERRIBOS.md](PLAN_DERRIBOS.md) (secciones 3, 9 y 12.1, puntos 3 y 4). Mano virtual: [VIRTUAL_HAND.md](VIRTUAL_HAND.md).

Módulo **autónomo**, como la mano: no está conectado al formato de escenario ni al ejecutor (eso es una PR posterior) y no modifica `src/uke/`, `src/scenario/`, `src/metrics/` ni `src/recording/`.

## Qué es

Una mano virtual tiene un solo punto de agarre, pero la fuerza de un agarre real no se transmite por un punto: la solapa carga la nuca y la espalda alta, no el pecho (plan 12.1, punto 4). El catálogo dice, para cada agarre con nombre:

- el **tipo de unión** por defecto de la mano (`rope`, `clamp` o `push`);
- **dónde se reparte la fuerza** en el cuerpo de Uke: lista de puntos con su fracción (positivas, suman 1).

«Izquierda» y «derecha» son el lado **de Uke** (los nombres de sitio del humanoide), no el de la mano de Tori.

**Los repartos son de partida: sin calibrar y sin fuente biomecánica.** No hay en el repositorio ninguna medida del reparto de fuerza de un agarre de ropa o de miembro (la única cifra que cita el plan, Yilmaz 2015, es de fuerza máxima por mano, no de reparto). Cada reparto sale de la issue #76 y del razonamiento mecánico de la columna «Razón». No se ha contrastado con datos.

## Tabla

| Agarre | Unión | Reparto de la fuerza | Razón |
|---|---|---|---|
| `sleeve_left/right` | `rope` | 100 % en `*_sleeve_elbow` | La manga es tela: solo tracción. Se toma a la altura del codo y la tela lo transmite al antebrazo. Con Uke `tense` esto levanta el brazo sin mover el cuerpo, y es correcto (issue, «Trampas»). |
| `lapel_left/right` | `rope` | 70 % en `nape`, 30 % en el hombro **contrario** | La solapa rodea el cuello: carga la nuca y la espalda alta (plan 12.1, punto 4). La parte que cruza la espalda hacia el otro lado carga el hombro contrario. El 70/30 es el de la issue; no hay fuente. |
| `collar_back` | `rope` | 100 % en `nape` | El cuello trasero de la ropa tira de la nuca. |
| `belt_front`, `belt_back` | `rope` | 100 % en el punto del lado **contrario** de la pelvis (a la altura del cinturón) | El cinturón rodea la cintura y carga el lado contrario al que se toma (plan 12.1, punto 4). Requiere `grip_side`. |
| `wrist_left/right` | `clamp` | 100 % en `*_wrist` | La mano rodea la muñeca: tracción y empuje, todo en el extremo del antebrazo. |
| `triceps_left/right` | `clamp` | 100 % en `*_triceps` | La mano rodea la parte trasera del brazo: tracción y empuje, todo en el brazo. |
| `collar_tie` | `rope` | 100 % en `nape` | Mano en la nuca, el mismo reparto que `collar_back`. Unión `rope` según la tabla de la issue (ver «Desviaciones y dudas»). |
| `underhook_left/right` | `push` | 100 % en `*_armpit`, solo hacia arriba y hacia Tori | Empuje por contacto bajo la axila: `push` nunca tira y su eje fija la dirección (ver «Underhook»). |
| `waist_lock` | `clamp` | 50 % en cada lado de la pelvis | Los brazos rodean la cintura: la fuerza se reparte a partes iguales entre los dos lados, sin par por asimetría. |
| `knee_back_left/right` | `clamp` | 100 % en `*_knee_back` | La mano o el brazo rodea la corva: tracción y empuje, todo en la pierna. |
| `ankle_left/right` | `clamp` | 100 % en `*_ankle` | La mano rodea el tobillo: tracción y empuje, todo en el pie. |

Es la tabla de la issue; una prueba la compara con una copia independiente escrita en el propio test.

## API

Funciones puras salvo las tres últimas (usan un modelo MuJoCo a través de interfaces estructurales: no se importa MuJoCo).

```ts
GRIP_NAMES, GRIP_CATALOG                       // los 19 agarres y su definición (unión, reparto, razón, espejo)
resolveGrip(name, options?): ResolvedGrip       // RangeError claro si no existe o falta/sobra un dato
underhookAxis(toward_tori_m, elevation_deg?)    // eje de push de underhook_*; isUnderhookAxis lo comprueba
mirrorGripRequest(name, options?)               // agarre espejo con los datos de quien llama reflejados en Y
handConfigForGrip(grip, base): HandConfig       // pone link (y axis en push) para resolveHandParams
splitGripForce(shares, force_n): Vec3[]         // fracción · F por punto; la suma es F
equivalentGripPoint(shares, positions, velocities) // posición y velocidad ponderadas: lo que pide stepHand
gripTorqueAbout(positions, forces, about_m)     // Σ (p − c) × F respecto a un punto (el CM de Uke)
locateGrip(model, grip): LocatedGrip            // cuerpo y punto local de cada reparto; error si falta algo en el modelo
gripPointPositions / gripPointStates(model, data, grip)  // posición (y velocidad) de cada punto en el mundo
applyGripForce(data, grip, force_n): Vec3[]     // suma a xfrc_applied la parte de cada punto (con applyForceAtPoint)
```

Uso desde un ejecutor, una vez por paso físico y antes de `mj_step`, tras `mj_kinematics`, `mj_comPos` y `mj_comVel`:

```ts
const grip = locateGrip(model, resolveGrip("lapel_left"));
const params = resolveHandParams(handConfigForGrip(grip, { shoulder_m, phases }));
let state = createHandState();
// en cada paso, con xfrc_applied a cero:
const { points_m, velocities_mps } = gripPointStates(model, data, grip);
const eq = equivalentGripPoint(grip.shares, points_m, velocities_mps);
const r = stepHand(params, state, { t, dt, grip_m: eq.point_m, grip_velocity_mps: eq.velocity_mps });
state = r.state;
applyGripForce(data, grip, r.force_n);   // la suma de lo aplicado es r.force_n
// r.reaction_n (= −F) sigue siendo la reacción de Tori para el presupuesto
```

La mano ve **un** punto de agarre (el equivalente) y calcula **una** fuerza con sus topes; el catálogo solo decide cómo se reparte esa fuerza. La reacción de Tori sigue siendo `−F`, entera: repartir no crea ni quita fuerza.

### Qué sitios usa y qué hacer cuando falta uno

Los agarres usan los sitios con nombre del humanoide (`nape`, `*_sleeve_elbow`, `*_wrist`, `*_triceps`, `*_armpit`, `*_knee_back`, `*_ankle`, `belt_front`, `belt_back`; ver `HUMANOID_SITE_BODY` en `src/scenario/schema.ts` y [UKE_MODEL.md](UKE_MODEL.md)). Tres repartos nombran puntos que **no existen como sitio** en el MJCF generado. No se ha editado `src/uke/generate.ts` (otro agente puede necesitarlo estable): cada uno se resuelve en `locateGrip` a **cuerpo + punto local** calculado con datos que el modelo ya tiene:

| Punto que falta | Dónde se usa | Cómo se resuelve ahora (por tipo de ancla) | Sitio que habría que añadir |
|---|---|---|---|
| «Hombro contrario» | `lapel_*` (30 %) | `child_origin`: cuerpo `chest`, punto = origen del cuerpo `<lado contrario>_upperarm` en el sistema de `chest`, es decir, la articulación del hombro `(0, ±shoulderY, shoulderZ)`. | `left_shoulder_top` y `right_shoulder_top` (o nombre equivalente) en el cuerpo `chest` en `(0, ±shoulderY, shoulderZ)`, quizá subidos a la superficie superior del hombro. |
| «Cada lado de la pelvis» | `waist_lock` (50 % + 50 %) | `geom_semiaxis`: cuerpo `pelvis`, punto `(0, ±b, 0)` con `b` el semieje Y del elipsoide `pelvis_geom` (leído del modelo; requiere el elipsoide alineado con el cuerpo, si no lanza error). Cae exactamente sobre la superficie. | `pelvis_side_left` y `pelvis_side_right` en `pelvis`, en `(0, ±pelvisA[1], 0)`. |
| «Sitio del lado contrario de la pelvis» | `belt_front`, `belt_back` | `site_lateral`: cuerpo `abdomen`, punto `(0, ±hypot(x, y), z)` de `belt_front` o `belt_back`: el sitio de cinturón girado 90° hasta el lado contrario (mismo radio y altura). Cae sobre la cápsula del abdomen, a la altura del cinturón. | `belt_side_left` y `belt_side_right` en `abdomen`, en `(0, ±abR, 0.04·st)`. |

No se ha usado `*_shoulder_impact` para el hombro: está en el brazo (`*_upperarm`), 4 cm bajo la articulación, y cargaría el brazo y no el tronco. Una prueba comprueba que los únicos agarres que no se apoyan solo en sitios existentes son `lapel_*`, `belt_*` y `waist_lock`, y otra que todo sitio, cuerpo y geometría del catálogo existe en el modelo generado con 55 kg / 1,55 m, 75 / 1,75, 100 / 1,95 y 150 / 2,00.

Cuando se añadan esos sitios, basta cambiar el ancla de esos tres agarres a `{ kind: "site", site: ... }`; la API y las pruebas de fracciones, espejo y par no cambian.

## Datos que da quien llama

El catálogo no conoce a Tori ni a la mano que agarra. Dos agarres necesitan un dato:

- **`belt_front`, `belt_back`: `grip_side`** (`"left"` o `"right"`): lado de Uke por el que la mano toma el cinturón. La fuerza va al lado **contrario**. La issue no da el lado (la tabla dice «el sitio del lado contrario de la pelvis» sin decir contrario a qué), y es lo único que permite elegir un lado: se interpreta como «contrario al lado del agarre», igual que «el hombro contrario» de la solapa. Sin `grip_side` hay error.
- **`underhook_left/right`: `toward_tori_m`** y, opcionalmente, `underhook_elevation_deg`. Ver abajo.

Pasar un dato a un agarre que no lo usa también es un error (así un fallo de nombre no pasa en silencio).

### Underhook: «solo hacia arriba y hacia Tori»

La restricción se expresa con el **eje `axis` de la unión `push`** (ver [VIRTUAL_HAND.md](VIRTUAL_HAND.md), «Interpretación de `push`»): `push` solo empuja, y la fuerza sale siempre a lo largo de `axis` en sentido positivo; nunca tira y nunca sale en otra dirección. Basta entonces fijar `axis` a un vector que sea combinación no negativa de «arriba» y «hacia Tori»:

```
axis = cos(e) · ĥ + sin(e) · ẑ        con ĥ = parte horizontal de toward_tori_m, normalizada
```

- **«Hacia Tori» es un dato de quien llama.** En HPL Tori no tiene cuerpo (sección 3 del plan: «Tori abstracto»), así que el catálogo no puede calcularlo. Quien llama da `toward_tori_m`: el vector de Uke hacia Tori en el marco en el que trabaja la mano (normalmente el mundo; por ejemplo, de la posición de Uke a la de los pies de Tori del escenario, o a `shoulder_m` de la mano). Solo cuenta su parte horizontal; si es nula o no finita, error. Como `axis` se fija una vez, no sigue a Tori si éste se mueve: si el consumidor necesita lo contrario, tendrá que reconstruir la mano (limitación de `push` en la PR 1: el eje se fija al cerrarse).
- `e` es la elevación sobre el plano horizontal, de 0° (solo hacia Tori) a 90° (solo arriba). **Por defecto 45°** (`UNDERHOOK_DEFAULT_ELEVATION_DEG`): es un punto de partida sin calibrar.
- `isUnderhookAxis(axis, toward_tori_m)` comprueba que un eje cumple la restricción (unitario, sin componente lateral y con las dos componentes no negativas). Una prueba recorre objetivos de la mano en todas las direcciones y comprueba con `stepHand` real que la fuerza resultante es siempre paralela al eje, con componente lateral nula y componentes hacia arriba y hacia Tori no negativas, y que hay objetivos (detrás, abajo) que no producen fuerza.

## Espejo

Izquierda y derecha son espejo exacto: mismas fracciones, mismos puntos reflejados en Y y misma unión. `mirrorGripRequest(name, options)` da el agarre espejo con `grip_side` invertido y `toward_tori_m` reflejado en Y. `collar_back`, `collar_tie`, `waist_lock` son su propio espejo (el reparto de `waist_lock` es simétrico por construcción); los cinturones lo son con `grip_side` invertido.

## Medidas

Todas con el modelo generado y MuJoCo WASM real, en `tests/unit/gripCatalog.test.ts`.

**Par respecto al centro de masas** de Uke de pie en postura neutra con 300 N hacia delante (+X), en N·m, componentes `[x, y, z]` (y = cabeceo, z = guiñada):

| Talla | `lapel_left` | `sleeve_left` | `collar_back` |
|---|---|---|---|
| 55 kg / 1,55 m | [0, 140,9, 18,5] | [0, 32,5, −72,8] | [0, 150,1, 0] |
| 75 kg / 1,75 m | [0, 159,1, 20,3] | [0, 36,7, −80,0] | [0, 169,4, 0] |
| 100 kg / 1,95 m | [0, 177,3, 22,2] | [0, 40,9, −87,5] | [0, 188,8, 0] |
| 150 kg / 2,00 m | [0, 179,2, 28,9] | [0, 39,3, −112,6] | [0, 191,0, 0] |

La misma fuerza, repartida de otro modo, da un par de cabeceo entre 4,3 y 4,6 veces mayor por la solapa que por la manga (se aplica más arriba) y una guiñada de signo contrario (la manga izquierda queda a la izquierda del cuerpo; la solapa izquierda carga el hombro derecho).

**Caída libre, Uke `relaxed`.** Montaje: postura neutra con la raíz elevada 1 m, velocidad cero, amortiguación pasiva de `relaxed` y sin controlador; 40 pasos (80 ms) de 300 N hacia +X; cae unos 3 cm sin tocar nada. La gravedad uniforme no da par respecto al centro de masas, así que lo único que cambia el momento angular son los efectores. Se mide el momento angular total (`subtree_angmom`) y la velocidad angular de `chest`.

| Talla | Aceleración angular media del tronco (rad/s²), cabeceo / guiñada, solapa | idem, manga | Diferencia solapa − manga, tronco | Diferencia de par previsto (N·m) |
|---|---|---|---|---|
| 55 kg | 80,4 / 62,9 | −22,6 / −137,4 | 103,1 / 200,3 | 108,5 / 91,3 |
| 75 kg | 54,6 / 42,8 | −16,4 / −84,1 | 71,0 / 126,8 | 122,5 / 100,3 |
| 100 kg | 37,9 / 29,7 | −11,7 / −54,1 | 49,6 / 83,8 | 136,4 / 109,8 |
| 150 kg | 27,4 / 18,5 | −8,5 / −33,4 | 35,9 / 51,8 | 139,9 / 141,4 |

En las cuatro tallas la respuesta del tronco difiere en el sentido que predice el reparto (signo de la diferencia de pares en cabeceo y guiñada). No se compara el tamaño porque el tronco no es un sólido: hay articulaciones relajadas entre la pelvis, el pecho, la cabeza y los brazos. Con la manga, el tronco incluso cabecea hacia atrás (−16,4 rad/s²) aunque el par total sobre el cuerpo sea hacia delante (+36,7 N·m): el brazo y el antebrazo absorben parte del golpe. Es un resultado que conviene tener presente para la PR de escenarios.

Comprobación del par: el momento angular total respecto al CM medido en MuJoCo coincide con `∫ Σ (p − c) × F dt` calculado paso a paso con el error del integrador (menos del 0,3 % con la solapa y entre el 0,8 % y el 2,2 % con la manga, donde el antebrazo oscila; al dividir el paso entre 4 el error se divide por 4, medido una vez). La prueba admite el 3 %.

**Simetría dinámica.** Agarre izquierdo con la fuerza reflejada en Y frente al derecho (`lapel_left`, `sleeve_left`, `belt_front`, `waist_lock`, `underhook_left`, `knee_back_left`): la mayor diferencia entre las posiciones de todos los cuerpos reflejadas es inferior a 5·10⁻¹⁶ m y la de la velocidad angular del tronco inferior a 3·10⁻¹⁵ rad/s (umbral de la prueba: 10⁻⁹).

**Velocidad de los puntos.** La velocidad de cada punto del reparto, a partir de `cvel` (la misma fórmula que `src/scenario/run.ts`), frente a la derivada numérica de su posición por diferencias centradas con `mj_integratePos`: error inferior a 3·10⁻¹⁰ m/s (umbral 10⁻⁶ m/s) con puntos a 0,43 y 0,64 m/s.

**Con la mano.** `lapel_left` con una mano `rope` en caída libre (pico 500 N, el tope): la suma de `xfrc_applied` de todos los cuerpos coincide con la fuerza de la mano en cada paso con una diferencia máxima de 8·10⁻¹⁴ N.

## Desviaciones y dudas respecto a la issue

- **`grip_side` en los cinturones.** Es un dato que la tabla no da y hace falta para elegir un lado (ver arriba). Interpretación mía, no de la issue.
- **Ancla del «hombro contrario»** (articulación del hombro en el sistema de `chest`) y de los lados de la pelvis (punto en la superficie del elipsoide): no existen como sitio; ver la tabla de sitios que faltan.
- **`collar_tie`.** La tabla de la PR 2 dice `rope`; la PR 1 lo cita como `rope` («ropa, collar tie») y también incluye «nuca con la mano» entre los usos de `clamp`. Se ha seguido la tabla de la PR 2. Si el collar tie es una mano sobre la nuca (no ropa), `clamp` sería más fiel: cambiar `link` en una línea.
- **`collar_back` y `collar_tie` son idénticos** (100 % en `nape`, `rope`). Se mantienen como dos nombres por la semántica.
- **`belt_front` y `belt_back` dan el mismo reparto** (la cápsula del abdomen es circular, los sitios son simétricos). Lo que cambia entre ellos es dónde está la mano, que lo da la posición de la mano, no el reparto.

## Qué no se ha comprobado

- Que los repartos se parezcan a los de un agarre real: no hay fuente ni medida.
- Que el punto equivalente (media ponderada) sea el mejor punto de agarre para el muelle de la mano: es el más simple que conserva la fuerza total. Con Uke `tense` y con contactos, el efecto del reparto sobre el derribo no se ha medido (las pruebas dinámicas son con Uke `relaxed` en caída libre).
- La estabilidad numérica del muelle con la masa efectiva del punto equivalente (pendiente desde la PR 1: ver «Estabilidad» en [VIRTUAL_HAND.md](VIRTUAL_HAND.md)).
- Que el eje por defecto de `underhook_*` (45°) sea razonable.

## Qué queda para PR posteriores

- Integración en el escenario y el ejecutor (efector `virtual_hand` con `grip: "lapel_left"`, marcos, grabación): no está hecha.
- Añadir al generador los sitios de la tabla «sitios que faltan» y cambiar las tres anclas.
- Presupuesto de Tori (PR 4): sigue siendo `Σ reaction_n` de las manos; repartir la fuerza no lo cambia.
