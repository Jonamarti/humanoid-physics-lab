# Mano virtual

Issue #76, PR 1. Código: `src/effectors/hand.ts`. Pruebas: `tests/unit/handEffector.test.ts` (cuerpo de 10 kg y de otras masas en MuJoCo WASM real).
Plan: [PLAN_DERRIBOS.md](PLAN_DERRIBOS.md) (secciones 3, 9 y 12.1).

Esta PR entrega el módulo **autónomo**: no está conectado todavía al formato de escenario ni al ejecutor (ver «Qué queda»). No depende de
`src/control/`, `src/physics/` ni de `src/scenario/`.

## Qué es

Una mano es un punto objetivo `p_t(t)` unido por muelle y amortiguador a un punto de agarre `p_g` del cuerpo de Uke. Sustituye a la fuerza
prescrita (que aporta energía sin límite) como efector normal de Tori: tiene tope de fuerza, tope de potencia, alcance máximo desde un hombro y
suelta. La fuerza prescrita queda solo para las pruebas analíticas del cilindro.

## API

Funciones puras y deterministas; el estado es inmutable (cada paso devuelve un estado nuevo).

```ts
resolveHandParams(config: HandConfig): HandParams        // valida y aplica los valores por defecto (lanza RangeError)
createHandState(): HandState
stepHand(params, state, { t, dt, grip_m, grip_velocity_mps }): HandStepResult
//   → { force_n, reaction_n, state, sample, events }
settleHandWork(state, grip_velocity_mps): HandState      // cierra el trabajo del último paso
applyForceAtPoint(data, bodyId, point_m, force_n): void  // suma fuerza y par r × F a xfrc_applied
handTargetOffset(phases, t): Vec3                         // objetivo por fases (mínimo tirón), sin estado
```

Uso desde un ejecutor, una vez por paso físico y antes de `mj_step` (como los efectores actuales de `src/scenario/run.ts`):

```ts
const params = resolveHandParams({ link: "rope", shoulder_m, phases: [{ start_s: 0.3, duration_s: 0.4, offset_m: [0.3, 0, 0.1] }] });
let state = createHandState();
// en cada paso, tras mj_kinematics y mj_comPos, con la posición y la velocidad del punto de agarre en el instante t:
const r = stepHand(params, state, { t, dt, grip_m, grip_velocity_mps });
state = r.state;
applyForceAtPoint(data, bodyId, grip_m, r.force_n);   // xfrc_applied puesto a cero al empezar el paso
// grabar r.sample y r.events; r.reaction_n (= −F) es la reacción de Tori para el futuro presupuesto
// al terminar: state = settleHandWork(state, grip_velocity_mps)
```

Convenciones:

- Unidades SI. Todos los vectores (agarre, objetivo, desplazamientos, hombro, eje) están en el **mismo marco**, que elige quien llama
  (normalmente el mundo). El módulo no convierte entre marcos: la conversión desde `actor_initial` de Uke o desde el marco de Tori es cosa de la
  integración.
- `t` es el instante en que se miden la posición y la velocidad (el comienzo del paso `[t, t + dt]`); la fuerza devuelta se mantiene constante en
  todo el paso. El objetivo se evalúa en `t`.
- `grip_m` y `grip_velocity_mps` son los del punto de agarre, no los del centro de masas del cuerpo. `applyForceAtPoint` usa `xipos` (centro de
  masas) y deja el par `(p − c) × F` en `xfrc_applied`: es la matemática de `src/scenario/run.ts` y de `src/physics/mujoco.ts`.

## Fórmulas

Con `d = p_t − p_g` (del agarre al objetivo, tras recortar el objetivo al alcance), `L = |d|`, `d̂ = d / L` y `v_g` la velocidad del agarre:

| Unión | Fuerza sobre el agarre |
|---|---|
| `rope` (ropa; solo tracción, con holgura) | si `L ≤ slack_m`: 0. Si no, `F = [k·(L − slack_m) − c·(v_g · d̂)] · d̂`; si el corchete es negativo, 0 |
| `clamp` (muñeca, tobillo, corva; tracción y empuje) | `F = k·d − c·v_g` (sin holgura) |
| `push` (mano abierta, antebrazo; solo empuje) | con eje unitario `n̂` y `s = n̂ · d`: si `s ≤ slack_m`, 0. Si no, `F = [k·(s − slack_m) − c·(v_g · n̂)] · n̂`; si el corchete es negativo, 0 |

`rope` nunca empuja: la fuerza apunta siempre hacia el objetivo y es 0 si el objetivo está a menos de `slack_m` del agarre. `push` nunca tira:
la fuerza apunta siempre a lo largo de `n̂` en sentido positivo. `clamp` es un muelle-amortiguador completo en 3D.

**Interpretación de `push` (a revisar).** La issue dice «solo empuje, igual que `rope` con el signo contrario». Un muelle hacia el objetivo
da la misma geometría para empujar que para tirar, así que la diferencia solo puede estar en la condición unilateral. Se ha implementado `push`
como el muelle unilateral de `rope` **medido sobre un eje de empuje `n̂`** en lugar de sobre la línea mano–agarre: la fuerza es normal al
contacto, nunca tira y es 0 si el contacto se pierde (objetivo por detrás del agarre en el eje, aunque el agarre se aleje deprisa). `n̂` es
`axis` (configurable; el catálogo de agarres de la PR 2 lo necesitará para «solo hacia arriba y hacia Tori») o, por defecto, la dirección del
hombro al agarre en el instante de cerrarse la mano (la mano empuja alejándose de Tori), fijada en el estado. `push` sin `axis` con el agarre
sobre el hombro lanza error. Si se prefiere otra lectura del «signo contrario», cambia una sola función (`linkForce`).

## Límites y su orden

Aplicados por paso, en este orden:

1. **Alcance.** `p_t` se recorta a la esfera de radio `reach_m` alrededor de `shoulder_m` (se mantiene la dirección desde el hombro). Se registra
   el objetivo con y sin recortar. Si el propio agarre está fuera de la esfera, el objetivo recortado queda dentro y el muelle tira hacia él.
   El alcance limita el **objetivo**, no el cuerpo: un cuerpo de 10 kg arrastrado a gran velocidad contra el límite se pasa por su cantidad de
   movimiento (medido: 0,886 m del hombro con un alcance de 0,75 m) y vuelve (final 0,728 m).
2. **Tope de fuerza.** Si `|F| ≥ max_force_n`, la fuerza se reduce a `max_force_n` (conservando la dirección) y el paso cuenta como «en el tope».
3. **Tope de potencia.** Si `F · v_g > max_power_w`, `F` se multiplica por `max_power_w / (F · v_g)`. Solo se limita la potencia positiva: una
   fuerza que frena el agarre no se recorta.

(La issue enumera fuerza, potencia y alcance. El alcance actúa sobre el objetivo, antes de calcular la fuerza, porque es el único sitio donde
puede actuar; fuerza y potencia siguen el orden de la issue. Ambos topes reducen el módulo de `F` sin cambiar su dirección, así que el resultado
numérico es el mismo en cualquier orden: `|F| = min(demanda, max_force_n, max_power_w / (v_g · F̂))`. El orden solo decide qué bandera
(`force_limited`, `power_limited`) se levanta; no se ha podido, y no hace falta, probar un orden contra otro.)

La potencia se calcula con la velocidad **al comienzo del paso** (la que ve el ejecutor antes de `mj_step`). La velocidad media del paso es algo
mayor cuando el cuerpo acelera, así que la potencia «real» puede pasar del tope por una fracción pequeña. Medido con 10 kg: `F · v_inicio`
máximo 600,000 W (el tope, exacto por construcción) y `F · v_media` máximo 603,2 W (+0,53 %); ver la tabla de medidas. Un cuerpo ligero acelera
más por paso y la desviación crece como `F² · dt / (2 · m · P)`. El trabajo acumulado se calcula con el trapecio de velocidades, como el
ejecutor, y coincide con la energía cinética ganada (diferencia < 1e-6 J).

## Suelta

- **Programada:** `release_s`. En el primer paso con `t ≥ release_s` la fuerza es 0 y se registra el evento `release`.
- **Automática («resbala»):** si la **demanda** del muelle está en el tope de fuerza (`|F| ≥ max_force_n` antes de limitar) durante `slip_after_s`
  seguidos, el siguiente paso es el de la suelta: fuerza 0 y evento `slip`. El tiempo en el tope se reinicia en cuanto un paso queda por debajo.
  Se cuenta la demanda y no la fuerza ya recortada por potencia: con un agarre que se aleja deprisa el tope de potencia baja la fuerza, y la mano
  debe resbalar igual (es el caso «mano que se aleja 2 m» de la PR 4). Una mano soltada no vuelve a agarrar.
- El cierre de la mano también deja un evento (`close`, en `close_s`). Antes de `close_s` la mano no actúa.

## Objetivo por fases

`phases` es una lista ordenada y sin solapes de `{ start_s, duration_s, offset_m }`. `offset_m` es el desplazamiento **acumulado** respecto al
punto de agarre al cerrarse la mano. Durante el tramo el desplazamiento pasa del valor del tramo anterior (0 al principio) a `offset_m` con el
perfil de mínimo tirón `10τ³ − 15τ⁴ + 6τ⁵` (velocidad y aceleración nulas en los extremos); entre tramos se sostiene. Ningún tramo puede empezar
antes de `close_s`. El origen es la posición del agarre en el instante del cierre, no la inicial.

## Reacción de Tori

Cada paso devuelve `reaction_n = −F`. El estado acumula el impulso de la fuerza (`impulse_ns`), el de la reacción (`reaction_impulse_ns = −impulse_ns`),
el pico de fuerza, el pico de potencia, el trabajo y el tiempo en el tope. El presupuesto de Tori (PR 4) sumará las reacciones de todas las manos.

## Valores por defecto y origen

| Parámetro | Valor | Origen |
|---|---|---|
| `k_n_per_m` | 3000 N/m | Issue #76. Sin fuente propia: punto de partida a calibrar |
| `c_ns_per_m` | 200 N·s/m | Issue #76. Da razón de amortiguamiento 0,58 con 10 kg y 1,83 con 1 kg. Punto de partida a calibrar |
| `slack_m` (`rope`) | 0,03 m | Issue #76. Holgura de la tela; sin medida propia. `clamp` y `push` usan 0 |
| `max_force_n` | 500 N | Issue #76: 470–620 N por mano (Yilmaz 2015, citado en la issue y en el plan, sección 3). No se ha vuelto a consultar la fuente |
| `max_power_w` | 600 W | Issue #76. No hay fuente en el repositorio; es un orden de magnitud para un tirón breve (= 500 N a 1,2 m/s) |
| `reach_m` | 0,75 m | Issue #76. Alcance del brazo desde el hombro; sin medida propia |
| `slip_after_s` | 0,4 s | Issue #76. Sin fuente propia |

**Todos son puntos de partida a calibrar, no verdades** (plan, secciones 3 y 12.5). Ninguno se ha calibrado con datos; las pruebas solo verifican que
el módulo respeta sus propios topes con estos valores, no que sean los de una persona.

## Estabilidad con los valores por defecto (medida)

La issue avisa de que un muelle rígido con paso de 2 ms puede oscilar con segmentos ligeros. Medido con MuJoCo WASM (`implicitfast`, paso 2 ms,
cuerpo libre de masa `m` sobre una deslizadera, sin gravedad, escalón de 10 cm del objetivo en 20 ms, `k = 3000`, `c = 200`, topes desactivados
para no enmascarar nada; `tests/unit/handEffector.test.ts`):

| Masa | `clamp` | `rope` |
|---|---|---|
| 10 kg | estable | estable |
| 5 kg, 2 kg | estable | estable |
| 1 kg | estable (sobreamortiguado) | estable |
| 0,5 kg, 0,25 kg | estable | estable |
| 0,2 kg | **diverge** | **diverge** |
| 0,1 kg | **diverge** | **diverge** |

**Con 10 kg y con 1 kg los valores por defecto no oscilan creciendo** (la envolvente del error decae y el cuerpo acaba en el objetivo, o dentro de la
holgura con `rope`). El límite es de otra naturaleza que el de la issue: con la fuerza evaluada al comienzo del paso (esquema explícito), la velocidad
se multiplica por `1 − c·dt/m` en cada paso por el amortiguador solo, y eso es inestable si `c·dt/m > 2`, es decir `m < c·dt/2 = 0,2 kg` con los valores
por defecto (medido: 0,2 kg diverge, 0,25 kg no). Análisis lineal de `clamp` con el esquema semiimplícito (con `a = c·dt/m` y `b = k·dt²/m`, comprobado
con el radio espectral de la matriz de paso, calculado aparte y sin dejar el script en el repositorio): estable si `a < 2` y `b < 4 − 2a`; con 0,25 kg
el radio espectral es 0,970; con 0,21 kg, 0,970; con 0,2 kg, 1,030 (inestable); con 0,1 kg, 3,09. **Para ese límite subir la amortiguación empeora la inestabilidad**: la consigna de la issue («sube la
amortiguación antes que bajar el paso») es correcta cuando el muelle es el que oscila (poco amortiguado), pero no para masas muy ligeras. Propuesta,
sin implementar aquí (necesita la masa efectiva en el agarre, que es de la integración): limitar `c ≤ m_eff / dt` (factor 2 de margen sobre el límite
teórico `2·m_eff / dt`) y comprobar `k·dt² / m_eff < 2` (con `a ≤ 1`, es la condición `b < 4 − 2a` de arriba); si el segmento es demasiado ligero para
eso, bajar el paso es la salida. La masa efectiva
real de un antebrazo con su cadena articulada no es la de un cuerpo libre y **no se ha medido sobre Uke**; hay que repetir esta prueba con el humanoide
cuando se integre la mano (PR posterior).

`rope` no amortigua mientras está dentro de la holgura (la cuerda está floja: es el modelo, no un defecto): un cuerpo puede seguir a la deriva
despacio dentro de la holgura (medido: 1,1 cm/s a 3,5–5 s con 10 kg) hasta que sale de ella y la cuerda lo frena.

## Qué queda para PR posteriores

- **Integración en el escenario** (`schema.ts`, `validate.ts`, `run.ts`, grabación): efector `virtual_hand`, conversión de marcos (`actor_initial`
  de Uke o marco de Tori), grabar `HandSample` y los eventos, resumen de picos. Se ha dejado fuera a propósito: otras PR editan esos ficheros.
- **PR 2 (catálogo de agarres):** repartir la fuerza de una mano en varios puntos del cuerpo de Uke.
- **PR 3 (obstáculo con fuerza limitada).**
- **PR 4 (presupuesto de Tori y pista de Tori):** sumar `reaction_n` de todas las manos, comprobar rozamiento, tope total de tiro, potencia total e
  impulso; objetivos tomados de una pista en lugar de `phases`.
- **Calibrar** los valores por defecto con datos y con los primeros derribos.
- **Masa efectiva en el agarre** y ajuste de `c` para segmentos ligeros (sección anterior), medido sobre el humanoide.
- **Resbalón físico del agarre:** hoy el «resbalón» es solo una suelta; no hay deslizamiento del punto de agarre sobre la tela.
