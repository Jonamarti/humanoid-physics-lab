# Primer experimento de paso físico (issue #13)

Este incremento es **experimental, opt-in y separado del modo de pie normal**.
No afirma que el humanoide ya sea capaz de recuperar cualquier empujón.

## Secuencia

`ExperimentalStepSequence` coordina `STAND → SHIFT_WEIGHT → LIFT → SWING
→ LAND → RECOVER → COMPLETE`, con `ABORT` y `FALL` distintos:

1. Cambiar gradualmente la referencia del centro de masas hacia el pie de
   apoyo, mediante los pares ya acotados de los tobillos.
2. **Solo** empezar a levantar cuando el apoyo real supera el 70 % de
   las cargas verticales medidas (`efc_force` de MuJoCo).
3. Aplicar nuevos objetivos de ángulo de cadera (pitch y roll), rodilla y
   tobillo solo a las bisagras de la pierna libre, con `limitedJointTorque`.
4. **Solo** pasar de `LIFT` a `SWING` cuando la planta asciende físicamente
   más de 3 cm y la fuerza normal de ese pie baja de 25 N.
5. **Solo** preparar el aterrizaje tras un desplazamiento real >5,5 cm y
   elevación de la planta >2 cm. `LAND` a `RECOVER` requiere carga
   vertical real >40 N y la planta a menos de 2 cm de su altura inicial.
6. Completar exige carga de **ambos** pies, pelvis erguida y tiempo de
   readaptación. Los fallos por no transferir peso, no despegar o no
   aterrizar son `ABORT`; caída del cuerpo por límites es `FALL`.

El experimento **no usa root actuation, ninguna alteración de qpos ni
animaciones del pie**. Las medidas se toman cada 10 pasos de integración
(~20 ms con timestep 0,002 s); el estado de contacto de MuJoCo se
corresponde con el paso físico anterior (retardo máximo 2 ms).

## Limitación actual

- Todavía es un ensayo de paso *voluntario*, no la respuesta a un empujón.
  No hay corrección en lazo cerrado de pie objetivo mediante Jacobiano/IK:
  el desplazamiento XY inicial se transforma en objetivos aproximados de
  cadera/rodilla, limitados por la amplitud articular.
- Puede registrar `ABORT` sin levantar un pie, o `FALL` por pérdida de
  equilibrio. Los tests no fingen éxito ni fijan una trayectoria cinemática.
- Falta demostrar en pruebas físicas un paso completo robusto en los dos
  lados; después habrá que integrarlo al planificador y al visor.
- El planificador deja de declarar `FALL` por defecto cuando el COM
  está entre pies, y en su lugar propone `SHIFT_WEIGHT` hacia un apoyo
  viable, siempre que encuentre destino alcanzable.

## Pruebas

- `tests/unit/experimentalStep.test.ts`: se requieren cambios reales
  de carga, altura y contacto para avanzar de fase y completar.
- `tests/unit/experimentalStepMujoco.test.ts`: 2 escenarios (paso
  izquierdo y derecho) con masas reales y pares limitados, registrando
  los resultados físicos, incluidos fallos.
- La bipedestación y empujones previos deben conservar sus métricas.

## Resultado observado en CI — revisión de dirección y estabilidad

Registro de la rama experimental:
[GitHub Actions](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml)
(`tests/unit/experimentalStepMujoco.test.ts`,
`tests/unit/stepTransitionTrace.test.ts`).

Los primeros ensayos *sin comprobar desplazamiento firmado* indicaron
`COMPLETE` porque hubo elevación, cambio de contacto y aterrizaje. **Era
un falso positivo de aceptación**: un ensayo solicitado hacia delante
podía terminar con el pie desplazado hacia atrás.

Ahora se exige progreso firmado sobre la dirección objetivo, aterrizaje
bajo carga y recuperación sostenida. La versión probada registra
`ABORT` para ambos ensayos diagonales iniciales, aunque detectó
despegue real (~3,94 cm), vuelo y al menos un aterrizaje cargado.
Los barridos de seis direcciones adicionales detectan abortos y algunas
caídas físicas. **Ninguna de esas seis combinaciones ha superado aún
el criterio completo de marcha/colocación.**

Trazas representativas con controlador nominal preservado:

- Paso izquierda diagonal: transferencia a pie de apoyo en t≈1,36 s,
  despegue en t≈1,58 s, contacto en t≈2,14 s, pero el pie pierde
  el avance firmado antes de recuperar, por lo que se informa `ABORT`.
- Paso izquierda delante: el pie avanzó 11,7 cm en vuelo, tocó el suelo
  en t≈1,82 s, y después se deslizó hacia atrás, terminando en
  `ABORT`, no en «paso conseguido».
- Prueba de ponderación de pares de tobillo según GRF real: empeoró
  notablemente la estabilidad y terminó con caídas. Esa modificación
  ha sido **revertida**: el controlador nominal no cambia.

### Siguientes criterios de aceptación (pendientes)

- Control de posición del pie de vuelo en bucle cerrado (Jacobiano/IK
  con torques internos acotados), no solo comandos angulares heurísticos.
- Mantener soporte y la posición de aterrizaje: error al objetivo,
  deslizamiento de suela de apoyo y objetivo firmado tras 1 s.
- Realimentación del CoP/GRF y del COM sin alterar de forma brusca los
  puntos de equilibrio durante apoyos simples/dobles.
- Dos pies, desplazamientos anteriores/posteriores/laterales;
  recuperación probada contra perturbaciones independientes.
- No activar automáticamente esta secuencia para empujones hasta
  superar pruebas con despegue/contacto y sin caída.

La UI experimental permite iniciar y **observar un intento**, y
distingue `ABORT` de `FALL`. No se afirma marcha estable. El
benchmark puede pasar sus tests técnicos aunque la maniobra de
locomoción todavía falle: los tests registran el resultado real.

## Iteración: control cartesiano y apoyo posterior — 9 nov 2026

La fase de seguimiento de pie ahora dispone de:

- **Jacobiano analítico real del MuJoCo WASM:** columna de bisagra
  `axis_world × (site_world - joint_anchor_world)` y par
  `tau = Jᵀ F`, limitado a DOFs internos autorizados, sin aplicar
  fuerzas a la raíz ni alterar posiciones cinemáticamente.
- **Trayectoria de pie 3D por fases:** elevación, traslación XY y
  descenso suavizados; velocidad medida mediante diferencias por
  timestep y filtrado, con fuerza de corrección acotada.
- **Mediciones estrictas:** deslizamiento horizontal acumulado del pie
  de apoyo desde la postura inicial, error del pie hasta su destino,
  deslizamiento después del primer aterrizaje cargado, inclinación
  del tronco, altura mínima de pelvis y fuerzas reales por pie.
- **Referencia de equilibrio tras aterrizar:** se desplaza suavemente
  hacia el nuevo centro de la base de apoyo, calculado con la posición
  realmente alcanzada por el pie, en lugar de devolver instantáneamente
  el COM a la referencia anterior.
- **Comparación en el visor:** selector de intensidad de la impedancia
  cartesiana 0/20/50/100 %, **valor por defecto: 0 %**. El modo
  experimental no sustituye al controlador nominal de bipedestación.

### Resultados experimentales

Barrido de 12 pruebas (direcciones adelante, lateral y diagonal;
intensidades 0, 0.2, 0.5, 1) registradas en `STEP-CARTESIAN-SWEEP`:

- **0 de 12** alcanzaron `COMPLETE` con los criterios de destino,
  apoyo y estabilidad; todas abortaron o cayeron.
- Por ejemplo, con paso diagonal e intensidad 0.2, el error final
  horizontal fue aprox. **6,1 cm**, pero el pie llegó a deslizarse
  unos **15,4 cm** después del primer aterrizaje: sigue siendo fallo.
- Con algunas intensidades altas el humanoide perdió el equilibrio.
  No se puede inferir que subir la ganancia sea mejor.
- También se probó mantener rígidamente los ángulos articulares
  alcanzados al aterrizar; **aumentó las caídas y se revirtió**.

El barrido extenso se conserva, pero solo se ejecuta expresamente
con `RUN_LONG_BENCHMARKS=1 npm test`, para no duplicar en cada CI
12 simulaciones de 5,2 s. El CI ordinario sigue comprobando
contactos, fases y ausencia de NaN.

**Conclusión:** el Jacobiano es una base matemática y de ingeniería
verificada; la locomoción controlada no está resuelta. Siguiente
trabajo: controlar el pie de apoyo y el centro de presión bajo
fricción, estabilizar el nuevo soporte durante ≥1 s y luego comparar
ambas piernas/direcciones con pruebas de aceptación **que exijan
`COMPLETE` físico**, no simplemente un test sin excepciones.

## Revisión de la PR #45: contacto, fricción y éxito sostenido

La revisión posterior incorporó medidas que faltaban para distinguir
una simulación que *se ejecuta* de un paso realmente logrado:

- `mj_contactForce` y `mju_decodePyramid` decodificados mediante
  `efc_force` también para **fuerzas tangenciales**, no solo normales.
  Se informa fuerza de fricción horizontal, máximo y promedio
  ponderado de utilización del cono/pirámide por pie. Son **magnitudes
  del solver**, no mediciones de personas ni demostración automática
  de deslizamiento. Código fuente MuJoCo:
  [engine_core_util.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_core_util.c)
  y [engine_util_misc.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_util_misc.c).
- Mientras el pie está en vuelo, el servo cartesiano puede actuar.
  **Cuando recibe carga al aterrizar, su fuerza se atenúa;** se
  desactiva al pasar a `RECOVER`/`COMPLETE` para no seguir empujando
  horizontalmente un pie que ya apoya.
- Se exige **un segundo completo, muestreado cada ~20 ms**, con
  dos pies cargados, COM y capture point dentro del polígono de
  apoyo *cargado*, deriva de apoyo ≤2 cm, deslizamiento de aterrizaje
  ≤2 cm, error del destino ≤5 cm y velocidad horizontal de COM
  <0,12 m/s. Una interrupción o muestra omitida reinicia el intervalo;
  la postura momentánea no basta. Estos son **criterios de ingeniería
  experimentales**, no umbrales validados en humanos.
- Se conservan los registros de `ABORT` y `FALL`; una caída
  posterior a `ABORT` se reporta como `FALL`.

### Comparación con contacto físico

En `tests/unit/stepFrictionTrace.test.ts`, ambos ensayos de paso
diagonal izquierdo de 5,2 s siguen siendo `ABORT`:

| Medida | Servo 0 % | Servo 20 % |
| --- | ---: | ---: |
| Deslizamiento máximo tras aterrizar | 12,7 cm | 11,1 cm |
| Desplazamiento máximo del pie de apoyo | 5,1 cm | 0,5 cm |
| Fricción máxima usada por pie de llegada | 100 % | 100 % |
| Resultado aceptado | ABORT | ABORT |

Las fuerzas tangenciales alcanzan el límite de la pirámide del solver
alrededor del impacto. Hay que estudiar velocidad y fuerza normal
del primer contacto, control de par y apoyo bipodal; todavía no se
ha demostrado un paso estable.

### Experimento descartado

Se intentó retrasar `LAND` hasta una posición más cercana al destino
y extender suavemente la rodilla durante 0,55 s. Ambos ensayos diagonales
(0 % y 20 %) acabaron en **FALL**; antes eran `ABORT` con pelvis
estable. **Se revirtió** para conservar el comportamiento más seguro
y la reproducibilidad. No usar este resultado como evidencia de
mejora y no habilitar el control experimental por defecto.

### Próxima iteración (#49)

Usar el instante de contacto para controlar velocidad tangencial y
transferencia de cargas, contrastando la demanda de fricción con la
fricción disponible, en lugar de elevar indiscriminadamente los
pares o la rigidez de las articulaciones. Comparar con tests
de aceptación independiente y grabaciones reales del visor.
