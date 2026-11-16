# Laboratorio de controladores y paso natural — 16-11-2026

## Resultado actual

Se mantiene MuJoCo. Hay adaptadores CPU reproducibles para ProtoMotions y MuJoCo MPC, una referencia de PD directo, grabaciones físicas y reproducción sin integración. El laboratorio está aislado: no cambia el controlador de Uke ni la ejecución del visor web.

El ensayo **ProtoMotions natural v3** baja los brazos y flexiona ligeramente los codos. Da un paso izquierdo observado de **26,69 cm**, eleva toda la suela **3,67 cm**, aterriza y conserva la estabilidad durante el último segundo. El apoyo contrario está cargado durante el despeje. No hay fuerzas externas ni se escriben posiciones o velocidades de la raíz durante la ejecución.

Esto acredita un paso voluntario guiado en este caso. No acredita marcha continuada ni recuperación reactiva ante empujones. La colocación lateral termina a **5,20 cm** de su posición inicial: excede por 2 mm la tolerancia de 5 cm del objetivo. Por eso se conserva el resultado completo del objetivo como **incompleto**, aunque el ciclo físico de paso sí está observado. La prueba separada de postura natural también señala 2,44 cm de movimiento del pie cargado durante el asentamiento inicial; no se oculta ni se relaja el umbral de 2 cm.

El vídeo frontal/perfil está en:
`out/protomotions-natural-v3/delivery/paso-natural.mp4`.
Primero velocidad real, después ×0,25. La revisión de geometría está en
`out/protomotions-natural-v3/protomotions/natural_step_left/physical-review.json`.
Se mide la distancia mínima al suelo de **todos los volúmenes de pie y dedos**. La altura del tobillo sola daba falsos positivos cuando la punta seguía en el suelo.

## Comparación inicial conservada

Mismo cuerpo, referencias, estado completo inicial, contactos, motor y perturbaciones; 9 casos por controlador. Las 18 comparaciones contra PD directo pasan las comprobaciones de identidad. Los impulsos de prueba se aplican al pecho durante 160 ms, integrando exactamente 25, 38,4 u 80 N·s.

| Controlador | Casos completos sin artefactos | Observación |
|---|---:|---|
| PD directo del laboratorio | 0/9 | No sostiene este cuerpo con seguimiento articular simple. |
| Adaptador MPC, presupuesto fijado | 0/9 | Esta configuración experimental no sostiene la postura. No demuestra incapacidad de la biblioteca MPC. |
| ProtoMotions, primeras referencias | 2/9 | Postura y sentadilla; paso y transferencia incompletos, perturbaciones fuertes con caída. |

El PD de esta tabla no es el controlador original de Uke aplicado a otro rig. No se comparan rendimientos entre cuerpos diferentes.

MPC utiliza el planificador SamplingPlanner oficial, 32 muestras, horizonte de 48 pasos a 120 Hz, control a 30 Hz y ejecución física compartida a 960 Hz. Se conserva la corrección entre iteraciones. Su puente C++ compila únicamente los componentes necesarios; no requiere gRPC, GPU ni WSL. Se documentan dos adaptaciones de compatibilidad: semilla reproducible por candidato/tiempo y el argumento opcional de normal de `mj_ray` de MuJoCo 3.5.

## Qué aporta y qué falta

- Modelo publicado: SOMA anatómico, 52 articulaciones accionadas y 42,47 kg. Sus límites y motores son decisiones de ingeniería; no es una validación médica del cuerpo humano.
- Uke usa otro esqueleto. El controlador entrenado no se puede copiar directamente: hacen falta adaptación de observaciones, acciones y referencias, y posiblemente entrenamiento.
- En dos cuerpos dinámicos se han ensayado objetivos de brazo, pierna y apoyo corporal, con equilibrio aprendido observado desde cada actor. Solo integra la simulación compartida; las copias de observación no integran física. El control negativo reproduce los mismos objetivos articulares con contactos cruzados desactivados.
- El ensayo de brazo no consigue contacto en esta configuración. Pierna y apoyo cambian el impulso y la respuesta al activar contactos, pero presentan artefactos; no son técnicas validadas ni una demostración de ausencia universal de penetraciones.
- Después de caída, un tracker que sigue pidiendo estar de pie puede introducir trabajo indeseado. No sustituye al reflejo protector corregido en la PR #108.

Siguientes hitos: colocar el pie sin deriva lateral, eliminar el deslizamiento del asentamiento, repetir con la otra pierna y encadenar apoyos. Después: disparar y gobernar fases con cargas y contactos medidos para recuperar perturbaciones.

## Reproducción y condiciones de uso

Ver [las instrucciones del laboratorio](../scripts/controller_lab/README.md). Las dependencias, pesos y vídeos permanecen en `out/`; no se añaden al producto ni al paquete npm.

Código [ProtoMotions](https://github.com/NVlabs/ProtoMotions/tree/13de0cccd06f53ce5c9cf9ec7d1932c7da73610f) y [MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc/tree/ff572a21e7c2bf9fda62e1862a758da7e9a8719b): Apache-2.0. Los pesos publicados tienen [NVIDIA Open Model License](https://github.com/NVlabs/ProtoMotions/blob/13de0cccd06f53ce5c9cf9ec7d1932c7da73610f/legal/MODEL-LICENSING.md).

El tracker publicado fue entrenado por NVIDIA con Motion Data by [Bones Studio](https://bones.studio/). El dataset tiene condiciones separadas. La demostración inicial se ejecutó antes de completar esta comprobación; su muestra y salidas quedan apartadas de la entrega. **La reproducción del laboratorio no descarga ni usa movimientos BONES-SEED, AMASS o SMPL.** Las referencias se generan aquí mediante cinemática inversa y parámetros propios sobre primitivas del modelo Apache-2.0.

## Verificación

706 pruebas del proyecto correctas, 9 omitidas y 3 pendientes; typecheck, lint y build correctos. El laboratorio añade pruebas de integridad de comparación, falsos pasos durante caída, conservación del tipo de actuador al exportar y medición real de la suela. Los vídeos se decodifican y su proceso de reproducción prohíbe `mj_step`.

Se guardan tablas, hipótesis, controles negativos y resultados fallidos. Los indicadores de prueba son criterios de ingeniería, no límites de lesión ni certificaciones biomecánicas.
