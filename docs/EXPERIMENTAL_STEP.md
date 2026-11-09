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
