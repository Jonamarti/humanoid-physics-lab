# Primera pieza del control articular — PD clásico (#8)

Código: `src/control/jointPD.ts`; tests: `tests/unit/jointPD.test.ts`.

## Ley de control

Para una articulación de un solo eje:

`τ_deseado = Kp (q_objetivo - q) - Kd * qdot`

`τ_aplicado = max(-τ_max, min(τ_max, τ_deseado))`

- `Kp`: N·m/rad.
- `Kd`: N·m·s/rad.
- `q` y `q_objetivo`: radianes.
- `qdot`: rad/s.
- `τ_max`: N·m, saturación simétrica de un motor articulado interno.

La implementación valida parámetros finitos, ganancias no negativas y límites de par. **No** contiene una política aprendida ni calcula fuerzas globales en la pelvis.

## Unión con MuJoCo

`applyHingePD(model, data, settingsByJointIndex)` consulta:
- `model.jnt_type` para permitir **solo bisagras** (MuJoCo tipo 3).
- `model.jnt_qposadr` y `model.jnt_dofadr` para mapear el estado.
- `data.qpos`, `data.qvel` para error de posición y amortiguación.
- `data.qfrc_applied[DoF]` para escribir exclusivamente pares de bisagras.

Queda prohibido por API intentar conducir el `freejoint` de pelvis (tipo 0). El test de integración aplica par a la bisagra lumbar del humanoide sintético y compara movimiento con una ejecución pasiva de MuJoCo real.

## Estado y límites

**Este paso es el controlador articular de bajo nivel, no un controlador de equilibrio.** No mueve el humanoide automáticamente en el visor: primero hay que desarrollar el bucle sensorial, perfiles de grupos musculares, metas coordinadas y criterio de contacto con el suelo.

No usar `targetRad=0` para todos los joints como garantía de bipedestación: con root libre, un controlador que mantenga ángulos relativos puede igualmente dejar caer al personaje. Las ganancias deben derivarse de masa, longitud y pruebas, con límites de torque razonables.

**Próximos pasos (#8–#9):** integrar la actualización PD en cada timestep físico; estimar CoM, velocidad, CoP y orientaciones; controlar conjuntamente tobillos/caderas/tronco; ejecutar y evaluar `stand_10s` sin sujeciones artificiales.
