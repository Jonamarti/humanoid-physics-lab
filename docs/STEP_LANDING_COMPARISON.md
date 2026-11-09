# Ensayo A/B: qué hacer con las articulaciones después de apoyar el pie

**Fecha:** 9-11-2026. Código base: PR #45. Código experimental: PR #50.

El controlador de los pasos voluntarios se mantiene en modo *opt-in* y
sin fuerza aplicada a la raíz libre. El resultado de un CI verde solo valida
la reproducibilidad de las muestras; **no equivale a una marcha conseguida**.

## Datos experimentales

Para el mismo humanoide de 75 kg, objetivo de la planta izquierda
`(+0,12 m, +0,08 m)`, inicio en 0,8 s, ejecución de 5,2 s:

| Política después del contacto | Escala servo | Estado | Resbalamiento llegada | Deriva pie de apoyo | Error destino |
| --- | ---: | --- | ---: | ---: | ---: |
| Mantener referencia anterior | 0 % | ABORT | 12,67 cm | 5,09 cm | 11,76 cm |
| Mantener referencia anterior | 20 % | ABORT | 11,05 cm | 0,50 cm | 11,98 cm |
| Volver a PD neutro | 0 % | ABORT | 9,98 cm | 5,09 cm | 9,84 cm |
| Volver a PD neutro | 20 % | ABORT | 9,89 cm | 4,21 cm | 14,22 cm |
| Mantener pose articular medida al tocar suelo | 0 % | ABORT | 11,91 cm | 5,10 cm | 11,66 cm |
| Mantener pose articular medida al tocar suelo | 20 % | **FALL** | **47,08 cm** | **31,04 cm** | **49,98 cm** |

CI ejecutado:
https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42689963405

**Conclusión:** aunque la orden neutra reduce algo el deslizamiento del
pie que llega, no consigue los límites de aceptación (2 cm por pie,
error destino menor de 5 cm, un segundo de apoyo estable). Congelar
articulaciones al tocar el suelo puede empeorar radicalmente el control.
Ninguna estrategia de este ensayo se promueve a controlador predeterminado.

## Siguiente hipótesis comprobable

La causa puede estar **antes** del primer contacto, por una velocidad
horizontal/vertical excesiva del pie de vuelo, la utilización del cono
de fricción o un exceso de impulso tangencial al apoyar. Registrar
velocidad de la planta inmediatamente antes del primer contacto cargado,
pico de carga y velocidad de deslizamiento, y comparar desaceleración
gradual de la pierna libre durante LAND con control nulo.

Prohibido fingir que el pie no resbala mediante teletransporte, anclaje
geométrico o restricciones de mundo fuera del MJCF. Mantener pruebas
del equilibrio nominal y la clasificación honesta ABORT/FALL.
