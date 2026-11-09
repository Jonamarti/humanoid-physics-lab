# Modelo humanoide MJCF v1 — base física no controlada

Archivo fuente público: [`public/models/humanoid_v1.xml`](../public/models/humanoid_v1.xml).

Este humanoide sintético de **75 kg y aproximadamente 1,76 m** es un punto de partida para validar colisiones, grados de libertad, fuerzas y masa. **Todavía no dispone de control postural, musculatura activa, recuperación ni ukemi.** Si se perturba, debe comportarse como ragdoll y puede caerse; eso no significa que haya un fallo en el controlador (que aún no existe).

## Segmentos y masas

| Segmento | Masa (kg) | Número |
|---|---:|---:|
| Pelvis | 14.0 | 1 |
| Abdomen | 9.0 | 1 |
| Tórax | 16.0 | 1 |
| Cabeza | 5.0 | 1 |
| Muslo | 7.5 | 2 |
| Pierna inferior | 3.5 | 2 |
| Pie | 1.1 | 2 |
| Brazo superior | 2.1 | 2 |
| Antebrazo | 1.3 | 2 |
| **Total** | **75.0** | **14 segmentos** |

Masa y distribución **aproximadas**, no medidas de un ser humano concreto. La inercia de cada cuerpo se deriva de su geometría en MuJoCo.

## Articulaciones

- Pelvis con **freejoint 6 DoF** sin fijación artificial.
- Columna lumbar: flexión y inclinación lateral.
- Tórax: flexión, inclinación lateral y giro.
- Cuello: flexión y giro lateral.
- Cada cadera: 3 DoF (pitch, roll, yaw).
- Cada rodilla: bisagra de flexión.
- Cada tobillo: 2 DoF (pitch, roll).
- Cada hombro: 3 DoF.
- Cada codo: bisagra.
- Extremos con geometrías de caja/cápsula/ellipsoide, sin mallas externas.
- `sternum_impact`, `pelvis_origin`, `left_sole` y `right_sole` son puntos de referencia para sensores/impactos futuros.

El modelo se creó explícitamente sin `<actuator>`: ningún motor ancla el cuerpo en el espacio ni impone que permanezca de pie.

## Validación automatizada

`tests/unit/humanoidModel.test.ts` usa el paquete MuJoCo WASM **real** y verifica la carga, el total de masa, la existencia de articulaciones, ausencia de actuadores, el freejoint, contacto de suelo, movimiento tras pequeña velocidad lateral y reinicio.

No se considera validado el equilibrio de **10 segundos** hasta completar las issues #8–#9. Tampoco está conectado al visor Three.js; #7 deberá completarse cuando se visualicen sus cuerpos en movimiento mediante transformaciones reales de MuJoCo.

## Parámetros iniciales

- MuJoCo con Z vertical y gravedad `0 0 -9.81`.
- Timestep físico fijo de `0.002 s`.
- Centro de pelvis inicial `z=0.99 m`, tobillos sobre `z=0.07 m`, planta al suelo.
- Geometría y fricción aproximadas, ajustables tras los primeros tests y análisis del contacto.

**No incorporar FBX privados ni copiar modelos licenciados de terceros.**
