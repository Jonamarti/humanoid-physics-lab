# Velocidad en el primer apoyo cargado y ensayo de amortiguación

**Fecha:** 09-11-2026. Parte de PR #45 / issue #49. Rama de pruebas PR #52.

## Nueva instrumentación conservada

La FSM de paso ahora registra en el **primer muestreo de aterrizaje con
carga vertical >40 N**:

- `firstLandingHorizontalSpeedMps`: norma de la velocidad XY (m/s).
- `firstLandingDownwardSpeedMps`: componente vertical descendente (m/s).
- `firstLandingNormalForceN`: reacción vertical instantánea de pie en ese
  muestreo (N).

El vector de velocidad sale de diferenciar posiciones reales del
`left_sole`/`right_sole` entre pasos físicos MuJoCo de 2 ms y aplicar
filtro temporal (`0,75 × v_anterior + 0,25 × v_instantánea`). La FSM
lee fuerza y velocidad cada diez pasos (~20 ms). Por tanto son medidas
**después o alrededor del primer contacto observado**, NO velocidad
exacta justo antes del choque ni fuerza pico de impacto. Si nunca
se observa un aterrizaje, todos los campos son `null`, no cero.

El experimento de frenado se **descartó y se eliminó del código operativo**;
solo se conservaron la telemetría y las pruebas de regresión.

## A/B de frenado voluntario (código de experimento anterior, no fusionado)

Mismo modelo, mismo desplazamiento diagonal izquierda 0,12/0,08 m;
duración física 5,2 s. La fuerza de freno se generaba con
`tau = Jᵀ (-K_d v)` únicamente durante `LAND`, par limitado y
atenuado al medir contacto real.

| Servo del pie | Freno | Estado | Vel. XY contacto | Desliz. llegada | Deriva apoyo |
|---:|---:|---|---:|---:|---:|
| 0 % | 0 % | ABORT | 0,362 m/s | 12,7 cm | 5,1 cm |
| 0 % | 35 % | **FALL** | 0,396 m/s | 41,2 cm | 29,4 cm |
| 0 % | 70 % | **FALL** | 0,063 m/s | 84,0 cm | 47,8 cm |
| 0 % | 100 % | **FALL** | sin aterrizaje | — | 32,5 cm |
| 20 % | 0 % | ABORT | 0,083 m/s | 11,1 cm | 0,5 cm |
| 20 % | 35 % | ABORT | 0,016 m/s | 29,6 cm | 3,7 cm |
| 20 % | 70 % | ABORT | 0,212 m/s | 10,6 cm | 4,4 cm |
| 20 % | 100 % | ABORT | 0,277 m/s | 13,8 cm | 0,5 cm |

**Fuente de telemetría reproducible:** [CI 42690713011](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42690713011), stdout `STEP-LANDING-BRAKE-SWEEP`.

El frenado aislado no cumple simultáneamente los límites de
deriva de apoyo <2 cm, deslizamiento de llegada <2 cm y avance estable
>1 s. Puede incluso causar caída. **Se ha revertido la actuación extra
en MuJoCo** y quedan únicamente nuevos sensores de primer contacto.

## Siguiente prueba recomendada

Analizar el límite de fricción y el reparto de momentos en cada fase,
en especial la velocidad de deslizamiento tangencial **tras** contacto
y la posición COM/capture point cuando aumenta la carga. Construir
control de distribución de carga de apoyo de forma conservadora,
registrando los pares saturados, sin aumentar arbitrariamente la fricción
del suelo hasta conseguir que pase un test.
