# Hito 1: equilibrio omnidireccional ante empujones (#38)

**Alcance de esta PR:** ampliar el protocolo de empujón desde cuatro a ocho
direcciones, sin fingir que el controlador ya sabe dar pasos.

## Convenciones y controles

- X positivo: delante; Y positivo: izquierda; Z: vertical.
- Direcciones cardinales y diagonales a 45 grados sobre el plano XY.
- Los vectores de dirección tienen norma 1, de modo que un pulso de
  `120 N` ejerce **120 N de fuerza resultante** tanto en cardinal como
  diagonal (componentes ~84,85 N cada una en la diagonal).
- El perfil es media onda sinusoidal de 0,12 s y comienza a 1,0 s.
  El impulso total teórico es `2 × F_peak × duration / pi`.
- **No mezclar fuerza pico, fuerza media e impulso.**
- El resultado de la simulación es MuJoCo, no interpolación prefabricada
  ni desplazamiento cinemático del humanoide.

## Ensayo exploratorio

`tests/unit/standingOmnidirectionalSweep.test.ts` realiza dieciséis
escenarios (8 direcciones × 120/240 N), cada uno de 6 s, comenzando desde
la misma postura nominal controlada. Registra por ensayo:

- Desplazamiento final del centro de masas y velocidad horizontal.
- Altura mínima de pelvis e inclinación máxima de raíz.
- Deriva final de pies y máximo número global de contactos.
- Resultado `recovered_no_step`, `unstable` o `fall`.

La clasificación usa umbrales **de este banco de ingeniería**, no umbrales
de seguridad humana. No se declara paso de recuperación: el modelo actual
no camina. Tampoco se considera fallo de test una caída que realmente
resuelve MuJoCo. Los sensores de carga por pie añadidos en la PR #35
se integrarán al protocolo al entrar en main; el contador de contactos
global NO es una fuerza de apoyo ni identifica el pie cargado.

## Próximas fases

1. Usar carga por pie, capture point 2D y planificador de la PR #35.
2. Añadir transferencia real de masa al pie de apoyo, swing, landing y
   rearme del control de equilibrio, manteniendo la raíz libre (#13).
3. Medir recuperación con pasos en las 8 direcciones, más perturbaciones
   suficientemente intensas que causen caída (#14).
4. Ajustar parámetros con datos científicos, manteniendo un conjunto de
   validación independiente (#36).
5. Tras conseguir equilibrio con pasos, implementar marcha recta con
   velocidades progresivas (#39), y después perturbaciones durante la
   marcha, incluidas laterales y diagonales (#40).

No usar el vídeo de robots comerciales como calibración cuantitativa si no
se conocen masa, punto de fuerza, perfil de empujón y controlador del robot.

## Primera medición del benchmark (CI, 09-11-2026)

Run con tests físicos MuJoCo WASM (8 × 2 intensidades):
https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42633730634

| Dirección | 120 N pico (9,17 N·s) | 240 N pico (18,33 N·s) |
| --- | --- | --- |
| Adelante | Recupera sin paso | Recupera sin paso |
| Atrás | Recupera sin paso | **Cae** |
| Izquierda | Recupera sin paso | Recupera sin paso |
| Derecha | Recupera sin paso | Recupera sin paso |
| Adelante-izquierda | Recupera sin paso | Recupera sin paso |
| Adelante-derecha | Recupera sin paso | Recupera sin paso |
| Atrás-izquierda | Recupera sin paso | **Cae** |
| Atrás-derecha | Recupera sin paso | **Cae** |

**13 recuperaciones / 3 caídas** conforme a los umbrales de observación
mencionados, **sin ningún paso intencionado**. En las tres caídas de
240 N posteriores/diagonales, la pelvis bajó hasta aproximadamente
0,124–0,127 m y la inclinación máxima alcanzó ~88–89 grados.
No existe aún validación de biomecánica real ni se ha demostrado que
esos valores de fuerza produzcan el mismo resultado en personas.

Este resultado fija una **línea base reproducible**: el controlador de
pasos deberá intentar recuperar algunos de los casos que ahora caen,
sin ocultar fallos y sin modificar artificialmente las fuerzas aplicadas.
