# Estado del laboratorio de derribos a 12-11-2026

Resumen de lo hecho en las fases 1 a 3 del [plan de derribos](PLAN_DERRIBOS.md), lo que quedó a medias al parar el
trabajo el 12-11-2026 y el diagnóstico de por qué las caídas no pasan el detector de artefactos.

## 1. Cómo verlo en la web

Web desplegada: <https://jonamarti.github.io/humanoid-physics-lab/> (despliegue del commit `4c0487c`, comprobado el
12-11-2026). En **Grabaciones de ejemplo** hay cinco grabaciones; también se abren por enlace:

| Enlace | Qué muestra |
|---|---|
| `?rec=cylinder_obstacle_low` | Cilindro que vuelca sobre un obstáculo bajo |
| `?rec=cylinder_obstacle_above_com` | Cilindro que no vuelca (obstáculo sobre el centro de masas) |
| `?rec=uke_stand_75_175_neutral` | Uke de pie 10 s |
| `?rec=uke_relaxed_75_175_collapse` | Uke `relaxed` desplomándose |
| `?rec=uke_tense_75_175_push_sternum` | Uke `tense` empujado en el esternón; cae de frente |

Límites de lo que se ve hoy:

- La web reproduce el cuerpo y las flechas de fuerza. Centro de masas, punto de captura y polígono de apoyo solo se
  dibujan en el vídeo de `scripts/render_recording.py`, no en el visor web.
- La web no muestra los artefactos detectados; están en `summary.artifacts` de cada grabación y los imprime `npm run sim`.
- No hay todavía ninguna grabación con golpes, reflejos de caída, agarres ni obstáculos: ese trabajo no está fusionado.
- Probado en navegador de escritorio con pantalla emulada de 375 px. Sin probar en un móvil real ni en Safari.

## 2. Fusionado en `main`

| PR | Fase | Qué aporta |
|---|---|---|
| #83–#88 | 1 | Esquema de escenario, ejecutor sin navegador, comando `sim`, vídeo sin navegador, reproducción en la web |
| #87, #89 | 2 (PR 1 y 2) | Uke paramétrico por masa y altura; escalado del control y actitud `tense` |
| #90 | 2 (PR 3) | Actitud `relaxed`, efectores por sitio (`site`), pruebas de energía, péndulo, simetría y robustez |
| #91 | — | Galería de grabaciones de ejemplo y enlaces `?rec=` |
| #92 | 3 (PR 1) | Mano virtual (`src/effectors/hand.ts`): `rope`, `clamp`, `push`, con topes de fuerza, potencia y alcance. Módulo suelto, sin conectar al escenario |
| #93 | 2 (PR 4) | Veredicto del humanoide (`standing`, `stepped`, `fall`, `unresolved`) y métricas de equilibrio por muestra |
| #94 | 2 (PR 5) | Detector de artefactos, `diagnostics` por paso físico y `sim --strict` (código de salida 4) |

Medidas que conviene recordar (Uke de 75 kg y 1,75 m salvo que se diga otra cosa):

- `relaxed` cayendo: la energía mecánica nunca sube (747 J → 117 J).
- Péndulo de 5° a 45°: 0,9920 s simulado frente a 0,9924 s teórico.
- Empujón sagital de 150 N·s en el esternón: desvío lateral 0,000 cm y giro de pelvis 0,000°.
- Umbral de caída hacia atrás de `tense` sin dar pasos: entre 12 y 13 N·s.
- Uke de 150 kg y 2,00 m de pie: el punto de captura queda a 1,5 mm del borde del apoyo. Se sostiene, con muy poco margen.
- Mano virtual sobre un cuerpo de 10 kg: fuerza máxima 500,0 N, potencia máxima 600 W (+0,5 % con velocidad media).
  El amortiguador explícito diverge con masas efectivas por debajo de 0,2 kg.

## 3. A medias, sin fusionar

Ramas subidas con el trabajo tal como quedó al interrumpirlo. **Ninguna está terminada ni probada entera**; no tienen PR.

| Rama | Tarea | Estado |
|---|---|---|
| `feat/f3-grip-catalog` | Fase 3, PR 2: catálogo de agarres | Código y pruebas confirmados y mezclados con `main`; faltaba la última pasada de la suite y abrir la PR |
| `feat/f3-limited-obstacle` | Fase 3, PR 3: obstáculo con fuerza limitada | `obstacle.ts` y sus pruebas escritos; el agente estaba rehaciendo dos pruebas. Sin documento |
| `feat/f2-strikes` | Fase 2, PR 7: golpes (`impulse` y `ram`) | Efectores escritos y a medio conectar al ejecutor. Sin escenarios, sin pruebas propias, sin vídeo |
| `feat/f2-fall-reflexes` | Fase 2, PR 6: reflejos de caída | Solo el parámetro `fall` en esquema y validador. La capa de reflejos no está escrita |

Sin empezar: fase 3 PR 4 (presupuesto y pista de Tori) y la conexión de mano, agarres y obstáculo al formato de
escenario. Esta última no figura en la issue #76: se añadió al hacer las PR 1 a 3 como módulos sueltos para poder
avanzar en paralelo con la fase 2. Hasta que exista, no hay derribos que simular ni que ver.

## 4. Por qué las caídas no pasan el detector

El detector marca todas las caídas existentes al golpear el suelo; los casos que quedan de pie salen limpios.

| Caso | Penetración en el suelo (límite 1 cm) | Fuerza de contacto (límite 10 kN) |
|---|---|---|
| Cilindros que caen (4) | 2,6–3,6 cm | 19–23 kN |
| Uke `relaxed` desplomándose | 1,2 cm | 7,8 kN |
| Uke `tense` empujado | 1,95 cm | 15,5 kN |

### Diagnóstico

No es un fallo del detector ni un error numérico. Los dos límites son incompatibles entre sí para un cuerpo rígido
que llega al suelo a la velocidad de una caída.

El contacto es un muelle con amortiguamiento crítico de constante de tiempo `τ = 0,015 s` (`solref="0.015 1"`). Para
un impacto a velocidad `v` con masa efectiva `m`:

- penetración máxima ≈ `v · τ / e`
- fuerza máxima ≈ `2 · m · v / τ`

En el empujón al esternón el pecho llega a 3,69 m/s. La primera fórmula da 3,69 × 0,015 / 2,718 = 2,0 cm; lo medido
es 1,95 cm. La segunda, con los 15,5 kN medidos, corresponde a una masa efectiva de unos 31 kg, razonable para el
tronco de un cuerpo de 75 kg que cae rígido.

Consecuencia: endurecer el suelo reduce la penetración y sube la fuerza en la misma proporción.

| `τ` | Penetración a 3,7 m/s | Fuerza con 31 kg |
|---|---|---|
| 0,015 s (actual) | 2,0 cm | 15 kN |
| 0,0073 s | 1,0 cm | 31 kN |
| 0,023 s | 3,1 cm | 10 kN |

Para cumplir a la vez 1 cm y 10 kN a 3,7 m/s haría falta una masa efectiva de unos 10 kg. Ningún ajuste del suelo lo
consigue. Esto es una estimación con las dos fórmulas anteriores, no un barrido medido de `τ`.

### Qué significa

- Un humano tampoco cae así. Los 15 kN son el resultado físico de un cuerpo que cae como una tabla, sin flexionar
  ni sacar los brazos. Lo que baja el pico es llegar más despacio y con menos masa de golpe: flexión de rodillas y
  cadera, brazos que frenan, espalda redondeada. Eso son los reflejos de caída (fase 2, PR 6), que no están hechos.
- El límite de 1 cm de penetración se pensó para detectar cuerpos que atraviesan el suelo. Un cuerpo real se deforma
  varios centímetros al golpear (tejido blando, ropa, tatami); el modelo no tiene tejido blando, así que esa
  deformación aparece como penetración del contacto.
- El límite de 10 kN se pensó para detectar explosiones del solver, no impactos.

### Opciones (decisión pendiente)

1. **Separar impacto de reposo.** Penetración: exigir 1 cm solo cuando el cuerpo está en reposo o apoyado, y permitir
   en el impacto hasta `v · τ / e` más un margen. Fuerza: comparar con `2 · m · v / τ` en vez de con un valor fijo. El
   detector seguiría cazando explosiones y cuerpos hundidos, y dejaría de marcar impactos coherentes con el modelo.
2. **Dejar los límites y esperar a los reflejos.** Medir cuánto bajan los picos con `instinctive` y `ukemi` antes de
   tocar nada. Es lo que se había encargado; quedó interrumpido sin resultados.
3. **Añadir una capa blanda** (tatami) como parámetro del suelo. Cambia la física de todos los escenarios y obliga a
   repetir la calibración de contacto de la fase 1.

Las opciones 1 y 2 no se excluyen. La 1 es un cambio de criterio, así que según la regla 9 del plan va en una PR
propia, sin mezclar con cambios de física.

## 5. Otras cuestiones abiertas

- `sim --strict` devuelve 4, no 3 como pedía la issue #75: el 3 ya significaba «excepción del ejecutor».
- `stepped` solo está probado con trayectorias sintéticas; ningún Uke actual da pasos.
- La velocidad de impacto de «espalda» se publica como la del pecho: el modelo no tiene un cuerpo espalda.
- El detector de temblor usa una amplitud mínima de 0,005 rad elegida por visibilidad. Con 0,001 rad las dos caídas
  superarían el límite de 10 cambios por segundo: hay una vibración articular de menos de 0,06° sin calibrar con vídeo.
- Los detectores de tope de efector y de fuerza en la raíz solo se han visto dispararse con datos sintéticos.
- No existe el paso de `tense` a `relaxed` en mitad de un clip (pérdida de consciencia tras un golpe). No está en
  ninguna issue.
- La altura del tobillo es fija (0,07 m × altura / 1,76). Con personajes calzados la pelvis simulada queda varios
  centímetros más baja que la del rig; habrá que compensarlo en la conversión a clip (#80) o admitirla como parámetro.
- Los valores por defecto de la mano virtual (500 N, 600 W, 0,75 m, 0,4 s) son puntos de partida sin calibrar.
- Todo el vídeo revisado hasta ahora es vista lateral. Nadie ha revisado vista frontal ni cámara lenta fotograma a
  fotograma.
