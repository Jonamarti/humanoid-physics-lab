# Ensayo clásico: cilindro y obstáculo de altura variable (pre-seoi)

**Estado:** banco de pruebas educativo para un único cuerpo rígido en MuJoCo WASM, sin control muscular ni movimiento humano. La finalidad es estudiar torques, vuelco, fricción y colisiones antes de añadir un humanoide activo.

## Origen del protocolo

Este benchmark reconstruye **independientemente** los parámetros de unas pruebas anteriores conservadas en AnimSpec, en una carpeta privada de trabajo físico aparcado. No se ha copiado el código fuente ni ningún modelo, render o archivo privado. La implementación TypeScript se publica bajo la licencia del laboratorio.

El objetivo histórico era comprender por qué un cuerpo apoyado contra una barrera baja o contra la espalda/cadera de Tori puede pivotar, deslizar o volcar según la altura del obstáculo y la dirección del tirón. **Un cilindro no equivale a una persona:** no puede dar un paso, doblarse, agarrarse ni reaccionar activamente.

## Modelo y fuerza

- Cilindro homogéneo libre: masa **75 kg**, altura **1,75 m**, radio **0,18 m**.
- Gravedad **9,81 m/s²** y rozamiento deslizante de referencia **0,6**, geometría MuJoCo nativa.
- Obstáculo fijo, profundidad **0,12 m**, separado **0,02 m** del borde del cilindro.
- Alturas seleccionables: **0,10, 0,50, 0,80, 0,90 y 1,20 m**.
- Fuerza hacia delante **+X**, con componente vertical `Fz = ratio * Fx`, aplicada a una altura local de **1,40 m** (en la orientación actual del cilindro).
- Comienza a `t = 0,30 s` y dura por defecto **0,15 s**; por defecto `Fx=600 N` y `ratio=0,3`.
- Timestep físico de **0,001 s**, independiente del ritmo de render.
- Al tratarse de un cuerpo libre, la fuerza aplicada en punto excéntrico añade un par `τ = (p - CM) × F`, a través de `data.xfrc_applied` de MuJoCo. Cada paso limpia el wrench anterior antes de aplicar el correspondiente.
- `Fx` se refiere a la componente **horizontal**; el módulo total de fuerza es `Fx * sqrt(1+ratio²)`. El impulso horizontal de fuerza constante es `Jx = Fx * duración`.

## Comparación con la predicción energética

Se muestra una comparación orientativa de un **pivote ideal, fijo** en el borde superior del obstáculo. A partir de la fuerza integrada en el tiempo, se calcula el momento angular impulsivo `L`, la inercia `I_pivote = I_CM + m * |r_CM-pivote|²` y la energía cinética rotacional `L²/(2 I_pivote)`. La barrera gravitatoria se estima a partir de la elevación del CM hasta la vertical del pivote.

**La predicción es aproximada**: durante la simulación real el punto de contacto puede deslizarse, cambiar, perderse o experimentar choques. El cuerpo puede girar alrededor del pie, rebotar, pasar por encima o quedar apoyado. No interpretar discrepancias entre energía ideal y MuJoCo como un error por sí solas.

Se considera «vuelco» cuando la orientación del eje del cilindro supera **90° hacia delante**. La UI muestra el máximo ángulo **muestreado por fotogramas de render**, por lo que no garantiza capturar cada máximo intermedio; un futuro runner de simulación deberá registrar ángulo **en cada timestep**. Tampoco supone haber validado ningún seoi-nage humano.

## Uso en web

1. Abrir el laboratorio en GitHub Pages.
2. En **Escenario físico**, seleccionar **Cilindro + obstáculo**.
3. Ajustar **fuerza horizontal**, **duración**, **altura del obstáculo** y **componente vertical**.
4. Pulsar **Cargar simulación** para compilar el escenario con esos parámetros; **Iniciar** aplica el tirón a los 0,30 s.
5. Girar y acercar la cámara con el ratón o desde el móvil; **Pausar** y **Reiniciar** para repetir el ensayo.

Cambiar parámetros exige pulsar **Cargar simulación** de nuevo: la ejecución usa una copia fija del ensayo para evitar cambios a mitad del impulso.

## Pruebas autónomas

`tests/unit/cylinderBenchmark.test.ts` verifica valores y dimensiones, límites de parámetros, ventana de fuerza, signos de giro, predicción de pivote y carga en MuJoCo real de las cinco alturas históricas. Compara también una trayectoria con tirón y otra sin fuerza en WASM (sin mocks).

**Próximos pasos:** integrar métricas por cada timestep y un barrido automático de `Fx=[300,600,1000,1500]` N, `ratio=[0.3,1]`, alturas históricas; generar informe CSV con contacto, deslizamiento, ángulo máximo, energía y diferencias. Después incorporar perfil `kuzushi → tsukuri → kake` en lugar del único pulso; no necesitará que el usuario abra Blender.

**Trabajo en paralelo:** #8–#9 sigue siendo el objetivo principal de control activo del humanoide. Este experimento cumple un peldaño de #10 y #16 sin sustituirlos.

## Escenarios de datos y umbrales medidos (issue #74, PR 4)

Los cinco escenarios del cilindro viven en `scenarios/*.json` (formato `hpl.scenario.v1`, ver `SCENARIO_V1.md`) y los recorre
`tests/unit/scenarios.test.ts`: valida, ejecuta con `runScenario` (`RECORDING_V1.md`) y exige `summary.expect === "pass"`.
**Añadir una prueba es añadir un JSON** con su `expect`. Protocolo: el de este documento (cilindro de 75 kg, 1,75 m, radio 0,18 m;
obstáculo de 0,12 m de profundidad a 0,02 m; rozamiento 0,6; fuerza **constante** durante 0,15 s desde `t = 0,30 s`, a 1,40 m de
altura en la cara delantera, con componente vertical `Fz = 0,3·Fx`), con las diferencias de la sección «Diferencias» más abajo.
Los modelos son nuevos: opciones de `worldOptionTag()` (cono elíptico), `ccd_tolerance` 1e-14, paso de 2 ms (la web usa 1 ms y
el cono por defecto: **los números de esta sección no valen para el ensayo de la web**).

### Regla contra umbrales frágiles

Un caso «fall» debe estar al menos a 1,3 veces la fuerza de umbral y uno «standing» por debajo de 0,6 veces. La prueba lo
comprueba sin conocer el umbral: el caso «fall» con todas sus fuerzas ×(1/1,3) sigue cayendo y el «standing» con sus fuerzas
×(1/0,6) sigue de pie. Los umbrales de abajo se midieron aparte, barriendo `Fx` en pasos de 5 N (`ratio` 0,3) y de 25 N (`ratio` 1) con
el mismo ejecutor; son una **cota superior con 5 N (o 25 N) de resolución**: la fuerza más baja que da «fall» y, entre paréntesis, la
mayor que no lo da.

| Obstáculo | Umbral `ratio` 0,3 | Umbral `ratio` 1 | Observación |
|---|---|---|---|
| ninguno (suelo) | 275 N (270 N de pie) | 300 N | |
| 0,10 m | 285 N (280 N de pie) | 300 N | |
| 0,50 m | 625 N (620 N «tilted»); desde 695 N todas caen | 600 N; desde 650 N todas caen | banda 625–690 N mezcla «fall» y «tilted» |
| 0,80 m | 1580 N (1575 N «tilted») | 1225 N | |
| 0,90 m | 2420 N (2415 N «tilted») | 1725 N | |
| 1,20 m | no cae hasta 3000 N (solo «standing»/«tilted» y 7° a 1500 N con `ratio` 1) | no cae | centro de masas a 0,875 m: el obstáculo queda por encima |

Hay un resultado distinto de «fall» en torno a 2815 N en las filas sin obstáculo y de 0,10 m (no investigado; está fuera del rango
de los escenarios).

### Escenarios y comparación con el protocolo de la issue #30

| Escenario | Fuerza horizontal | Issue #30 | Umbral | F / umbral | Esperado | Obtenido |
|---|---|---|---|---|---|---|
| `cylinder_obstacle_low` (0,10 m) | 400 N | 300 N | ≤ 285 N | ≥ 1,40 | fall, 0° ± 30° | fall, 0,0° |
| `cylinder_obstacle_high` (0,80 m) | 2200 N | 1500 N | ≤ 1580 N | ≥ 1,39 | fall, 0° ± 30° | fall, 0,0° |
| `cylinder_obstacle_above_com` (1,20 m) | 1500 N | 1500 N | > 3000 N | < 0,5 | standing | standing (máx. 1,2°) |
| `cylinder_no_obstacle` | 400 N | 350 N | ≤ 275 N | ≥ 1,45 | fall, 0° ± 30° | fall, 0,0° |
| `cylinder_three_phase` | 650 N en la fase `phase_c` | 650 N | escala ≤ 0,59 | ≥ 1,69 | fall, 0° ± 30° | fall, 0,0° |

- **Dos casos de la issue no se reproducen como están y se han ajustado, no forzado.** Con 1500 N a 0,80 m y `ratio` 0,3 el cilindro
  se inclina hasta 104° y vuelve a quedar en «tilted»: no cae (umbral 1580 N). Con `ratio` 1 sí cae (1500 N, umbral 1225 N), pero
  1,22 veces el umbral no cumple la regla de 1,3. Se mantiene `ratio` 0,3 y se sube la fuerza a 2200 N. Los 300 N a 0,10 m caen
  (umbral 285 N) pero a 1,05 veces el umbral; los 350 N sin obstáculo caen (umbral 275 N), a 1,27 veces: ambos se suben a 400 N.
- La issue no dice qué resultado espera para «sin obstáculo, 350 N»; con este contacto cae hacia +X.
- El caso de 1,20 m reproduce la issue: de pie con 1500 N (máx. 1,2° de inclinación), y sigue de pie con 2500 N (la regla ×1/0,6).
- `cylinder_three_phase` es un perfil genérico de tres fuerzas consecutivas (efectores `phase_a`, `phase_b`, `phase_c`) con un
  obstáculo de 0,30 m: `phase_a` 100 N durante 0,5 s, `phase_b` 200 N durante 0,3 s y `phase_c` 650 N durante 0,15 s (cada una con
  `Fz = 0,3·Fx`). El perfil lo he definido yo, no lo he sacado del protocolo privado. Medido: multiplicando las tres fuerzas por
  0,59 ya cae (umbral); solo `phase_c` necesita escala 0,71 (461 N), y `phase_a` + `phase_b` sin `phase_c` no caen hasta
  escala 1,2: las fases previas bajan el umbral de `phase_c` un 17 %.
- En el caso de 0,80 m el cilindro pasa por encima del obstáculo (inclinación máxima 180°) y acaba tumbado. Por eso la dirección
  de caída se mide cuando el eje supera 60° y no al final (`RECORDING_V1.md`).

### Energía simulada y analítica en el umbral

`idealPivotPrediction` supone un impulso instantáneo sobre un pivote fijo. Para cada obstáculo, en la fuerza umbral simulada (`F_sim`,
la más baja que cae) se compara la energía cinética total del cilindro justo después del pulso (`t = 0,456 s`, a partir de `qvel`)
con la analítica a esa fuerza y con la barrera gravitatoria. El umbral del modelo ideal (`F_ideal`) se obtiene por bisección.

| Obstáculo | `F_sim` | `F_ideal` | `F_sim / F_ideal` | Energía analítica / barrera | Energía simulada / analítica |
|---|---|---|---|---|---|
| ninguno | 275 N | 244,6 N | 1,12 | 1,26 | 0,44 |
| 0,10 m | 285 N | 256,9 N | 1,11 | 1,23 | 0,45 |
| 0,50 m | 625 N | 364,2 N | 1,72 | 2,94 | 0,39 |
| 0,50 m (estable) | 695 N | 364,2 N | 1,91 | 3,64 | 0,43 |
| 0,80 m | 1580 N | 756,2 N | 2,09 | 4,37 | 0,37 |
| 0,90 m | 2420 N | 1148,5 N | 2,11 | 4,44 | 0,37 |

«Energía analítica / barrera» es lo que la issue llama relación entre energía simulada y analítica en el umbral (esperada entre 1 y 2):
a fuerzas bajas (suelo, 0,10 m) sale 1,2 y está dentro; con obstáculos de 0,5 m o más sale de 2,9 a 4,4, **fuera de lo esperado**: el
modelo ideal subestima la fuerza necesaria. Una causa probable (no comprobada) es que el pulso de 0,15 s no es instantáneo y el
cilindro se mueve y desliza durante él: la energía cinética simulada justo después del pulso es solo el 37–45 % de la analítica.
Esto se informa, no se afirma: la prueba solo imprime la predicción y el resultado (`informa (sin afirmar)...`).

### Diferencias con el ensayo de la web y con la issue

- Forma de onda `constant` (como el ensayo de la web), no `half_sine` del ejemplo de la issue: el impulso es `F·T` y
  `idealPivotPrediction` es comparable.
- Cuerpo `cylinder`, punto de aplicación `[0,18, 0, 0,525]` m (cara delantera, a 1,40 m de altura), como `localGripPoint()`.
- Obstáculo de 0,8 m de ancho en Y, 0,12 m de profundidad y separado 0,02 m (como `cylinderMJCF`).
- El tiempo del escenario empieza tras 0,5 s de asentamiento; el ensayo de la web no asienta.
- Paso de 2 ms y cono elíptico (la web, 1 ms y cono piramidal por defecto).
