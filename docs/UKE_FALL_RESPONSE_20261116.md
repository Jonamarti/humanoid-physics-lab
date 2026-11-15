# Corrección del reflejo de caída de Uke

## Resultado y alcance

Se corrige la amplificación de impacto detectada en #107. El caso pasivo de 75 kg pasa de 31,21 kN a 4,93 kN y de 1,86 cm a 0,684 cm de penetración del suelo. En 120 kg pasa de 57,21 kN a 8,05 kN y de 2,84 cm a 0,905 cm. La cabeza deja de tocar el suelo durante las repeticiones pasivas corregidas de esta matriz. No es una validación de ukemi ni una estimación de riesgo de lesión.

El cambio afecta al modo opcional `fall: instinctive`. No cambia geometrías, masas, contactos, rangos anatómicos, solver, paso físico, acciones de Tori ni activación del detector. Las diez grabaciones con `fall: none` quedan idénticas byte a byte entre antes y después. Se conserva la protección frente al empujón recuperable de 25 N·s, sin activación del reflejo.

## Causa observada

En la caída pasiva, el reflejo se activa en 0,984 s, cuando las rodillas ya están cerca de su flexión máxima. El objetivo fijo de -0,95 rad intenta volver a extenderlas. La traza anterior registra unos 142 N·m de extensión de rodilla en el primer contacto, elevación posterior del COM y un impacto de cabeza de 4,29 m/s. El controlador realiza aproximadamente 938 J de trabajo mecánico positivo por articulaciones durante los cuatro segundos; no es trabajo neto ni energía oculta.

Además, la actitud relajada conserva amortiguación pasiva baja mientras el reflejo activa rigidez postural. La primera ablación cambia solo la amortiguación implícita: reduce el pico a 10,70 kN, pero sigue realizando más de 1000 J de trabajo positivo. Por sí sola no resuelve el problema.

## Corrección implementada

1. Al activarse, la amortiguación implícita de las bisagras del actor sube al menos al valor asociado a las ganancias activas existentes. No se añade un segundo término explícito de amortiguación.
2. Se conserva una flexión de cadera/rodilla ya mayor que la postura protectora propuesta y un tronco ya redondeado, siempre dentro de sus límites. Se evita ordenar una extensión brusca desde la postura de colapso.
3. Se distinguen apoyo de miembros (fase `brace`) y contacto central de cabeza, pecho, abdomen o pelvis (fase `settle`). El apoyo de brazos puede frenar una caída; no obliga a abandonar toda la postura antes de que el tronco esté apoyado.
4. Tras contacto central, la rigidez y la contribución del controlador de pie se retiran en 150 ms hasta quedar a cero. Continúan gravedad, contactos, amortiguación implícita y topes articulares. No queda un esfuerzo activo para imponer la postura protectora en el suelo.

Se mantienen la rampa de activación de 120 ms y los topes de par originales. La activación sigue requiriendo pérdida sostenida de apoyo y descenso del COM; salir del polígono por sí solo no la dispara. Las dos raíces siguen libres y el control solo escribe bisagras del actor correspondiente.

## Comparación congelada

Base anterior: commit `b8cf20d0cc49a1f24b4973f0f1885795623b83df` (#107). Las entradas completas están en `tests/fixtures/fallResponseCases.json`. Diez escenarios, cada uno con y sin reflejo, a 500 Hz físicos y de registro, con exactamente los mismos parámetros antes y después. Los resultados históricos de 31,21/57,21 kN se reproducen a esta frecuencia de registro.

| Caso con reflejo | Contacto anterior → corregido | Penetración anterior → corregida |
|---|---:|---:|
| Pasivo 75 kg | 31,21 → 4,93 kN | 1,86 → 0,684 cm |
| Pasivo 55 kg | 10,70 → 3,49 kN | 1,90 → 0,617 cm |
| Pasivo 120 kg | 57,21 → 8,05 kN | 2,84 → 0,905 cm |
| Pasivo 1,95 m | 23,40 → 4,41 kN | 2,50 → 0,720 cm |
| Actor pasivo con prefijo | 31,21 → 4,93 kN | 1,86 → 0,684 cm |
| Empujón frontal | 7,12 → 7,12 kN | 2,871 → 2,871 cm |
| Empujón hacia atrás | 7,66 → 7,76 kN | 2,346 → 2,346 cm |
| Siega exterior | 9,68 → 9,67 kN | 2,721 → 2,721 cm |
| Entrada a piernas | 8,40 → 7,97 kN | 2,919 → 2,919 cm |
| Empujón recuperable | 0,184 → 0,184 kN | 0,0785 → 0,0785 cm |

El pico es la mayor fuerza normal de un contacto individual, medida por la sonda en cada paso. No equivale a la suma de fuerzas de todos los contactos. Las flechas del vídeo usan el estado de contacto grabado; en un impacto brusco una resolución del contacto y la del paso integrado pueden diferir. No se sustituyen los máximos de la sonda por el máximo visible de una flecha.

El trabajo positivo de pares del pasivo nominal baja de 937,88 a 71,81 J. Tras completar `settle`, los pares activos son cero en las regresiones y no siguen bombeando energía. En la última fracción de la caída el cuerpo puede seguir acomodándose por gravedad: menor impacto no implica haber detenido artificialmente todos los segmentos.

## Ensayos rechazados y límites

El registro de hipótesis y las siete etapas están en [experiments/fall-response-20261116](experiments/fall-response-20261116/README.md). No se realizó un barrido de ganancias. Se midieron amortiguación sola, retirada de rigidez con objetivos lentos, un cambio de cuello, la separación apoyo/impacto y la recuperación de la preparación original de los miembros.

La limitación adicional de velocidad de los objetivos retrasaba la preparación de brazos y empeoraba el golpe frontal; se retiró y se conserva la rampa de par anterior. Un objetivo aislado de flexión de cuello empeoró el contacto frontal de cabeza y se rechazó. Estos resultados quedan guardados; no se selecciona solo el caso que mejora.

Persisten penetraciones de brazos de 2,3–2,9 cm en los empujones fuertes y las proyecciones. Los umbrales de artefactos no se relajan: esos casos continúan señalados. El empujón hacia atrás aumenta ligeramente su pico (aproximadamente 1,3 %), aunque permanece por debajo de 10 kN. En el frontal, el cuerpo termina sostenido por miembros, sin contacto central y en fase `brace`; no se fuerza a que el pecho toque el suelo para declarar terminada una animación.

Los objetivos protectores siguen siendo heurísticos y las ganancias de amortiguación no están calibradas con tejidos humanos. No se ha demostrado una respuesta universal para cualquier orientación o velocidad, ni una caída entrenada natural. Tori conserva sus acciones prescritas en las dos referencias: no se cambia esa variable para mejorar el resultado de Uke.

## Evidencia visual y reproducción

Hay diez comparaciones: frontal y lateral de pasivo nominal, pasivo de 120 kg, siega exterior y entrada a piernas, además de vistas oblicuas de las dos proyecciones para resolver la oclusión por Tori. Cada vídeo muestra ANTES a la izquierda y DESPUÉS a la derecha, con la misma cámara y tiempo; primero x1 y luego x0,25. El render no ejecuta `mj_step` y rechaza escenarios, modelos o tiempos distintos. Las hojas incluyen los instantes de los picos de ambas versiones.

El índice local está en `out/fall-response/delivery/index.html`. Los vídeos y las grabaciones no se suben a Git. Sí se guardan métricas compactas, integridad, protocolo y validación.

```powershell
$env:HPL_FALL_PHASE='after'
npm.cmd exec -- vitest run tests/manual/fallResponseBench.test.ts --configLoader runner --maxWorkers=1
npm.cmd exec -- vitest run tests/unit/fallProtectionControl.test.ts --configLoader runner --maxWorkers=1
uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow python scripts/compare_fall_recordings.py out/fall-response/before/passive_neutral_instinctive.json.gz out/fall-response/after/passive_neutral_instinctive.json.gz out/fall-response/delivery/passive_neutral-side.mp4 --view side --sheet out/fall-response/delivery/passive_neutral-side.png
```

Para regenerar ANTES, crear un checkout del commit base y copiar a él únicamente el conductor `tests/manual/fallResponseBench.test.ts` y el fixture `tests/fixtures/fallResponseCases.json`; ejecutar ese conductor desde la raíz del checkout base con `HPL_FALL_PHASE=before`. Sus imports deben resolver el ejecutor y el reflejo del checkout anterior. No basta con poner la etiqueta before al código nuevo.

## Siguiente trabajo de biomecánica básica

La prioridad siguiente es el controlador general solicitado: perturbaciones, cambios de altura, transferencia de peso, zancadas y varios pasos. Se propone la batería de [BASIC_BIOMECHANICS.md](BASIC_BIOMECHANICS.md). Esta entrega no implementa todavía sentadillas, zancadas ni una marcha nueva; conserva los casos recuperables y aporta una transición de caída que evita volver a extender una postura plegada.
