# ProtoMotions: pasos alternos y perturbaciones de pecho — 16/11/2026

Se obtienen seis pasos alternos a **0,51 m/s**, postura con brazos bajos y dos segundos finales estables. El resultado sigue siendo experimental: el pie cargado se desplaza **2,38 cm**, por encima del límite conservado de 2 cm. No se declara una marcha ni una recuperación físicamente certificadas.

Este trabajo está apilado sobre la PR #110. No modifica el controlador de Uke del núcleo, Tori, Unity ni el rig consumidor. Se usa el robot público Anatomic SOMA de **42,47 kg**; todavía no es el Uke de 75 kg.

## Qué se cambia

- El adaptador llama explícitamente a **agent.eval()**, como el evaluador oficial: inferencia sin dropout ni actualización de normalizadores. La prueba carga el checkpoint real y verifica modo y buffers.
- Las referencias son propias, generadas mediante IK. La trayectoria del cuerpo combina avance continuo con derivadas continuas y balanceo lateral de un péndulo invertido dependiente de la cadencia.
- El pie inicia antes su vuelo y completa el avance antes de descender. La elevación objetivo de 18 cm compensa el seguimiento incompleto de la red; **la altura real se mide**, no se asume igual al objetivo.
- Se exige error de IK <2 mm antes de ejecutar una referencia de marcha. Los cuatro clips finales quedan por debajo de 0,03 mm. Algunas referencias exploratorias anteriores eran inalcanzables y se excluyen como candidatas.
- Un planificador de recuperación experimental calcula un punto de captura desde posición/velocidad observadas y construye objetivos de paso. No recibe el instante, fuerza ni dirección del golpe. **No se habilita por defecto**: no mejora todos los casos.
- Se guarda y restaura tanto el estado completo de MuJoCo como el estado sensorial del entorno. Las 24 parejas de impactos tienen ejecución idéntica antes del golpe.

No se entrenan pesos nuevos. La red sigue siendo un **motion tracker con referencias futuras**, no una política autónoma de navegación o locomoción. Sólo los motores limitados integran el humanoide; no se escriben posiciones o velocidades de raíz/pies durante la ejecución medida. Las escrituras de estado se limitan al inicio de cada ensayo.

## Velocidades físicas

Física a 960 Hz, control a 30 Hz y grabación a 120 Hz. Cambiar la cadencia modifica posiciones futuras, velocidades lineales, angulares y articulares de referencia; el vídeo normal permanece a 30 fps y tiempo real. Estas velocidades corresponden a marchas lentas; como referencia experimental humana, se han medido 1,4 m/s de velocidad natural en [Escalona et al., 2021](https://pubmed.ncbi.nlm.nih.gov/33503586/).

| Velocidad real | Cadencia objetivo | Pasos con planta completa >=3 cm | Alternancia | Estable 2 s al final | Trabajo motor positivo |
|---|---:|---:|---|---|---:|
| 0,107 m/s | 25 pasos/min | 7, incluido apoyo extra | No | Sí | 497,9 J |
| 0,216 m/s | 50 pasos/min | 6 | Sí | Sí | 418,3 J |
| 0,322 m/s | 75 pasos/min | 5 | Sí entre los detectados | Sí | 269,5 J |
| 0,511 m/s | 120 pasos/min | 6 | Sí | Sí | 319,5 J |

Se solicitan seis pasos en cada caso. El contador conserva el criterio anterior: planta completa de pie y dedos a >=3 cm durante >=30 ms, pie contrario cargado, aterrizaje cargado y desplazamiento >=6 cm. Un paso visible por debajo de ese despeje no suma en este contador. No se rebaja el umbral para obtener seis. Ninguna de las cuatro marchas cae, pero **ninguna pasa todos los detectores de artefactos**. La marcha lenta tiene un apoyo adicional y es poco natural; la de 0,216 m/s alcanza 1,34 cm de penetración de suelo y se rechaza.

En la comparación a la misma cadencia de 0,8 s/paso y con **modelo, duración y estado inicial completo idénticos**, los nuevos objetivos reducen:

| Medida | Antes | Después |
|---|---:|---:|
| Error lateral máximo de aterrizaje | 5,99 cm | 2,63 cm |
| Traslación máxima de pie cargado | 5,83 cm | 2,41 cm |
| Trabajo motor positivo | 432,4 J | 269,5 J |

Son mejoras medidas; la traslación de pie todavía supera el límite. El diagnóstico de velocidad material de los contactos también confirma deslizamiento real. Se conserva separadamente la traslación del origen del pie, que puede incluir giro/rodadura; no se usa esa ambigüedad para quitar una bandera.

## Pecho: direcciones e intensidades

La batería definitiva incluye **48 ensayos de impacto y dos controles sin golpe**:

- Cuatro direcciones mundiales: +X delante, -X atrás, +Y izquierda, -Y derecha.
- Impulsos de 8, 20 y 36 N·s.
- Pulsos rectangulares de 80 ms y 180 ms, denominados equivalentes de puñetazo y patada.
- Tracker original y tracker con planificador observado, desde el mismo estado.
- Tres segundos de preparación física libre, guardados aparte. No se coloca ni frena cinemáticamente el cuerpo para estabilizarlo.

**Son cargas analíticas en el centro de masa del pecho, no golpes de un atacante articulado.** La batería prueba la respuesta corporal a esas cargas; no valida la ejecución de una patada/puñetazo ni el choque entre dos humanoides. Las colisiones del cuerpo consigo mismo y con el suelo permanecen activas.

Las 24 parejas cumplen igualdad de modelo, estado inicial completo, referencia de entrada, trayectoria/órdenes anteriores al impacto e impulso integrado. Resultado global, incluyendo controles:

- 45/50 terminan con dos segundos estables.
- Cuatro caídas: los golpes fuertes de 36 N·s hacia atrás, en ambos perfiles y ambos controladores.
- Seis ensayos terminan estables y presentan pasos que pasan el contador de despeje; el planificador puede añadir un segundo paso frontal.
- Sólo cuatro ensayos quedan sin banderas; **cero recuperaciones con paso pasan todos los controles físicos**.
- El planificador no aporta una mejora general; no debe reemplazar al tracker por defecto. Hay además un ensayo sin caída que no cumple la estabilidad final.

Seguir de pie no equivale a una recuperación aceptada: se conservan deslizamientos, vuelos involuntarios, penetraciones y velocidades articulares excesivas. Los casos de caída muestran también el límite de mantener una referencia de postura cuando ya no es recuperable; falta arbitraje de caída.

## Qué se ha descartado

Las hipótesis y resultados se conservan; no se hacen barridos ciegos de ganancias.

| Prueba | Conclusión |
|---|---|
| Compensación lateral sólo del pie derecho | Empeora la colocación intermedia hasta 15 cm. |
| Amortiguación de apoyo por Jacobiano y motores existentes | No mejora globalmente; aumenta penetraciones y empeora la marcha lenta. |
| Contacto normal de 10/5 ms | Cambia la estabilidad; no se adopta para ocultar penetraciones. |
| Tres iteraciones NoSlip | Reduce parte de la deriva, empeora otros apoyos; no se adopta globalmente. |
| Cono de fricción elíptico, resto idéntico | No conserva estabilidad final en todos los casos. |
| Pelvis casi estática a 0,11 m/s | Persisten apoyos extra; prueba rechazada. |
| Avance continuo con vuelo tardío | Objetivos inalcanzables por 27–76 mm; se corrige el vuelo y se impone la puerta de IK. |

Los primeros ensayos con modo de entrenamiento, restauración incompleta, selección incorrecta de clip o velocidades residuales en una supuesta postura fija quedan como diagnósticos en out/. No sustentan la tabla definitiva. Las referencias estáticas definitivas tienen velocidades lineales, angulares y articulares exactamente nulas.

## Interpretación biomecánica

Los motores de este modelo no son músculos. Pares y trabajo mecánico no miden EMG, coactivación ni energía metabólica. En estas cuatro marchas, aumentar velocidad ayuda a la regularidad de los pasos, pero el trabajo no decrece monótonamente: pasa de 269,5 J a 319,5 J entre 0,322 y 0,511 m/s.

Un estudio de 22 personas encontró menor coste **adicional** de estabilización sagital ante perturbaciones de cinta a mayor velocidad: [Muijres et al., 2026](https://pubmed.ncbi.nlm.nih.gov/41269262/). Eso respalda una parte concreta de la hipótesis del usuario, no que toda la actividad muscular disminuya; otro estudio observó mayor activación media del tronco con velocidad: [Estudio de activación del tronco](https://pubmed.ncbi.nlm.nih.gov/16517182/).

Esta batería aplica los golpes partiendo de pie. Para comprobar si caminar más rápido mejora la recuperación ante el mismo golpe falta una comparación de impactos **durante la marcha** a las mismas cadencias. No se infiere esa conclusión de los ensayos estáticos.

## Reproducir y revisar

Usar el entorno y fuentes fijados por scripts/controller_lab/prepare.ps1; versiones, hashes, licencias y medidas completas en [el paquete de evidencias](experiments/protomotions-gait-20261116/provenance.json). No se usan motions crudos de BONES-SEED, AMASS ni SMPL. Código upstream Apache-2.0 y pesos bajo NVIDIA Open Model License, con condiciones distintas de los datos de entrenamiento.

Desde la raíz de este worktree:

~~~powershell
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/gait_suite.py walk --output out/gait/reproduction-walk
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/gait_suite.py impacts --output out/gait/reproduction-impacts
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/report_gait.py out/gait/reproduction-walk
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/report_gait.py out/gait/reproduction-impacts
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/render_gait.py out/gait/reproduction-walk/higher out/gait/reproduction-video.mp4 --slow
.\out\native\.venv\Scripts\python.exe -m unittest discover -s scripts/controller_lab -p "test*.py" -v
~~~

Vídeos locales en out/gait/delivery/: marcha-higher.mp4, marcha-higher-x025.mp4, cuatro-velocidades.mp4, marcha-antes-despues.mp4 e impactos-antes-despues.mp4. El último muestra cuatro escenarios representativos, tracker arriba y planificador debajo, ambos con frente/perfil. Son **diagnósticos**, incluidos fallos; no candidatos aprobados. En la rejilla de velocidades, al finalizar cada grabación se mantiene su último fotograma; ese panel no añade tiempo físico simulado. El replay prohíbe mj_step y todos los MP4 se decodifican completos.

Validación: **23 pruebas nativas**, incluidas inferencia congelada, impulso exacto, velocidad física, referencias estáticas y distinción entre rodadura/deslizamiento; typecheck, lint y build correctos; **706 pruebas TypeScript pasadas**, 9 omitidas y 3 pendientes. CI por sí sola no verifica esta física.

Siguiente prioridad tras el feedback visual: reducir el deslizamiento restante sin endurecer artificialmente los pies, gobernar las fases lentas por descarga/contacto observado y añadir paso de recuperación hacia atrás con arbitraje de caída. Después, repetir con impactos durante la marcha y adaptar el controlador al Uke y al rig consumidor. La issue #77 permanece abierta.
