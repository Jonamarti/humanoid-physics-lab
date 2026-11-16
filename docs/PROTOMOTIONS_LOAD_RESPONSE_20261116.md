# ProtoMotions: recepción del peso y vibración del tronco (16-11-2026)

La preparación de flexión antes del talón reduce la vibración rápida de cabeza y pecho en esta marcha, conserva seis pasos alternos y termina estable. Sigue siendo un diagnóstico: persiste el deslizamiento y aumenta el trabajo motor. No se promueve al núcleo ni se declara validado para otras velocidades, golpes o tallas.

## Qué hacen las personas y qué implementamos

- Al recibir el peso, la rodilla puede flexionarse mientras los extensores producen un momento de frenado: trabajo negativo. Winter midió fases de absorción en rodilla y tobillo, seguidas de generación en otras fases; la absorción del cuádriceps cambia con la velocidad. [Estudio original, 15 adultos](https://pubmed.ncbi.nlm.nih.gov/6839580/).
- La cadera ayuda a sostener y controlar la pelvis. En un modelo muscular contrastado cualitativamente con EMG aparecen extensores de cadera/rodilla al comienzo del apoyo, flexores plantares antes del despegue, tibial anterior en apoyo temprano/oscilación y frenado de la pierna al final de la oscilación. Es coordinación temporal, no aumentar la tensión de todo el cuerpo. [Rajagopal y colaboradores](https://pmc.ncbi.nlm.nih.gov/articles/PMC5507211/).
- Sóleo y gemelos no cumplen una única función intercambiable. En estimulación experimental, ambos modificaron soporte y avance del centro de presión; sus efectos sobre la aceleración hacia delante dependieron del músculo y de la fase. [Francis y colaboradores, 20 participantes](https://pmc.ncbi.nlm.nih.gov/articles/PMC3795949/).
- Pie, calzado, tejidos y tendones también absorben o almacenan energía. Nuestros volúmenes rígidos y motores articulares no reproducen esa anatomía. No atribuimos el par de un motor a un músculo concreto ni inferimos activación muscular de su potencia.

La hipótesis mecánica es llegar al apoyo con la pierna preparada para recibir el peso y devolver la altura gradualmente. No hemos añadido cuádriceps, gemelos ni tendones fisiológicos.

## Por qué la reacción al impacto no bastó

En el control guardado, los máximos de aceleración de cabeza/pecho aparecen aproximadamente 8 ms después del segundo contacto de talón, a t=2,043 s. La referencia reactiva se actualiza a 30 Hz; no puede deshacer una colisión ya resuelta. Además, en cinco de las seis recepciones las rodillas seguían extendiéndose durante los primeros 100 ms.

Se probaron hipótesis sucesivas, con sus resultados negativos conservados:

1. Ceder 8 mm de altura de referencia tras la carga real: reducción pequeña de vibración de cabeza, sin mejora del pecho.
2. Repartir ese descenso entre 3 mm de preparación y 5 mm reactivos: no mejoró.
3. Reducir 20 % la rigidez sagital de cadera/rodilla durante la recepción: aumentó la absorción de rodilla, pero también el trabajo total y las correcciones al parar; se descarta.
4. Mantener la rigidez y añadir 50 % de amortiguación viscosa sólo tras un paso real: aumentó la absorción, pero no redujo los máximos; se descarta.
5. Preparar 3 cm de descenso de referencia hasta 50 ms después del talón: bajó la vibración media, pero empeoró los máximos.
6. Adelantar 90 ms sólo la subida de esa preparación, manteniendo profundidad y recuperación: conserva la reducción media y reduce los máximos. Es la variante del vídeo nuevo.

El descenso es un objetivo de referencia generado mediante IK en un modelo de cálculo separado, con los dos objetivos mundiales de los pies y sus rotaciones conservados. En el cuerpo simulado se aplican únicamente pares motores limitados: no se escriben poses durante la marcha ni se aplican fuerzas a la raíz. El tracker aprendido sigue siendo un seguidor con anticipación de referencias; esto no lo convierte en un planificador autónomo de pasos.

## Comparación congelada

Anatomic SOMA, masa 42,466827 kg; MuJoCo 3.5.0 a 960 Hz; política a 30 Hz; grabación a 120 Hz. Seis pasos solicitados, periodo 0,5 s/paso y 9 s por ensayo, con parada incluida. No es el Uke de 75 kg.

La comparación final mantiene idénticos el estado inicial completo, las observaciones iniciales y todas las matrices del modelo compilado: masas, volúmenes, contacto de 10 ms, rozamiento, ganancias, límites motores y articulaciones. Cambia la referencia coordinada de altura/flexión, conservando el recorrido horizontal y talón-punta. El control anterior se reprodujo exactamente en estados, cargas y órdenes respecto a la grabación congelada.

| Medida | Antes | Preparación adelantada |
|---|---:|---:|
| Pasos alternos con apoyo y despeje real | 6 | 6 |
| Aterrizajes dominados por talón | 6/6 | 6/6 |
| Ciclos talón→antepié antes del siguiente despegue | 4/4 | 4/4 |
| Dos segundos finales estables | Sí | Sí |
| Velocidad media real (m/s) | 0.517 | 0.523 |
| Vibración de cabeza 4–15 Hz (m/s²) | 0.855 | 0.725 |
| Vibración de pecho 4–15 Hz (m/s²) | 1.292 | 1.227 |
| Máximo de aceleración de cabeza (m/s²) | 14.78 | 12.47 |
| Máximo de aceleración de pecho (m/s²) | 22.54 | 21.63 |
| Oscilación lateral del pecho, desviación típica (°) | 1.93 | 1.65 |
| Trabajo motor positivo total (J) | 242.3 | 263.5 |

La vibración es la RMS de las tres componentes de aceleración en 4–15 Hz, calculada durante t=1–4 s. El filtro se aplica al análisis; no suaviza las poses ni el vídeo. El máximo es la norma de aceleración estimada de velocidades registradas a 120 Hz; no es una fuerza de choque ni una medida clínica.

La rodilla llega más flexionada: 22–27 grados frente a 16–21 grados. La energía negativa de los motores de rodilla en los primeros 200 ms de las seis recepciones pasa de 1,81 a 3,68 J, pero también aumenta su generación positiva, de 5,34 a 10,88 J. Hay más absorción y más trabajo: no es una mejora de eficiencia ni prueba de fisiología muscular.

## Qué sigue sin estar resuelto

Ambos ensayos conservan la bandera de desplazamiento del origen de pie cargado (>2 cm). La integración adicional de velocidad material en los contactos también llega a unos 2,1 cm en el pie derecho: la rodadura no explica todo. El mayor hundimiento del suelo aumenta de 5,65 a 7,12 mm, por debajo del límite de 1 cm; no se ha cambiado ningún umbral.

La oscilación lateral del pecho mejora, pero el cabeceo lento de la cabeza aumenta (desviación típica de inclinación: aproximadamente 1,46 a 1,75 grados). La cabeza no queda inmovilizada ni se han filtrado sus movimientos. La naturalidad visual debe valorarse con el vídeo; la reducción en 4–15 Hz no es una puntuación universal de marcha humana.

No se han repetido todavía otras cadencias ni impactos durante esta marcha. No se deduce una mejora de recuperación ante golpes de este ensayo sin perturbación. Antes de extenderlo: reducir patinaje y trabajo de la recuperación de altura, comprobar el cabeceo lento y después repetir velocidades/golpes desde estados comparables.

## Versiones y evidencias

- Marcha anterior de pie plano intacta: etiqueta remota codex/protomotions-flat-foot-baseline-20261116, commit 2b8d963.
- Talón-punta anterior intacto: etiqueta remota codex/heel-toe-before-load-response-20261116, commit 8c703f4.
- Vídeos anteriores comprobados por SHA-256, sin modificaciones.
- Nuevo vídeo local: out/load-response/delivery/recepcion-antes-despues.mp4, con ×1 y ×0,25; también versiones separadas y hojas de fotogramas. Izquierda: antes. Derecha: preparación adelantada. Fuerzas verticales y reparto talón/antepié proceden de los contactos resueltos; se prohíbe integrar física al reproducir.
- [Tabla completa, incluidos fallos](experiments/load-response-20261116/metrics.csv), [ensayos e hipótesis](experiments/load-response-20261116/hypotheses.json), [resúmenes completos](experiments/load-response-20261116/cases.json) y [auditoría y hashes](experiments/load-response-20261116/audit.json).
- Fuente oficial ProtoMotions fijada a 13de0cccd06f53ce5c9cf9ec7d1932c7da73610f; checkpoint c13e56e85494f8c93fe9eed5b07cadef7281c2eb1188231755f858e1d0f340f0. Referencias propias; sin distribuir motions BONES-SEED/AMASS/SMPL. Código Apache-2.0; pesos sujetos a NVIDIA Open Model License.

Validación: 50 pruebas nativas correctas, incluidos detección de un paso con apoyo contrario/despeje/duración, ausencia de reactivación por simples descargas en suelo, causalidad del objetivo reactivo, amortiguación disipativa con saturación, IK alcanzable, reproducción de ganancias registradas y protección contra sobrescritura. Las comprobaciones TypeScript anteriores siguen documentadas en PROTOMOTIONS_GAIT_20261116.md (706 correctas); esta modificación no toca el núcleo TypeScript ni los workflows.

Para reproducir, usar carpetas nuevas: los ejecutores rechazan sobrescribir experimentos preservados.

~~~powershell
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/prepared_acceptance.py --early --output out/load-response/reproduction-early
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/render_rollover.py out/load-response/reproduction-early/before out/load-response/reproduction-early/prepared_early out/load-response/reproduction-video/comparacion.mp4
.\out\native\.venv\Scripts\python.exe -m unittest discover -s scripts/controller_lab -p "test_*.py"
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/evidence_load_response.py
~~~

Los experimentos reactivos se reproducen con load_response.py --output <nueva>, weight_acceptance.py --output <nueva> y weight_acceptance.py --damping --output <nueva>. La preparación que empeoró los máximos se conserva como prepared_acceptance.py sin --early. La auditoría del paquete reúne los resultados originales preservados, no descubre automáticamente una carpeta de reproducción.
