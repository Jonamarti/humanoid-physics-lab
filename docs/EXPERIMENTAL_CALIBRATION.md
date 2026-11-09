# Calibración biomecánica con datos experimentales abiertos

**Propósito:** validar y ajustar el humanoide sintético de 75 kg, sin confundir
la física de MuJoCo con fisiología humana ni convertir muestras de personas en
límites universales de daño, KO o fuerza de derribo.

## Conjuntos de datos con trazabilidad

| Prioridad | Recurso y fuente | Medidas disponibles | Aplicación |
| --- | --- | --- | --- |
| A | *Full Body Motion Capture of Single Individuals Following External Perturbations from Different Directions*, Zenodo 10512652, DOI 10.1016/j.jbiomech.2023.111703 | 21 adultos, peso medio 70.2 ± 12.1 kg, talla 1.74 ± 0.08 m; 45 marcadores 3D a 200 Hz; fuerzas temporales sincronizadas de perturbaciones a altura de hombros; paquete C3D/7z ~1.2 GB | Reproducción de empujones dirigidos, amplitud/tiempo de reacción, trayectorias de tronco/CoM, *sin asumir* que todas las pruebas acaban en caída. https://zenodo.org/records/10512652 |
| A | *Standing Balance Experiment with Long Duration Random Pulses Perturbation*, Zenodo 3631958 | 8 sujetos, 4 ensayos de 5 min cada uno; markers 100 Hz, GRF 6D, EMG y ángulos/torques estimados de cadera, rodilla y tobillo; procesados ~264 MB y crudos ~2.9 GB | Ajustar respuesta de tobillo/cadera, amortiguamiento, CoP, error de postura, carga real de apoyo. https://zenodo.org/records/3631958 |
| A | *A data set with kinematic and ground reaction forces of human balance*, DOI 10.6084/m9.figshare.4525082 | 49 sujetos (27 jóvenes, 22 mayores), bipedestación de 60 s; plataformas de fuerzas separadas, CoP, 42 marcadores y 73 ángulos; 1,813 ficheros; licencia CC0 | Validar postura nominal, oscilación, reparto de cargas izquierda/derecha, sin extrapolar a empujones. https://pmc.ncbi.nlm.nih.gov/articles/PMC5534162/ |
| B | *Upper body perturbations dataset*, University of Twente, van Mierlo et al. 2022 | 10 participantes caminando en cinta con perturbaciones de pelvis, tronco o ambas, mocap, EMG y fuerzas | Validar **en fase posterior** respuestas mientras se camina, no directamente la bipedestación estática. https://research.utwente.nl/en/datasets/upper-body-perturbations-dataset/ |
| B | *Dynamic Spatial Tuning Patterns of Shoulder Muscles*, Fice et al. 2021, DOI 10.3389/fbioe.2021.761799 | 17 participantes sentados sujetos por el torso; carga desde 8 direcciones, EMG y desplazamientos de brazo | Control reactivo de hombro, **no** modelo directo de patada frontal recibida de pie. https://pmc.ncbi.nlm.nih.gov/articles/PMC8652075/ |

**Estado de adquisición:** catálogo de fuentes verificado mediante páginas de autores/archivos;
**aún no se han descargado los datasets ni medido errores contra ensayos emparejados**.
Comprobar licencias y términos en cada registro antes de importar o redistribuir
datos. No añadir material de sujetos ni material clínico a GitHub sin aprobación
de reutilización.

## Protocolo de comparación, sin trampas

1. **Modelo y escala.** Registrar masas segmentarias, longitudes,
   inercias, fricción, rigidez y rangos articulares de nuestro MJCF.
   Escalar sujetos de forma individual cuando sus metadatos lo permitan.
   No igualar todos los experimentos a 75 kg por definición.
2. **Entrada experimental.** Recuperar curva de fuerza F(t), punto 3D,
   coordenadas, duración y momento inicial. Integrar el impulso
   `J = ∫F(t) dt` (N·s); no equiparar fuerza pico con impulso.
   Para el dataset de hombros, la aplicación no es el esternón del
   experimento sintético actual; usar el punto anatómico correspondiente.
3. **Estado previo.** Igualar postura inicial, anchura/separación de pies,
   condición visual, apoyo, restricción y altura de COM cuando se conozcan.
   Etiquetar como *no comparable* si no puede emparejarse.
4. **Observables sincronizados.** Error temporal en desplazamiento frontal,
   lateral y vertical de COM; inclinación/velocidad de tronco; cadera,
   rodilla y tobillo; carga vertical izquierda/derecha; CoP;
   desplazamiento y tiempo de despegue/contacto de pies.
   Informar RMSE, MAE, máximos, tiempo de eventos y cobertura por sujeto.
5. **Calibración vs validación.** Ajustar ganancias PD, damping, reacción y
   fricción en sujetos/ensayos de entrenamiento; reservar participantes
   y direcciones para validación *out-of-sample*. Versionar parámetros,
   escalas, semillas, runtime MuJoCo, timestep y filtros.
6. **Grados de evidencia.** Marcar cada escenario como `validated`
   (medida emparejada), `plausible` (patrón cualitativo),
   `synthetic` (solo motor) o `not_tested`.
   No ajustar el modelo hasta forzar que todas las pruebas «pasen».

## Prioridad para el estado actual del código

- Fase 1: verificar **fuerzas de contacto reales** de los pies: con
  75 kg y quietud el total vertical *esperado en promedio* ronda
  `75 × 9.81 = 735.75 N`; en un instante puede diferir por aceleración.
- Fase 2: comparar bipedestación de 10 s contra el dataset CoP/GRF
  antes de optimizar reacciones.
- Fase 3: ajustar un ensayo de empujón con curva temporal y punto de aplicación
  de Zenodo 10512652; luego validar las demás direcciones y sujetos.
- Fase 4: pasos de recuperación y caídas (requieren observaciones
  de cambios de contacto, no simples poses).
- Fase 5: golpes: distinguir objetivo rígido, saco, maniquí y humano reactivo.
  Un pico en N al golpear un objetivo **no determina** el desplazamiento
  del receptor: la duración, la trayectoria, la compliance y la postura
  cambian la transferencia de momento. Buscar mediciones específicas de
  recepción antes de usar impactos de patadas para calibrar.

## Referencia técnica sobre cargas en MuJoCo WASM

La implementación de `mj_contactForce` usa `efc_force` y decodifica
el cono piramidal mediante `mju_decodePyramid`: para fricción piramidal
de dimensión 3 la normal es la suma de las cuatro componentes
`efc_force[efc_address ... efc_address+3]`.
En la API WASM 3.17.0 la función de salida puede no copiar de vuelta el
buffer JS pasado; el adaptador usa directamente las fuerzas resueltas.
Ver código fuente upstream:
- https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_core_util.c
- https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_util_misc.c
- https://mujoco.readthedocs.io/en/stable/programming/simulation.html#contacts
