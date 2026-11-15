# Revisión física de derribos — 15-11-2026

**Entrega experimental, no aprobada como dos técnicas biomecánicamente validadas.** No cambia Última Ronda ni AnimSpec. La rama parte de `main` remoto `2f06812586e6bfc06178cd7c20fb16156cdd0b51` (PR #105).

## Estado remoto comprobado

El clon principal seguía en `experiment/70-load-aware-recovery`, anterior al laboratorio de escenarios. Se creó un worktree independiente en `work/codex-physical-takedowns`, rama `codex/physical-takedown-validation`.

- #73–#80 siguen abiertas. Hay implementación fusionada de F1/F2/F3, pero sus checklists no certifican la aceptación final.
- #105 añade cuatro pistas de Tori; #103 permite cintura flexible; #100, editor web. Ya están en `main`.
- #62 y #71 siguen DRAFT/no-merge. No se modificaron. #104 es un comparador cinemático aislado; tampoco se fusionó ni se usó como evidencia dinámica.
- El despliegue remoto del commit base figura exitoso. Eso no valida las proyecciones.

## Diagnóstico y correcciones

### Masa del controlador

`makeStandingController` suma las masas del modelo entero. Al reutilizarlo para Uke en una escena con Tori, `createTenseController` usaba también la masa del atacante en el término gravitatorio de los tobillos. Ahora usa la masa declarada y generada de Uke. La prueba `actorMassIsolation` compara el mismo empujón con y sin un Tori distante de 150 kg, sin contacto ni agarres.

Una expectativa histórica de `toriBody.test.ts` dejó de ser cierta: la proyección sigue ocurriendo, pero el presupuesto detecta impulso horizontal excesivo al final. La regresión exige ahora ese incumplimiento concreto; no se aumentó su límite de 225 N·s.

### Agarres acoplados

Nuevo `tori.body.grip_mode: "coupled"`. El modo histórico `target` sigue disponible para conservar los escenarios y grabaciones anteriores. Las referencias nuevas usan `coupled`.

- El punto de Tori es el extremo del antebrazo **simulado**, no la mano ideal de la pista.
- Durante la ventana de cierre, el brazo intenta acercarse mediante el seguimiento limitado ya existente e IK de alcance finito. La posición dinámica del brazo nunca se sobrescribe.
- Solo adquiere si la distancia real es menor que el radio declarado. Un intento fuera de alcance termina en `missed` y no se vuelve a activar automáticamente.
- Cuerda con longitud de reposo capturada al cerrar, holgura, amortiguación relativa y topes de fuerza y potencia positiva sobre Uke. Se suelta por distancia, sobrecarga sostenida o tiempo programado.
- Fuerza sobre Uke y fuerza opuesta sobre el antebrazo. Ambos wrenches se aplican en el mismo punto del mundo, incluido `r × F`, para no introducir un par externo neto.
- Se registra el trabajo sobre Uke y el de la reacción sobre Tori. El balance de energía incluye ambos.

Los valores por defecto son parámetros de ingeniería sin calibración biomecánica: 2500 N/m, 80 N·s/m, holgura 2 cm, captura 12 cm, ventana 0,35 s, rotura a 25 cm y suelta tras 0,2 s de saturación. Las referencias limitan cada mano a 400 N; el valor por defecto de potencia es 600 W. No son máximos humanos certificados.

La referencia de siega prepara una manga y un punto frontal del pecho que aproxima la toma de solapa; no simula tela. La entrada a las piernas alinea las manos simétricamente y permite adquirir las corvas cuando llegan. Conservan la pista corporal sintética previa; no son nuevas animaciones aprobadas.

### Límites articulares

El generador ya tenía rangos anatómicos aproximados, pero los límites son restricciones blandas. Se mide ahora la excursión real a 500 Hz. `joint_limit_timeconst_s` permite endurecer su respuesta sin ampliar ningún rango. Las referencias usan 0,004 s, dos pasos físicos. Los modelos antiguos conservan su configuración.

La primera comparación de las referencias midió 0,2094 rad (12,0°) en la siega y 0,1298 rad (7,44°) en la entrada. Con respuesta más rígida bajaron a 0,0242 y 0,0176 rad, antes del ajuste de orden del ciclo físico. Los datos finales están en la tabla de evidencias; las restricciones siguen siendo blandas, no límites geométricos exactos. La prueba compara ambos modelos y exige que sus rangos sigan idénticos.

### Frecuencia de grabación

El ejecutor histórico llamaba a `mj_forward` al guardar una muestra, antes del control de Uke. Cambiar `record_hz` modificaba las oportunidades de actualizar contactos y sesgos dinámicos. En el modo nuevo se calculan control y contactos en cada paso físico; el muestreo solo observa. `recordingCadence.test.ts` exige `qpos`, `qvel` y acumulados de agarre idénticos a 50 y 125 Hz en ambos mecanismos.

Se activa con agarres acoplados, respuesta protectora o configuración explícita del límite. `RunOptions.coherentControl` permite usarlo directamente. El modo histórico conserva el orden anterior para no invalidar silenciosamente toda la galería; su dependencia de muestreo sigue siendo una limitación documentada.

### Protección opcional

`actors[0].fall: "instinctive"` activa una respuesta clásica experimental. La captura fuera del apoyo por sí sola no basta: exige movimiento descendente real sostenido; también observa inclinación de tronco/pelvis y ausencia de apoyo. Tras 100 ms de evidencia y 150 ms de latencia modifica objetivos articulares mediante pares limitados; tras contacto corporal reduce su rigidez progresivamente. No escribe posiciones ni fuerzas sobre la raíz, ni decide el veredicto.

Una primera versión basada solo en el margen de captura estropeaba una recuperación frontal de 25 N·s. Se descartó; ese caso está en la regresión. La versión entregada mantiene las recuperaciones ensayadas. En dos caídas independientes reduce el pico de contacto, pero aumenta la penetración del antebrazo: es una compensación pendiente, no ukemi validado. No se activa por defecto.

## Modelo físico y asistencia que permanece

Uke tiene pelvis libre, masas e inercias de geometrías, articulaciones con rangos, control por pares internos limitados y contactos MuJoCo con fricción. Los rangos existentes son aproximaciones clínicas pendientes de calibración multiaxial; no equivalen a conos anatómicos completos.

**Tori continúa formado por 14 segmentos libres, sin articulaciones entre ellos ni equilibrio propio.** Cada segmento recibe seguimiento externo limitado. Las nuevas grabaciones declaran `meta.tori_assist: "independent_segment_tracking"` y el modelo de agarre. La reacción de la mano sí llega al segmento; su transmisión hasta tronco, piernas y suelo aún requiere el segundo humanoide articulado de #79. No se oculta esta asistencia ni se interpreta un presupuesto favorable como demostración de ejecutabilidad humana.

## Demostraciones y reproducción

```powershell
npm.cmd run sim -- scenarios/physical_refs/outer_reap.json --out out/demo_outer
npm.cmd run sim -- scenarios/physical_refs/leg_drive.json --out out/demo_legs
```

En el visor local, **Cargar grabación** abre el `recording.json.gz` generado. Ofrece pausa, ×0,25, avance por muestra y barra de tiempo. El centro de masas y el apoyo usan los datos físicos grabados; el anillo magenta es el punto de captura y la flecha azul representa 0,3 s de desplazamiento a la velocidad del COM. El punto de captura es una aproximación de planificación del péndulo invertido, no un detector universal de caída.

En **Fuerzas, apoyos y estado de los agarres** se inspeccionan carga de cada pie, separación y estado de cada agarre, asistencia y artefactos. Contactos y reacciones son capas activables. Las flechas de fuerza reutilizan el sistema existente, incluidos sus glifos de entrada/salida de pantalla. `joint_torques_nm`, velocidad del COM, momento angular y reacción del suelo también están en el JSON de las ejecuciones detalladas. Los límites articulares y penetraciones se acumulan a 500 Hz.

## Campañas reproducibles

`tests/manual/takedownBench.test.ts` escribe escenarios efectivos, grabaciones y `results.json`, sin seleccionar casos exitosos. Antes y después se ejecutaron las mismas 22 condiciones. La matriz ampliada añade protección, atacante ligero, acción lenta, orientación oblicua, proporciones y pasivo sin atacante: 34 condiciones.

```powershell
$env:HPL_TAKEDOWN_BENCH='1'
$env:HPL_COUPLED_GRIPS='1'
$env:HPL_REF_SUITE='1'
$env:HPL_RESULTS_DIR='out/takedowns/new_campaign'
npm.cmd exec -- vitest run tests/manual/takedownBench.test.ts --configLoader runner --maxWorkers=1 --no-file-parallelism
```

Quitar `HPL_REF_SUITE` usa las pistas históricas para comparar con la línea base. Las variables solo habilitan la campaña; no cambian resultados ni reglas de éxito. `passive_no_attacker` evita atribuir al derribo el desplome espontáneo de un cuerpo sin tono.

Las grabaciones completas y vídeos están en `out/takedowns/` y se mantienen fuera de Git. La tabla compacta final se conserva en `docs/experiments/takedowns-20261115/`. No se sustituyen las grabaciones públicas de la galería por estas candidatas.

## Limitaciones y aceptación pendiente

- No hay todavía recuperación reactiva demostrada mediante un paso correctivo. El campo histórico `step` solo detecta desplazamiento del pie; **no demuestra un paso motor válido**, especialmente durante una caída.
- Las fases siguen siendo intención temporal de la pista; todavía no hay transición general por condiciones dinámicas.
- La siega cae demasiado de lado y el cuerpo de Tori se deforma al separarse sus segmentos. Sigue faltando coordinación humana.
- Ambas pistas pueden derribar sin agarres por su contacto corporal. Eso demuestra que estas referencias no aíslan aún el mecanismo técnico solicitado; no se presentan como O-soto-gari o double-leg excelentes.
- Persisten penetraciones de impacto, deslizamientos y algunos temblores. El detector sigue señalándolos: no se relajaron sus umbrales. Endurecer el suelo para esconder penetración aumentaría el pico de fuerza.
- El presupuesto de Tori sigue dando casos inviables; no se impone todavía como restricción dinámica global.
- Solo el elemento de agarre se ensaya con timesteps de 1, 2 y 4 ms. Las campañas de humanoide utilizan 2 ms fijos; no se declara robustez del cuerpo completo a otros timesteps.
- Escritorio y viewport móvil emulado comprobados; sin validación en un móvil real, Safari, tejido blando ni contra captura de movimiento humana.

Por estos motivos, la PR debe permanecer **DRAFT, sin fusionar ni desplegar automáticamente**. Se entrega una corrección funcional verificable de fundamentos, referencias de prueba y diagnóstico; los criterios globales de dos técnicas excelentes y recuperación con paso siguen abiertos. No se añadió una tercera técnica.

## Siguiente trabajo e integración futura

1. Un único humanoide articulado de Tori, con reacciones a través de la cadena cinemática y pelvis libre (#79), antes de atribuir credibilidad global a las proyecciones.
2. Recuperación de un paso con cargas y aterrizaje medidos (#77), conservando los ensayos negativos de #71. No repetir barridos de ganancias ya descartados.
3. Ablaciones que separen empuje corporal, toma de piernas y segado; ajustar intención motora, nunca el resultado ni la posición de Uke.
4. Calibrar contacto de impacto y protección conjuntamente, conservando sus dos costes: fuerza y penetración.
5. Para Última Ronda: exportar clips y metadatos solo tras validar estos hitos y adaptar proporciones/contactos al rig. No se realizó integración ni se tocaron sus archivos.

## Fuentes primarias contrastadas

- [MuJoCo: parámetros del solver y restricciones blandas](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters). Justifica distinguir rango articular de cumplimiento exacto y no confundir rigidez con ausencia de penetración.
- [MuJoCo: `mj_subtreeVel`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-subtreevel), para velocidad del COM y momento angular del subárbol.
- Las ganancias, latencias, postura protectora y topes de agarre de esta entrega son estimaciones explícitas, no valores biomecánicos extraídos de esas fuentes.

### Otras expectativas históricas corregidas

La corrección de masa también excluye obstáculos del término gravitatorio de Uke. El escenario histórico de cadera `uke_tense_75_175_hip_roll` sigue girando sin golpear la cabeza, pero revela contacto excesivo entre `right_shin|left_foot` (~11,0 kN), penetración entre segmentos (~6,59 cm) y contra el suelo (~6,05 cm). Sus tests e informe exigen ahora mostrar esos defectos; no se ocultan ni se elevan los límites. Esta tercera técnica histórica no se ha perfeccionado ni se propone para uso. La prueba de claves desconocidas pasó de `fall` (ahora implementada) a `fall_mode` (sigue siendo inválida).

## Verificación final ejecutada

`npm run typecheck`, `npm run lint` y `npm run build -- --configLoader runner`: pasan. La batería completa final, `npm test -- --configLoader runner --maxWorkers=2`, termina con **725 pruebas correctas, 11 omitidas y 3 todo** (93 archivos correctos, 8 omitidos). Las campañas manuales se ejecutaron aparte; sus omisiones en la suite habitual no se cuentan como evidencia física.

Tres mutaciones temporales fueron detectadas y restauradas: masa de todo el modelo en Uke, reacción de agarre eliminada y control ligado a la frecuencia de grabación. Visor comprobado en Edge sin ventana con pausa, tiempo, avance, capas de contactos/reacciones y viewport móvil de 390 × 844, sin errores de página ni desbordamiento. Se inspeccionaron hojas frontales/laterales; no se afirma revisión humana completa de todos los fotogramas.

Resultados compactos: [tabla y protocolo](experiments/takedowns-20261115/README.md), [78 filas CSV](experiments/takedowns-20261115/metrics.csv), [manifest de validación](experiments/takedowns-20261115/validation.json). Informe audiovisual local: `out/takedowns/delivery/index.html`. Los vídeos no se incluyen en Git. La compilación conserva los avisos de externalización de módulos de Node y tamaño del bundle de Vite.
