# Golpes y derribos existentes con ProtoMotions — 16-11-2026

La transferencia funciona: el humanoide responde a los golpes y los contactos/agarres de Tori provocan las dos caídas. No produce todavía caídas naturales ni recuperaciones limpias con paso. El planificador de recuperación no mejora estos casos; en varias caídas aumenta el trabajo motor y el máximo de contacto.

## Qué significa «motor nuevo»

La física sigue siendo MuJoCo. Lo que cambia es el controlador aprendido ProtoMotions sobre el modelo publicado Anatomic SOMA (42,466827 kg), con la amortiguación de tronco/piernas de la entrega de marcha y un planificador opcional basado en el estado observado.

La preparación de flexión de la marcha no es un reflejo universal: necesita una referencia de paso. Estos ensayos parten de pie, con referencia de postura y posibilidad de proponer un paso observado. No activan automáticamente la trayectoria de marcha de seis pasos ni validan que ésta resista un golpe andando.

Tori conserva las pistas públicas existentes y sus fuerzas/par limitados. Sigue siendo el atacante abstracto de catorce segmentos libres independientes y 75 kg; no tiene articulaciones que unan esos segmentos ni un controlador aprendido de equilibrio. No se ha sustituido por el atacante articulado de otra rama. La viabilidad global de su equilibrio/presupuesto no se valida aquí.

## Escenarios trasladados

- Banco existente de golpes: puño equivalente de 20 N·s hacia delante/atrás en 80 ms; patada equivalente de 36 N·s lateral/hacia atrás en 180 ms. Se conserva la fuerza y duración de gait_suite.impact_cases, a t=2,3 s.
- Son pulsos analíticos aplicados al pecho, no golpes de un brazo o pierna articulados. Los vídeos muestran las reacciones de Uke y la dirección de la carga.
- Siega exterior y entrada a las piernas: las dos referencias públicas corregidas de la rama codex/physical-takedowns, commit 1d10c0643a038534d82941c1cfeda7e1b01e87c8. Sus pistas se copian sin cambiar; se añaden dos segundos de postura inicial antes de iniciar la acción.
- Son referencias del laboratorio HPL; no se han retargeteado los clips privados de Blender/AnimSpec.
- Dos modos por escenario: tracker congelado, y el mismo tracker con planificador de paso por COM/velocidad/apoyo observados. El planificador no recibe el reloj, dirección ni fuerza del golpe.

La entrega final reúne 14 ensayos: dos controles sin ataque, ocho golpes y cuatro derribos. Cada ensayo dura 8 s, además de 3 s reales de asentamiento registrados aparte. El lote inicial sin asentamiento se conserva en out/existing-scenarios/trials-v1; tenía patinaje incluso sin ataque. El catálogo final-v3 conserva los diez controles/golpes de prepared-v2 y las cuatro proyecciones corregidas de native-targets-v3, sin duplicar ni sobrescribir grabaciones.

## Física y adaptador

- MuJoCo nativo 3.5.0, 960 Hz físicos; política 30 Hz; grabación 120 Hz. Código upstream/pesos fijados por la entrega anterior.
- Uke mantiene los 52 motores y los 53 cuerpos de observación del tracker; los cuerpos auxiliares de Tori se añaden al final y quedan fuera de la ordenación de observación de la política.
- Las referencias de Tori se calculan con las funciones TypeScript existentes, evaluadas en cada paso nativo de 960 Hz. La exportación preliminar a 240 Hz se rechazó: desviaba hasta 12,94 cm el objetivo del muslo derecho de la siega (t=1,340625 s) y 1,14 cm el brazo de la entrada. La exportación definitiva mide error cero respecto a la función TypeScript en la rejilla física; [auditoría de objetivos](experiments/existing-scenarios-20261116/targets-audit.json). Estos giros bruscos de la referencia no garantizan naturalidad del atacante. La IK de brazos durante la adquisición replica la espera del agarre hacia el punto corporal real.
- Cada segmento de Tori conserva masa e inercia; los límites horizontales, verticales y de par de su controlador permanecen activos y se auditan. Tori puede separarse de su pista ante el contacto; no se prescribe su posición durante la simulación.
- Los agarres acoplan dos puntos simulados: captura hasta 12 cm durante la ventana de adquisición, holgura, muelle/amortiguador, fuerza/potencia limitadas, rotura por distancia/sobrecarga y suelta programada. La tolerancia de captura es una aproximación; no certifica contacto de dedos o tela.
- Se aplican ambas fuerzas centrales y sus momentos sobre los cuerpos reales. Las pruebas comprueban fuerza y momento netos, incluso con puntos alejados de sus centros de masa. No hay fuerza directa a la raíz de Uke ni escritura de sus poses durante el ensayo.
- Los sitios antiguos se adaptan explícitamente a cuerpos/puntos SOMA (nuca, codo y parte posterior de rodilla); no son los mismos volúmenes ni proporciones del Uke histórico de 75 kg.

## Comparación y auditoría

Los estados físicos completos y las observaciones iniciales de cada pareja son idénticos. La referencia de entrada y las órdenes anteriores a la intervención también coinciden. Las trayectorias y parámetros del atacante son los mismos; sus fuerzas de contacto posteriores pueden cambiar por la respuesta distinta de Uke.

La auditoría conserva los hashes brutos de cada modelo. En las cinco parejas de controles/golpes del lote prepared-v2, recalcular la amortiguación desde la inercia produjo diferencias de redondeo en la columna de velocidad de actuator_biasprm. La diferencia máxima es 3,55×10⁻¹⁵; el resto de matrices/opciones coincide exactamente. Las dos parejas definitivas de derribos sí tienen huellas binarias de modelo idénticas. Se registra la diferencia y se comprueba que queda dentro de 64 epsilon de float64 escalados por el valor; no se presenta como identidad binaria. El ejecutor entregado congela esa configuración una vez por pareja para evitar el recálculo. Los hashes de fuentes corresponden a los archivos entregados; esta congelación es posterior a los controles/golpes v2 y ya se aplica a los derribos v3.

Los umbrales físicos de penetración, velocidad, fuerza, vuelo y deslizamiento se conservan. La tolerancia numérica anterior no altera esos detectores. Ninguna comparación certifica superioridad sobre el Uke anterior: cambian talla, masa, articulaciones y backend, por lo que haría falta otro experimento controlado.

[Auditoría de siete parejas](experiments/existing-scenarios-20261116/pairs.json) · [Métricas completas](experiments/existing-scenarios-20261116/metrics.csv) · [Resúmenes y hashes](experiments/existing-scenarios-20261116/results.json) · [Procedencia y vídeos](experiments/existing-scenarios-20261116/provenance.json).

| Caso | Modo | Cae | Estable al final | Pasos contados | Suelo, cm | Máximo de contacto, kN |
|---|---|---|---|---:|---:|---:|
| Sin ataque | tracker | No | Sí | 0 | 0.04 | 0.39 |
| Sin ataque | planificador | No | Sí | 0 | 0.04 | 0.39 |
| Puño +X 20 | tracker | No | Sí | 0 | 0.94 | 1.08 |
| Puño +X 20 | planificador | No | Sí | 0 | 0.82 | 1.43 |
| Puño -X 20 | tracker | No | Sí | 0 | 0.78 | 2.70 |
| Puño -X 20 | planificador | No | Sí | 0 | 0.78 | 2.70 |
| Patada +Y 36 | tracker | No | Sí | 0 | 0.43 | 2.06 |
| Patada +Y 36 | planificador | No | Sí | 0 | 0.43 | 2.06 |
| Patada -X 36 | tracker | Sí | No | 0 | 2.00 | 3.28 |
| Patada -X 36 | planificador | Sí | No | 0 | 2.29 | 6.20 |
| Siega exterior | tracker | Sí | No | 1 | 1.65 | 2.86 |
| Siega exterior | planificador | Sí | No | 0 | 1.80 | 4.00 |
| Entrada a las piernas | tracker | Sí | No | 0 | 1.56 | 3.44 |
| Entrada a las piernas | planificador | Sí | No | 0 | 1.95 | 4.81 |


Los dos controles sin ataque terminan estables y sin artefactos. Los puños de 20 N·s y la patada lateral de 36 N·s se compensan sin caída en ambos modos, con patinaje. La patada fuerte hacia atrás y ambas proyecciones hacen caer en ambos modos.

No hay recuperación limpia con paso. Un despeje/paso contado durante la siega del tracker termina en caída; no se cuenta como recuperación. El máximo de contacto al golpe fuerte hacia atrás pasa de 3,28 a 6,20 kN con el planificador. En entrada a las piernas, el trabajo positivo de Uke pasa de 475 a 787 J. Es un resultado negativo de esta capa reactiva, no una mejora que deba activarse por defecto.

En las siegas aparece penetración entre actores de 2,18–2,43 cm, superior al límite de 2 cm. En las caídas hay también penetración del suelo y, en varios casos, velocidad articular excesiva. Los defectos se muestran; no se relajan umbrales ni se ocultan los ensayos fallidos.

## Qué se ve y conclusión

Los contactos desplazan el cuerpo y los agarres pueden tirar y soltarse sin fuerza a distancia. Uke ofrece una respuesta de equilibrio a perturbaciones moderadas. Al perder apoyo, el objetivo de postura sigue intentando corregir hacia estar de pie; aparecen movimientos rígidos, extensión y sacudidas en el suelo. Afirmar que esa intención causa cada pico concreto requeriría una ablación de los pares/objetivos durante la caída; aquí se observa el problema y la potencia añadida, sin atribuir todo a un único mecanismo.

ProtoMotions ayuda a controlar el cuerpo, pero seguir una postura o una marcha no proporciona por sí solo una caída protectora. La siguiente intervención debe arbitrar equilibrio/paso/caída desde apoyos y movimiento observados, limitar extensión y potencia añadida tras perder el soporte y evaluar una referencia de caída alcanzable. Comparar contra estos mismos casos antes de entrenar o integrar en el rig consumidor. Después, añadir golpes con extremidades articuladas y repetir durante la marcha.

## Vídeos y validación

Vídeos locales en out/existing-scenarios/delivery/:

- golpes-protomotions.mp4: cuatro cargas con planificador activo; no representa cuatro recuperaciones exitosas.
- derribos-protomotions-960hz.mp4: siega arriba y entrada abajo; tracker a la izquierda y planificador a la derecha. La etiqueta «planificador activo» no implica un paso ejecutado ni una recuperación aprobada. El vídeo preliminar derribos-protomotions.mp4 se conserva como diagnóstico de la exportación a 240 Hz y no es la entrega definitiva.
- Versiones separadas -x1.mp4/-x025.mp4 y hojas PNG. Se muestran frente y perfil, COM amarillo, carga vertical verde y fuerzas externas registradas en rojo. Las banderas del vídeo resumen el clip completo, incluso antes del instante del fallo.
- Se revisaron las hojas de fotogramas de golpes y proyecciones; no todos los fotogramas manualmente. Todos los MP4 se decodifican completos. El reproductor prohíbe mj_step: no integra ni suaviza poses.

Validación: 61 pruebas nativas correctas; 706 pruebas TypeScript correctas (9 omitidas, 3 pendientes); typecheck, lint y build correctos. El aviso habitual de tamaño de bundle se conserva. La evidencia de física son los datos y vídeos, incluidos sus fallos; CI no certifica estas caídas.

Para reproducir desde este worktree, con las dependencias ya preparadas, usar una carpeta nueva:

~~~powershell
node node_modules/vite/bin/vite.js build --config scripts/controller_lab/scene.vite.config.mjs --configLoader runner
node out/existing-scenarios/exporter/scene-export.mjs scenarios/physical_refs/outer_reap.json out/existing-scenarios/exports-native/outer_reap.json
node out/existing-scenarios/exporter/scene-export.mjs scenarios/physical_refs/leg_drive.json out/existing-scenarios/exports-native/leg_drive.json
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/existing_scenarios.py --output out/existing-scenarios/reproduction
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/render_existing_scenarios.py out/existing-scenarios/reproduction out/existing-scenarios/reproduction-video/golpes.mp4 --kind hits
.\out\native\.venv\Scripts\python.exe scripts/controller_lab/render_existing_scenarios.py out/existing-scenarios/reproduction out/existing-scenarios/reproduction-video/derribos.mp4 --kind throws
.\out\native\.venv\Scripts\python.exe -m unittest discover -s scripts/controller_lab -p "test_*.py"
~~~

La auditoría puede recopilar una carpeta alternativa mediante evidence_existing_scenarios.py --trials <carpeta>. No sobrescribir las grabaciones v1/v2 preservadas.
