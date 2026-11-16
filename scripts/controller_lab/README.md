# Laboratorio nativo de controladores

Windows, Python 3.12, uv, Git LFS y Visual Studio 2022 C++ Build Tools. Todo es CPU. Preparar desde la raíz del repositorio:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/controller_lab/prepare.ps1
```

El script fija versiones y verifica los hashes del SDK y del checkpoint. Descarga únicamente los pesos; **no descarga muestras de movimientos ni acepta accesos a datasets**. No instala CUDA ni servicios de nube. Requiere espacio para PyTorch CPU y las dependencias en `out/native/.venv`.

## Prioridad actual: postura natural y un paso con ProtoMotions

```powershell
$labPython = Resolve-Path out/native/.venv/Scripts/python.exe
& $labPython scripts/controller_lab/natural_step.py
& $labPython scripts/controller_lab/analyse_natural.py
& $labPython scripts/controller_lab/render_natural.py
```

Salida actual: `out/protomotions-natural-v3/`. Vídeo: `delivery/paso-natural.mp4`, frente/perfil, ×1 y después ×0,25. La medición autoritativa del despeje utiliza toda la geometría de la suela y dedos en `physical-review.json`. Se conservan separados el ciclo físico observado, el seguimiento exacto del objetivo y los artefactos.

Las referencias son objetivos cinemáticos calculados por IK, no animaciones que se escriben en el cuerpo simulado. El tracker genera acciones y MuJoCo integra el cuerpo con pelvis libre y actuadores limitados. El paso actual es voluntario y guiado; no es aún marcha general ni recuperación reactiva.

## Comparación inicial reproducible

```powershell
& $labPython scripts/controller_lab/benchmark.py --prepare
& $labPython scripts/controller_lab/benchmark.py --controller pd
& $labPython scripts/controller_lab/benchmark.py --controller mjpc
& $labPython scripts/controller_lab/benchmark.py --controller protomotions
& $labPython scripts/controller_lab/compare.py
& $labPython scripts/controller_lab/render.py
```

Antes de cambiar una referencia o un controlador, guardar el conjunto anterior en otra carpeta de resultados. `--reference-file` y `--output-dir` permiten ensayos separados. No sobrescribir una matriz publicada para hacerla pasar. Las versiones v1/v2/v3 de postura natural permanecen separadas de la comparación original.

Resultados y vídeos iniciales: `out/controller-comparison/`. El comparador rechaza cambios de modelo, referencia, estado inicial, motor, tiempo o impulso. La referencia PD directa es específica de este laboratorio; no representa al controlador de Uke trasladado a otro esqueleto.

## Dos humanoides

```powershell
& $labPython scripts/controller_lab/contacts.py
& $labPython scripts/controller_lab/render.py --contacts
```

Ensayos de contacto, no técnicas completas. En el caso con contactos, el equilibrio aprendido observa cada actor mediante una transformación rígida a su marco de observación; las copias solo hacen cinemática/lectura. Solo el modelo compartido ejecuta pasos físicos. Los objetivos de ciertos miembros se combinan con el equilibrio aprendido, siempre mediante actuadores. El control negativo reproduce los mismos comandos con contactos cruzados desactivados.

## Integridad y límites

```powershell
& $labPython scripts/controller_lab/test_lab.py
```

Se guarda un modelo binario exacto para reproducir el estado compilado. `mj_saveLastXML` por sí solo perdía el enum del sesgo PD modificado en ejecución; el exportador conserva también un XML con esos tipos explícitos y comprueba la recarga. El render nunca vuelve a simular.

52 articulaciones y 42,47 kg en el modelo publicado, con motores de ingeniería. No es intercambiable con Uke, ni valida anatomía o impactos. Ver [resultados y licencias](../../docs/CONTROLLER_COMPARISON_20261116.md). Las muestras del dataset de entrenamiento tienen condiciones propias y no forman parte del laboratorio reproducible.

## Marcha alterna y recuperación (16/11/2026)

Ver [informe de marcha](../../docs/PROTOMOTIONS_GAIT_20261116.md) y `gait_suite.py`. El adaptador ahora fuerza `agent.eval()`; las comparaciones anteriores son históricas y no deben mezclarse numéricamente con esta inferencia. Las nuevas referencias pasan una puerta de alcance IK antes de ejecutarse.

El mejor caso encadena seis pasos a 0,51 m/s y termina estable; quedan 2,38 cm de traslación de pie cargado y marcha muy lenta irregular. La batería de 48 impactos analíticos no valida todavía ninguna recuperación con paso sin artefactos. El planificador de recuperación y los residuos de apoyo son experimentales, no valores por defecto del núcleo. Los vídeos son diagnósticos para revisión humana.
