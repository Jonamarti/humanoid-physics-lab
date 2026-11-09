# Primer controlador de bipedestación activa: diez segundos sin empujones

**Estado de validación:** se han superado los diez segundos de prueba en MuJoCo WASM usando el modelo público `humanoid_v1.xml`, desde la postura inicial nominal. **NO** implica que el humanoide recupere el equilibrio ante empujones, camine o sea biomecánicamente fiel a una persona real.

## Origen del resultado

Se partió de un control PD básico sobre articulaciones, respetando límites de par sin fijar la pelvis. La prueba de diez segundos **falló**: con rigidez inicial `scale=1` el personaje había caído a los 2 segundos y la pelvis terminaba a unos 14 cm del suelo. El controlador estaba en saturación gran parte del tiempo.

Se realizaron ensayos parametrizados de tres segundos variando la ganancia articular (`scale=0.1/0.25/0.5/1`) y la corrección por COM:
- Reducir a la mitad las ganancias articulares permitió tres segundos sin caída en el caso nominal.
- Con la misma escala 0.5, una corrección `Kp=1200 N/m`, `Kd=300 N·s/m` para los ejes horizontal/sagital dio oscilaciones muy pequeñas.
- Se fijó esa configuración como **línea base experimental**, a la espera de perturbaciones y calibración con masas diferentes.

## Mecánica implementada

- Cada paso físico de `0.002 s` actualiza sensores del estado MuJoCo, incluido `mj_subtreeVel` para la velocidad del centro de masas.
- Un controlador PD mantiene los ángulos articulares de referencia: `τ=Kp(q_ref-q)-Kd qdot`, saturado por grupo de articulaciones.
- Dos tobillos aplican una corrección de momento basada en el centro de masas, velocidad y brazo de gravedad respecto al apoyo:
  - `τ_sagital = m*g*(comX-ankleX)+kpX*(comX-refX)+kdX*vcomX`
  - `τ_lateral = -[m*g*(comY-ankleY)+kpY*(comY-refY)+kdY*vcomY]`
  - Cada par se distribuye entre dos articulaciones internas de tobillos y se limita a `±160 N·m`. Los seis grados libres de la pelvis se mantienen sin esfuerzos externos.
- Los límites del PD de caderas, rodillas, tronco, cuello y brazos también se respetan.
- **No hay fuerzas dirigidas sobre el root, soldaduras de pies ni teletransporte de geometría**. Las fuerzas normales son producto de contactos de MuJoCo.
- Las constantes son experimentales y específicas del modelo 75 kg, no valores de fuerza/rigidez muscular universal.

## Aceptación que SÍ se ha comprobado automáticamente

- Simulación de 5000 pasos, duración **10 s**.
- Altura inicial de pelvis: `0.990 m`, final `0.989 m`, altura mínima aproximadamente `0.989 m`.
- Inclinación máxima de la raíz: `0.061°` en el ensayo nominal.
- Sin valores NaN y sin pares aplicados a los seis DoF de la pelvis.
- `tests/unit/standing10s.test.ts`: comprueba altura, inclinación y límites de raíz durante diez segundos.
- `tests/unit/standingRuntime.test.ts`: ejecuta el controlador a través de la misma API del visor y comprueba estabilidad de pelvis, deriva de los dos pies, diferencia respecto a ragdoll pasivo y reproducibilidad al reiniciar.

Resultados **solo aplicables al entorno de ejecución versionado**; repetir siempre la prueba después de modificar geometrias, integrador, fricción o controlador.

## Cómo verlo en GitHub Pages

1. Abrir https://jonamarti.github.io/humanoid-physics-lab/
2. En «Escenario físico» elegir **Humanoide — equilibrio activo experimental (10 s)**.
3. Pulsar «Cargar simulación», luego «Iniciar».
4. Observar el tiempo, altura de la pelvis, centro de masas y cámara 3D. Al alcanzar 10 segundos, la ejecución se pausa e informa de la prueba nominal.
5. Cambiar a «Humanoide — ragdoll pasivo» y comparar cómo cae sin el controlador.

## Qué NO está resuelto

- Control posicional/rotacional ante impactos externos, pelvis y tronco bajo gran momento angular.
- El polígono de apoyo actual procede de geometría próxima al suelo: **todavía no mide fuerza normal real por pie ni COP efectivo**.
- La prueba nominal no demuestra equilibrio con velocidad inicial, empujones laterales, cambios de terreno, variación de masas o retardo muscular.
- Paso de recuperación, protección de cabeza, caída controlada y ukemi siguen pendientes.

**Próximas pruebas físicas:** aplicar pequeños impulsos en pecho desde cuatro direcciones con diferentes duraciones, registrar tiempo de retorno de COM y límites de torque, y construir umbrales de estabilidad sin pasos. Solo después planificar recuperación con un paso.

Este módulo tiene carácter experimental; no se presenta como técnica comprobada de caída segura en seres humanos.
