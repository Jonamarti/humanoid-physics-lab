# Visor Three.js para MuJoCo (issue #5)

El visor consume las **posiciones producidas por MuJoCo**; no interpola poses inventadas ni actúa como un motor de física. En esta primera iteración, solo hay una esfera de radio 0,12 m sobre un plano: el modelo humanoide articulado llega en #7.

- MuJoCo: ejes mundiales **Z vertical**. Three.js: **Y vertical**. Convertir `(x, y, z) → (x, z, -y)`.
- Cámara: `OrbitControls` permite giro, zoom y desplazamiento con ratón/táctil.
- Física: el reloj fijo de MuJoCo conserva `dt=0,002 s`; Three.js solo renderiza al ritmo del dispositivo. La posición se actualiza con `qpos[0..2]` del cuerpo libre del escenario de ensayo.
- Render: pixel ratio máximo 2, luces simplificadas y suelo sin texturas privadas. `ResizeObserver` adapta el lienzo al contenedor.
- Memoria: liberar recursos de geometría, materiales, controles, animación y contexto al desmontar.
- Si el contexto WebGL no está disponible, se informa del fallo sin impedir que la prueba de telemetría física se ejecute.
- **No está validada aún la compatibilidad en un Android real ni el deployment Pages**; E2E browser queda para #6/#19.

Las transformaciones de rotación de cuaterniones y la representación del esqueleto humanoide llegarán con el modelo MJCF #7.
