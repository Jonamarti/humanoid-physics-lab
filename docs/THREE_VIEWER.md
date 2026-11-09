# Visor de física MuJoCo con Three.js (#5 y #7)

La página permite elegir dos escenarios con el mismo motor MuJoCo WASM:

1. **Esfera de gravedad:** cuerpo libre que cae desde 1,5 m y contacta con el suelo.
2. **Humanoide MJCF articulado:** 14 segmentos con articulaciones y pelvis libre, sin motores, control postural ni ukemi; es un **ragdoll pasivo** y se cae al iniciarse. Archivo `public/models/humanoid_v1.xml`.

## Contrato geométrico

- El visor extrae `model.geom_type`, `geom_size` y `geom_rgba` una vez al cargar cada modelo; genera mallas primitivas de esfera (2), cápsula (3), elipsoide (4) y caja (6).
- El plano MuJoCo (0) se dibuja como suelo y cuadrícula Three.js: no forma parte de las geometrías articuladas.
- Cada instante recibe `data.geom_xpos` y `data.geom_xmat` calculados por MuJoCo; **no** interpola poses inventadas ni actúa como otro motor físico.
- Coordenadas: MuJoCo **Z arriba**, Three.js **Y arriba**, con rotación global `Rx(-π/2)` y `(x, y, z) → (x, z, -y)`.
- La cápsula Three.js se alinea de eje local Y a eje local Z para coincidir con la convención del tipo cápsula de MuJoCo.
- La geometría y sus materiales se liberan al cambiar de escenario o cerrar. La cámara usa OrbitControls (ratón/táctil), calidad limitada por DPR y ResizeObserver.
- Las matrices del motor y la malla solo sirven para **visualización**, nunca para actualizar `qpos`, `qvel` ni forzar el humanoide.

## Interfaz web

1. Abrir `https://jonamarti.github.io/humanoid-physics-lab/` (si el workflow Pages más reciente pasó).
2. Elegir **Humanoide — ragdoll pasivo, sin equilibrio**.
3. Pulsar **Cargar simulación**. Si no se descarga el XML, la página muestra el error de red.
4. Pulsar **Iniciar**: el cuerpo de 75 kg cae y se mueve por gravedad/choques; puedes girar y acercar la cámara.
5. **Pausar** y **Reiniciar** permiten observar la caída y volver a la pose inicial.

**Limitaciones:** no hay control activo ni resultados de equilibrio 10 s, stepping, golpe ni ukemi. La calculadora de N·s no inyecta fuerzas físicas. Las pruebas de navegador real en móvil y E2E siguen pendientes; los tests de geometría se ejecutan en CI con el WASM real.

## Tests

- `tests/unit/geometry.test.ts`: enum de primitivas y arrays `geom_xpos` / `geom_xmat` del MuJoCo real.
- `tests/unit/humanoidModel.test.ts`: humanoide físico, masas, articulaciones y movimiento pasivo.
- `tests/unit/coordinates.test.ts`: conversión de ejes.
- `tests/unit/mujoco.smoke.test.ts`: gravedad y contacto de la esfera.

**Próximo paso:** acoplar sensores y actuadores articulares limitados, luego controlar equilibrio sin anclajes y demostrar 10 s de bipedestación (#8–#9).
