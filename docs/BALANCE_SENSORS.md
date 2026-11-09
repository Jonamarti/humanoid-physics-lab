# Sensores de equilibrio 3D — primer paso hacia los 10 segundos

## Qué observamos (física, no animación)

`src/control/balanceSensors.ts` calcula magnitudes deterministas a partir de:
- Centro de masas (COM) tridimensional del árbol de cuerpos físicos MuJoCo del humanoide.
- Velocidad del COM con `mj_subtreeVel`.
- Posición y orientación de las dos cajas que representan los pies.
- Polígono convexo proyectado en XY de los **pies geométricamente cercanos al suelo**.
- **Capture point 2D** como aproximación de péndulo invertido: `xi = com_xy + vcom_xy * sqrt(h/g)`.
- Indicadores de COM y punto de captura dentro/fuera de la zona geométrica candidata.

`MujocoSimulation.humanoidBalanceObservation()` lee esas magnitudes del motor MuJoCo WASM real y las muestra en la versión web bajo la altura de pelvis. El render **no modifica** el estado físico ni ejecuta controladores.

## Limitación fundamental

**Los pies proyectados sobre el suelo no equivalen al polígono de contactos efectivo ni al centro de presiones (CoP)**. Un pie muy cercano al suelo puede estar descargado, apoyado solo de puntillas o resbalando. Por tanto, el resultado de `comProjectionInsideCandidateSupport` y el punto de captura **son información de diagnóstico**, no pruebas de que el humanoide sea estable.

El punto de captura se basa en un modelo con COM a altura aproximadamente constante y movimiento del tronco reducido; es menos fiable cuando el muñeco cae, rueda o impacta.

Para certificar los **10 segundos de pie** hacen falta sensores de cargas/contactos reales, pares articulares, límites de fricción, control coordinado de tobillos, caderas y tronco, y un benchmark continuo con tolerancias de COM, penetración y deriva. Issues #8–#9 y #16.

## Tests

- `tests/unit/balanceSensors.test.ts`: hull geométrico, puntos dentro/fuera, capture point y pie elevado.
- `tests/unit/balanceSensorsMujoco.test.ts`: lectura física del centro de masas y las dos huellas sobre el humanoide real en WASM.
- `src/physics/mujoco.ts` resuelve WASM de forma distinta en navegador y Node, sin depender de un path absoluto de Vite durante los tests.

Este cambio no proporciona ni proclama equilibrio activo, pasos o ukemi. La UI del cilindro y sus parámetros permanecen independientes.
