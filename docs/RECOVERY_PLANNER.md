# Planificación de recuperación — fase 1 (issue #12)

Esta rama **NO hace caminar al humanoide**. Aporta funciones puras y unitarias para
identificar un soporte cargado y proponer, o rechazar, un destino de paso.

## Contrato físico y límites

- `footContacts.ts` recibe **muestras ya transformadas a coordenadas de mundo**
  con la fuerza normal y orientación del contacto. Cuenta solo contactos
  suelo/pie con componente vertical ascendente y los suma por pie.
- `loadedSupportPolygonXY` es la envolvente geométrica de los pies que superan
  un umbral de carga; **no es el polígono real de puntos de contacto ni el CoP**.
  El centroide ponderado es un promedio de posiciones, no integra momentos.
- **Existe adaptador MuJoCo → GroundContactSample** en
  `MujocoSimulation.humanoidGroundSupportObservation()`: se leen contactos
  suelo/pie y fuerzas resueltas `efc_force` (cono piramidal decodificado),
  con liberación de handles WASM. Una prueba de integración compara la
  reacción vertical total con el peso nominal y verifica reset.
  Aún no hay CoP 6D ni validación biomecánica externa; no derivar cargas
  de `data.ncon` o cercanía geométrica.
- El planificador utiliza el capture point de péndulo lineal invertido (LIPM).
  Rechaza inclinaciones acusadas, apoyos inexistentes, pasos fuera de alcance
  o posiciones sin pie de apoyo viable; es deliberadamente conservador.
- `STEP` representa solo **un objetivo propuesto en XY**, NO un paso
  físicamente ejecutado. No se mueve root, geometría ni articulación.
  La transferencia de peso, despegue y aterrizaje requieren el controlador
  de la issue #13 y medición del contacto resultante.
- `FALL` indica **inviabilidad bajo estas reglas**, no prueba biomecánica
  de que una persona deba caer; `NONE` tampoco certifica equilibrio.

## Pruebas y siguientes incrementos

`tests/unit/recoveryPlanner.test.ts` ejercita cargas, signos de normales,
obstáculos de soporte, latencia, límites y captura. Todos los fixtures son
sintéticos y **no constituyen integración real MuJoCo**.

Siguientes entregas sugeridas, en orden:

1. Incorporar la lectura física validada de cargas por pie al panel web,
   y probar la descarga de un pie en un ensayo controlado.
2. Verificar las decisiones del planificador en los empujones reales,
   con histéresis, latencia y contacto por timestep.
3. Contrastar CoP, fuerzas y respuestas contra datasets externos de
   `EXPERIMENTAL_CALIBRATION.md` antes de modificar ganancias.
4. Añadir una FSM de pasos con actuadores internos y contacto medido (#13).

**Importante:** el adaptador ya obtiene carga real del solver en Node/WASM,
pero todavía no la muestra en la web ni controla pasos. El polígono
sigue siendo una aproximación basada en geometría de pies *cargados*.
