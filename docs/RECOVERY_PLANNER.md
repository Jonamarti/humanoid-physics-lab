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
- **Todavía NO existe adaptador MuJoCo → GroundContactSample**: antes de
  conectarlo hay que comprobar en el runtime WASM 3.17.0 la convención de
  normales `contact.frame`, el estado del solver, `mj_contactForce` y su
  orientación, y contrastar la suma de fuerzas con el peso nominal.
  No derivar cargas de `data.ncon` o cercanía geométrica.
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

1. Integrar `mj_contactForce` / `data.contact` del WASM 3.17.0 con una
   prueba de pie en suelo, de caída libre y de inversión de normal.
2. Incorporar carga real por pie, puntos de contacto y giros de tronco a
   `humanoidBalanceObservation` y registrar por timestep.
3. Verificar decisiones en los escenarios reales de empujón publicados,
   con sensibilidad a latencia, fuerza, rozamiento y variación de altura.
4. Añadir una FSM de pasos con actuadores internos y contacto medido (#13).

**Importante:** esta rama incluye un nuevo cálculo de apoyo geométrico cargado,
pero no lo muestra en la web hasta completar la integración del solver.
