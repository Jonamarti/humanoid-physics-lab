# HPL: motor físico offline para generar animaciones

**Decisión del autor: 16-11-2026. Estado:** propósito y límites arquitectónicos ratificados; las fases del plan conservan su propio estado de implementación. Consultar [PLAN_DERRIBOS](PLAN_DERRIBOS.md) y [ADR de separación](ARCHITECTURE_BOUNDARY.md) antes de codificar.

## Misión

**Humanoid Physics Lab es un laboratorio público de simulación física reproducible** para generar movimientos reutilizables de humanoides y objetos: respuesta a impulsos frontales/laterales, control postural, pasos de recuperación, pérdida de equilibrio, caídas y derribos/contactos entre actores. Su salida útil es una grabación física con métricas/validación y un formato neutral de poses exportable. No es un motor de juego ni una biblioteca artística de técnicas.

Su visor web permite inspeccionar la simulación; la ejecución headless y los datos son la fuente de verdad. El objetivo es producir mejores animaciones con menos animación manual. No hay requisito de ejecutar MuJoCo ni de simular fuerzas en tiempo real en los juegos.

## Propiedad del trabajo

| Sistema | Responsable |
|---|---|
| HPL | MJCF, MuJoCo, contactos, fuerzas/pares limitados, integrador, control físico de Uke y Tori cuando sea necesario, estabilidad, grabación, pruebas de regresión, exportación neutral |
| AnimSpec/Blender (consumidor privado) | Técnica marcial con nombre y fases, pista de Tori guionizada, rigs de personajes, retargeting/IK, ajustes, aprobación visual y exportación a clips de juego |
| Juegos Unity | Detección de impactos, viabilidad, selección contextual, transición y reproducción de clips y parejas con colisiones/cámaras/daño; **sin HPL ni física biomecánica en vivo** |

## Reglas obligatorias

- Simular el movimiento **offline** y grabarlo para que el consumidor lo adapte. Un juego recibe clips ya calculados, no escenarios MuJoCo para ejecutar en la partida.
- HPL no debe conocer `seoi_nage`, `morote_gari`, nombres privados, rigs concretos, FBX/Blend o modelos licenciados. Usar familias mecánicas neutrales.
- No duplicar un motor físico en AnimSpec ni poner un módulo de exportación de rigs de juego en HPL.
- Evitar mover artificialmente raíz/pelvis para esconder inestabilidad; la física debe venir de contactos, efectores y pares internos declarados.
- Calcular movimiento físicamente plausible no basta para aprobar una animación estética: validar pruebas automáticas aquí y revisar la apariencia/contactos finales en el consumidor privado.
- Conservar resultados negativos, parámetros y versiones de los ensayos para comparar.
- No empezar una fase posterior sin verificar dependencias/ramas. Hay PR experimentales y en borrador; seguir el orden y las reglas de [PLAN_DERRIBOS](PLAN_DERRIBOS.md) y [GUIA_AGENTES](GUIA_AGENTES.md).
- Publicación/CI: este proyecto es público. Nunca incorporar clips de personaje, medidas privadas, guiones o contenidos propios del juego.

## Uso previsto

1. Un consumidor prepara un escenario genérico: masa, altura, postura, agarres/efectores, fuerzas, tiempos y resultado esperado.
2. HPL lo simula con reloj fijo, mide estabilidad, pasos, caída, contactos, artefactos y presupuesto de fuerzas.
3. Entrega grabación, telemetría, vídeo/visor y exportación neutral de poses según [#80](https://github.com/Jonamarti/humanoid-physics-lab/issues/80).
4. AnimSpec/Blender convierte la trayectoria a un rig, corrige y valida el clip; Unity solo lo utiliza como un recurso.

Este texto **es una referencia breve de propósito**, no sustituye las especificaciones de las fases [#73–#80](https://github.com/Jonamarti/humanoid-physics-lab/issues/74).
