# ADR-0001: propiedad del motor físico e integración con herramientas de animación

**Estado:** propuesta arquitectónica (9 de noviembre de 2026). Esta ADR no implica que el contrato JSON ni el exportador estén implementados.

## Decisión

Humanoid Physics Lab (HPL) es el **único propietario de la simulación física hacia delante** de humanoides articulados y objetos. Su interfaz web Three.js es un laboratorio de observación y ensayo, no el motor de animación. Un consumidor como AnimSpec define la intención y las técnicas y puede incorporar después la trayectoria física a sus propios rigs. HPL no conoce las técnicas marciales por nombre ni depende del consumidor.

### Dentro de HPL

- Modelos físicos sintéticos MJCF: masas, inercias, articulaciones, límites, colisiones, fricción, gravedad y escenarios de objetos.
- Integrador MuJoCo WASM, timestep fijo, estado, caché, sensores, cargas/contactos reales, impulsos, fuerzas y momentos aplicados en cuerpos físicos.
- Control articular clásico limitado (postura, estrategias de equilibrio, transferencia de carga, pasos recuperadores, caída protectora y relajación/ragdoll); decisiones físicas basadas en condiciones medidas, no una coreografía impuesta.
- Restricciones/contactos físicos entre cuerpos, agarres con capacidad/fuerza de ruptura, choques y respuesta a obstáculos: **solo cuando se implementen y prueben**.
- Banco de pruebas reproducible, telemetría y grabaciones con identificador de modelo, parámetros, versión y tolerancias.
- Visor web que **consume** el estado del simulador, sin añadir un segundo solucionador físico ni alterar huesos para ocultar errores.

### Fuera de HPL

- Biblioteca o semántica artística de técnicas, nombres como `seoi_nage`/`osoto_gari`, poses, keyframes canónicos, sincronización de coreografías y autorización visual.
- Blender, `bpy`, rigs Mixamo propietarios, archivos FBX/BLEND, conversiones de deformación de malla, clips finales y edición de curvas de animación.
- Datos de personajes privados, su identidad, sus modelos, su contenido licenciado y recetas concretas. Nunca subirlos a este repositorio público ni incluirlos en tests, ejemplos o artefactos de CI.
- Daño, dolor, lesión, KO y seguridad biomecánica humana (fuera del MVP; ver #20).

### Dependencias y módulos

- `src/physics/`: backend MuJoCo, modelos, fuerzas, colisiones, estados, exportación física.
- `src/control/`: sensores, PD/SPD y máquinas de estados de reacción; ningún acceso a Three.js o Blender.
- `src/renderer/` y `src/main.ts`: visualización y controles de experimentos; no determinan ni corrigen la trayectoria física.
- Los tests del motor deben poder ejecutarse sin UI, y los consumidores deberán poder obtener una grabación también sin navegador cuando exista un comando headless.
- Dependencia unilateral **consumidor de animación -> archivo JSON versionado -> HPL -> grabación JSON -> consumidor**. HPL no importa código ni assets de AnimSpec. No se crea un tercer repositorio compartido hasta que haya necesidad real de varios consumidores.

## Contrato público de intercambio propuesto (NO implementado)

El motor aceptará una descripción neutral de un escenario, no una animación Mixamo. Versionar y validar dos esquemas separados: `hpl.scenario.v1` (entrada) y `hpl.recording.v1` (salida). Rechazar versiones desconocidas y valores no finitos.

**Escenario de entrada mínimo:** identificador de caso, `world_frame`, modelo físico/actores/objetos sintéticos, estado inicial (posición y orientación de raíz, articulaciones), objetivos articulares o modo de control, condiciones de suelo y lista temporal de eventos (`impact`, `force`, `grip`, `release`, etc.). Los eventos aportan `actor`, `body` o articulación semántica, instante, duración, magnitud y dirección y punto con **marco explícito** (`world` o `body_local`). Un objetivo cinemático puede usarse como referencia o control limitado; no debe teletransportar al cuerpo dinámico.

**Grabación de salida mínima:** metadatos de versión de esquema/solver/modelo/parámetros, timestep físico, tasa de muestreo, `actor_id`, tiempo de cada muestra, traslación y cuaternión de raíz, orientaciones locales de articulaciones con identificador semántico, velocidades, contactos y fuerzas reales, estado del controlador, métricas y eventos. Indicar qué canales son *medidos*, *estimados* o *ausentes*. La reproducción de la grabación no debe recalcular físicas distintas.

**Convenciones:** SI (m, kg, s, N, N·m, N·s, rad); mundo diestro +X delante, +Y izquierda, +Z arriba, como `docs/COORDINATES.md`; cuaterniones serializados en orden explícito `wxyz` y frame indicado en cada transform. Three.js hace su conversión Y-up solo al dibujar. El consumidor traduce su marco de personaje/rig a este contrato; una pose de AnimSpec expresa x=izquierda, y=delante, z=arriba en su frame de reposo, y GrappleMap tiene además otra convención. **Nunca hacer un simple intercambio de ejes sin validar orientaciones y handedness**. Calibrar con rig sintético y tests ida/vuelta de rotaciones, lateralidad, pelvis y suelas.

**Privacidad/licencias:** emitir ejemplos genéricos de dominio público o creados expresamente aquí; perfiles de masa e inercias propios de un personaje privado se inyectan localmente desde el consumidor, sin persistirlos ni subirlos a GitHub Pages/Actions.

**Límites operativos:** GitHub Pages es un sitio estático, no un servicio remoto de simulación para Blender. La integración v1 debe funcionar con exportación/importación de ficheros JSON; una CLI Node/headless puede añadirse después. El resultado de un ensayo sintético no certifica biomecánica humana.

## Plan de implementación sin ruptura

1. Conservar `src/physics` y `src/control` actuales y sus pruebas de bipedestación. En #10 formalizar impactos 3D por cuerpo/punto/duración y sus telemetrías; el editor 3D multi-fuerza de la PR #47 sigue en revisión y no constituye todavía una API de intercambio versionada.
2. Los sensores de cargas reales, el planificador de paso y la visualización del COM están integrados en `main` (PR #35 y #44). La PR #45 es **todavía experimental y no fusionada**: sus pruebas muestran transferencia, despegue y aterrizaje medidos pero no un paso estable en la dirección requerida (#13, #49). Conservar controles de fase separados del planificador automático de empujones hasta validarlo.
3. Desarrollar caída protectora articulada (#14), seguida de agarres/contactos entre dos actores o con objetos, siempre con límites de fuerza y contacto físico.
4. Diseñar y validar los esquemas anteriores y grabación/replay/exportación en #17, con tests de serialización, coordenadas, identificadores semánticos, determinismo en el mismo runtime y ausencia de datos privados.
5. Añadir un escenario sintético con golpe/empujón -> reacción de Uke -> grabación -> rig sintético externo. El adaptador Blender y la aprobación visual pertenecen al consumidor.

**Regla para nuevas PR:** si una funcionalidad necesita saber qué es `seoi_nage` o cómo se llama un hueso Mixamo, probablemente no pertenece al núcleo de HPL; si calcula una fuerza, un par, contacto, estabilidad o trayectoria física, sí pertenece a HPL.
