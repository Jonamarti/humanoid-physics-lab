# Contribuir

0. Si eres un agente, lee antes la [guía rápida para agentes](docs/GUIA_AGENTES.md) y el [plan de derribos](docs/PLAN_DERRIBOS.md).
1. Revisa la [épica #1](https://github.com/Jonamarti/humanoid-physics-lab/issues/1), elige una issue y comprueba las dependencias.
2. Implementa cambios pequeños con unidades SI y referencias verificables.
3. Ejecuta `npm run typecheck && npm run lint && npm test && npm run build`.
4. Abre una PR enlazando la issue e indicando tests realizados y limitaciones.
5. No subas FBX, BLEND ni otros recursos privados de AnimSpec; tampoco copies código GPL sin revisión de licencias.

## Validación de física
Un test matemático no certifica equilibrio biomecánico. Las futuras pruebas deberán verificar contactos, límites de par, penetraciones, errores numéricos, pasos y tiempos de simulación con timestep fijo. No presentar animación guionizada como simulación física.
