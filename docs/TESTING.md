# Estrategia de pruebas

## Unitarias (implementadas)
`npm test` — Vitest: funciones matemáticas puras para impulso y límites. `npm run typecheck` — TS estricto. `npm run lint` — ESLint. `npm run build` — Vite. **Nada de esto demuestra equilibrio físico.**

## Integración física (pendiente: #4, #7–#9)
Ejecutar WASM MuJoCo real en Chromium headless o Node compatible; timestep fijo, caída/gravedad, contactos, reseteo y manejo de memoria. No confundir mocks con simulación real. Criterios físicos a registrar: centro de masas, torques, penetración de suelo, contactos, oscilación y recuperación.

## Matriz de ensayos (pendiente: #16)
Empujones frontal, posterior, lateral, diagonal, con varias magnitudes, puntos y duración, con reproducibilidad dentro de tolerancias y diagnósticos JSON/CSV. Clasificar mantenerse / recuperar con paso / caer.

## E2E visual (pendiente: #5, #6 y #15)
Playwright con assets servidos desde base `/humanoid-physics-lab/`; comprobar carga del WASM y ausencia de errores, controles y vista móvil. Contrastar posteriormente en Android real.

## Comprobación de mutación
Probar que un fallo deliberado y reversible hace fallar tests críticos; no guardar el fallo en main.
