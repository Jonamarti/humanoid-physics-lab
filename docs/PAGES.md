# Publicación de Humanoid Physics Lab en GitHub Pages

URL prevista: **https://jonamarti.github.io/humanoid-physics-lab/**

## Activación inicial del propietario (una sola vez)

1. En GitHub, abre el repositorio y entra en **Settings → Pages**.
2. En **Build and deployment → Source**, elige **GitHub Actions**. Si todavía no aparece un workflow de Pages, primero fusiona la PR que añade `.github/workflows/pages.yml`.
3. En **Actions → Deploy GitHub Pages** abre la ejecución de `main`; también se puede lanzar manualmente desde **Run workflow**.
4. Comprueba el estado de `Build` y `Publish site`; el entorno `github-pages` muestra la URL exacta tras un deploy satisfactorio.

Hasta que GitHub indique un deploy satisfactorio, **no debe afirmarse que la página está publicada**, incluso aunque el build Vite haya pasado.

## Cómo se publica

- Cada push a `main` de una persona o integración autorizada inicia `.github/workflows/pages.yml`.
- Si una PR la fusiona automáticamente el workflow con `GITHUB_TOKEN`, GitHub **no dispara otro workflow por `push`**. Por eso el workflow `auto-merge.yml` envía **`repository_dispatch` (evento `pages-after-auto-merge`)** solo después de la fusión verificada; `pages.yml` escucha ese evento, vuelve a comprobar tests y publica el `main` más reciente.
- El job de build ejecuta `npm ci`, TypeScript, ESLint, Vitest y Vite; cualquier fallo impide el despliegue.
- El segundo job utiliza `actions/deploy-pages` con `pages:write` e `id-token:write`; solo despliega el artefacto `dist` del primer job.
- Vite utiliza `base: '/humanoid-physics-lab/'` para que JavaScript y el módulo MuJoCo WASM carguen desde la ruta correcta.
- Se utiliza **MuJoCo single-thread**. No se requieren COOP/COEP para la primera demo.

## Diagnóstico de errores frecuentes

| Síntoma | Comprobación |
|---|---|
| Página 404 | Comprobar Settings → Pages, rama `main`, job `Publish site` y URL exacta. |
| Pantalla en blanco | Abrir consola del navegador; mirar errores de módulos JS, rutas de assets o WebGL. |
| WASM no carga | Comprobar petición al asset `mujoco.wasm`, su estado HTTP, origen y base de Vite. |
| Builds rojos | Consultar logs de `Verify and build the public website`; reproducir con `npm ci && npm run check`. |
| No se ve el humanoide | Es normal en la primera demo: empieza por esfera gravitatoria; el humanoide y control 10s son #7–#9. |

## Validación

Una vez que Pages esté activo, verificar en navegador real y viewport móvil:

1. El título y el contenido cargan sin errores.
2. **Cargar simulación** inicia MuJoCo real.
3. **Iniciar** cambia la altura de la esfera bajo gravedad; el cuerpo reposa sobre el suelo.
4. **Pausar** detiene el tiempo simulado; **Reiniciar** restaura la posición.
5. Si el visor Three.js (#5) está integrado, órbita/zoom funciona con dedo o ratón.

Esta lista es una prueba de humo; no implica que #7–#9 ni equilibrio humano estén terminados.
