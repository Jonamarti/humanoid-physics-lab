# Humanoid Physics Lab

[![CI](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml)

Laboratorio público para desarrollar **control clásico** de equilibrio, pasos y caídas articuladas en un humanoide físico, sin aprendizaje automático.

> **Estado actual:** estructura TypeScript/Vite y experimento de MuJoCo WASM y Three.js (esfera libre bajo gravedad, cámara 3D, pausa y reinicio). **Ya incluye un humanoide MJCF sintético de 75 kg y un visor articulado**, pero **no tiene controlador de equilibrio**: el cuerpo es un ragdoll pasivo, que caerá físicamente. El equilibrio durante 10 s se desarrollará en [las issues #8–#9](https://github.com/Jonamarti/humanoid-physics-lab/issues/9). La calculadora de impulso es solo una demostración de unidades SI y no predice caídas.

## Empezar

Necesitas Node.js 22 LTS y npm 10+.

```sh
npm install
npm run dev
```

Para verificar:

```sh
npm run typecheck
npm run lint
npm test
npm run build
npm run preview
```

## CI, dependencias y publicación

GitHub Actions ejecuta comprobación de tipos, ESLint, Vitest y compilación en cada push y pull request. El archivo `package-lock.json` está confirmado y el workflow utiliza `npm ci` y caché npm para instalaciones reproducibles. El lockfile se generó inicialmente en un runner de GitHub conectado al registro npm.

La ruta base de Vite `/humanoid-physics-lab/` ya está configurada. El workflow para GitHub Pages (`.github/workflows/pages.yml`) compila y despliega automáticamente **cuando GitHub Pages esté activado** en **Settings → Pages → Source: GitHub Actions**. Consulta la [guía de activación y verificación](docs/PAGES.md). El segundo intento de publicación de Pages terminó con éxito el 9-11-2026. Entra en <https://jonamarti.github.io/humanoid-physics-lab/>. Para la verificación de publicaciones posteriores, consulta el job `Publish site` de Actions.

## Principios

- Simulación física con MuJoCo WASM (#4), esfera de prueba y humanoide MJCF sin motores; render Three.js de geometrías físicas y cámara orbital. Sin segundo motor de física.
- Integración de timestep fijo, fuerzas SI, pares articulares limitados; evitar anclajes artificiales a root/pelvis.
- Models públicos sintéticos. **No subir FBX/BLEND propietarios de AnimSpec**.
- No declarar equilibrio validado hasta probar 10 s de simulación física (#9).
- Convenciones en [docs/COORDINATES.md](docs/COORDINATES.md) y pirámide de tests en [docs/TESTING.md](docs/TESTING.md).
- Contribuciones: [CONTRIBUTING.md](CONTRIBUTING.md); licencias de terceros: [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md).

Código original bajo licencia Apache-2.0.

## Runtime MuJoCo WASM

Escoge **Esfera** o **Humanoide (ragdoll sin equilibrio)** y pulsa **Cargar simulación** y después **Iniciar**. El visor Three.js muestra las transformaciones calculadas por MuJoCo, no una animación prefabricada. Más información técnica en [docs/MUJOCO_RUNTIME.md](docs/MUJOCO_RUNTIME.md). El humanoide es solo una base física pasiva: **no ha aprendido a estar de pie ni a dar pasos**. No se garantiza una tasa de 60 FPS en dispositivos móviles.

## Visor Three.js

El escenario de gravedad se observa desde una cámara orbital controlable con ratón o gestos táctiles. El objeto visual recibe su posición de MuJoCo; no es una animación prefabricada. Ver [docs/THREE_VIEWER.md](docs/THREE_VIEWER.md). El cuerpo humanoide articulado ya está implementado y se visualiza como ragdoll pasivo; el controlador de equilibrio y la recuperación con pasos continúan pendientes (#8–#9, #12–#13).
