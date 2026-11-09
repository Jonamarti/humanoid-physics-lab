# Humanoid Physics Lab

[![CI](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml)

Laboratorio público para desarrollar **control clásico** de equilibrio, pasos y caídas articuladas en un humanoide físico, sin aprendizaje automático.

> **Estado actual:** estructura TypeScript/Vite y experimento de MuJoCo WASM y Three.js (esfera libre bajo gravedad, cámara 3D, pausa y reinicio). **Todavía no hay humanoide ni controlador de equilibrio**: se desarrollarán en [las issues](https://github.com/Jonamarti/humanoid-physics-lab/issues/1). La calculadora de impulso es solo una demostración de unidades SI y no predice caídas.

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

La ruta base de Vite `/humanoid-physics-lab/` ya está configurada. El workflow para GitHub Pages (`.github/workflows/pages.yml`) compila y despliega automáticamente **cuando GitHub Pages esté activado** en **Settings → Pages → Source: GitHub Actions**. Consulta la [guía de activación y verificación](docs/PAGES.md). La URL prevista es <https://jonamarti.github.io/humanoid-physics-lab/>; no se considera publicada hasta que el job `Publish site` finalice correctamente.

## Principios

- Simulación física con MuJoCo WASM (#4), modelo básico de esfera; render Three.js integrado (#5) como visualizador, sin segundo motor de física.
- Integración de timestep fijo, fuerzas SI, pares articulares limitados; evitar anclajes artificiales a root/pelvis.
- Models públicos sintéticos. **No subir FBX/BLEND propietarios de AnimSpec**.
- No declarar equilibrio validado hasta probar 10 s de simulación física (#9).
- Convenciones en [docs/COORDINATES.md](docs/COORDINATES.md) y pirámide de tests en [docs/TESTING.md](docs/TESTING.md).
- Contribuciones: [CONTRIBUTING.md](CONTRIBUTING.md); licencias de terceros: [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md).

Código original bajo licencia Apache-2.0.

## Runtime MuJoCo WASM

El ensayo de gravedad se activa desde la página pulsando **Cargar simulación** y luego **Iniciar**. Muestra la altura real calculada por MuJoCo, sin visualizador 3D. Más información técnica en [docs/MUJOCO_RUNTIME.md](docs/MUJOCO_RUNTIME.md). No es un humanoide ni se garantiza una tasa de 60 FPS en dispositivos móviles.

## Visor Three.js

El escenario de gravedad se observa desde una cámara orbital controlable con ratón o gestos táctiles. El objeto visual recibe su posición de MuJoCo; no es una animación prefabricada. Ver [docs/THREE_VIEWER.md](docs/THREE_VIEWER.md). El cuerpo humanoide articulado y el control de equilibrio continúan pendientes (#7–#9).
