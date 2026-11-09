# Humanoid Physics Lab

[![CI](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml)

Laboratorio público para desarrollar **control clásico** de equilibrio, pasos y caídas articuladas en un humanoide físico, sin aprendizaje automático.

> **Estado actual:** estructura TypeScript/Vite y experimento de MuJoCo WASM y Three.js (esfera libre bajo gravedad, cámara 3D, pausa y reinicio). **Incluye un humanoide MJCF sintético de 75 kg** con dos escenarios: ragdoll pasivo y **primer control activo de tobillos/COM**, cuya postura inicial ha superado una simulación real de **10 segundos sin empujones**. **Ya existe un banco de 16 empujones físicos** en cuatro direcciones, con recuperaciones y fallos medidos, pero todavía faltan análisis reales de cargas de apoyo, perturbaciones amplias y pasos en [#9](https://github.com/Jonamarti/humanoid-physics-lab/issues/9), [#11](https://github.com/Jonamarti/humanoid-physics-lab/issues/11) y [#13](https://github.com/Jonamarti/humanoid-physics-lab/issues/13). La calculadora de impulso es solo una demostración de unidades SI y no predice caídas.

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

- Simulación física con MuJoCo WASM (#4), esfera de prueba, cilindro contra obstáculo, humanoide pasivo y controlador activo experimental. Render Three.js de geometrías físicas y cámara orbital, sin segundo motor de física.
- Integración de timestep fijo, fuerzas SI, pares articulares limitados; evitar anclajes artificiales a root/pelvis.
- Models públicos sintéticos. **No subir FBX/BLEND propietarios de AnimSpec**.
- Diez segundos de bipedestación nominal superados en CI con pelvis libre y pares internos limitados; faltan pruebas de perturbaciones, otras masas, condiciones de contacto y pasos (#9/#11/#13).
- Convenciones en [docs/COORDINATES.md](docs/COORDINATES.md) y pirámide de tests en [docs/TESTING.md](docs/TESTING.md).
- Contribuciones: [CONTRIBUTING.md](CONTRIBUTING.md); licencias de terceros: [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md).

Código original bajo licencia Apache-2.0.

## Runtime MuJoCo WASM

Escoge **Esfera**, **Humanoide pasivo**, **Humanoide — equilibrio activo experimental (10 s)** o **Cilindro + obstáculo**, y pulsa **Cargar simulación** y después **Iniciar**. El visor Three.js muestra las transformaciones calculadas por MuJoCo, no una animación prefabricada. Más información técnica en [docs/MUJOCO_RUNTIME.md](docs/MUJOCO_RUNTIME.md). El humanoide dispone de un modo pasivo y otro de bipedestación con control clásico; en este último puedes lanzar un empujón desde cuatro direcciones y observar la respuesta física. La matriz de pruebas de 6 s registra **15 recuperaciones de 16 casos**, con caída posterior a 240 N y un umbral observado entre 160 y 180 N. Consulta [docs/PUSH_RECOVERY.md](docs/PUSH_RECOVERY.md). **No está validada la recuperación general ni los pasos**. No se garantiza una tasa de 60 FPS en dispositivos móviles.

## Visor Three.js

El escenario de gravedad se observa desde una cámara orbital controlable con ratón o gestos táctiles. El objeto visual recibe su posición de MuJoCo; no es una animación prefabricada. Ver [docs/THREE_VIEWER.md](docs/THREE_VIEWER.md). El visor permite contrastar ragdoll pasivo con una bipedestación activa durante diez segundos, documentada en [docs/STANDING_10S.md](docs/STANDING_10S.md). Los empujones suaves sin pasos ya tienen pruebas reproducibles, pero la validación de cargas/contactos y la recuperación mediante pasos continúan pendientes (#9, #11–#13).
