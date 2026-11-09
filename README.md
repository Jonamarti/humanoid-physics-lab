# Humanoid Physics Lab

[![CI](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/ci.yml)

Laboratorio público para desarrollar **control clásico** de equilibrio, pasos y caídas articuladas en un humanoide físico, sin aprendizaje automático.

> **Estado actual:** estructura inicial TypeScript/Vite, página web y tests unitarios de matemáticas. **Todavía no hay simulación MuJoCo, humanoide ni equilibrio**: se desarrollarán en [las issues](https://github.com/Jonamarti/humanoid-physics-lab/issues/1). El cálculo de impulso de la pantalla es una demostración de unidades SI; no predice caídas.

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

La ruta base de Vite `/humanoid-physics-lab/` ya está configurada. El despliegue GitHub Pages pertenece a la [issue #6](https://github.com/Jonamarti/humanoid-physics-lab/issues/6).

## Principios

- Simulación física prevista en MuJoCo WASM (#4); render Three.js (#5), no un segundo motor de física.
- Integración de timestep fijo, fuerzas SI, pares articulares limitados; evitar anclajes artificiales a root/pelvis.
- Models públicos sintéticos. **No subir FBX/BLEND propietarios de AnimSpec**.
- No declarar equilibrio validado hasta probar 10 s de simulación física (#9).
- Convenciones en [docs/COORDINATES.md](docs/COORDINATES.md) y pirámide de tests en [docs/TESTING.md](docs/TESTING.md).
- Contribuciones: [CONTRIBUTING.md](CONTRIBUTING.md); licencias de terceros: [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md).

Código original bajo licencia Apache-2.0.
