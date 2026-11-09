# Política de fusión automática

Este repositorio utiliza [`.github/workflows/auto-merge.yml`](../.github/workflows/auto-merge.yml) para integrar en `main` las PRs de desarrollo **solo cuando** se cumplen todas las condiciones siguientes:

1. La PR está **abierta**, su rama de destino es `main` y **no** está en modo **Draft**.
2. No contiene la etiqueta **`no-merge`** (insensible a mayúsculas).
3. Proviene de una rama **del propio repositorio** (no de forks externos).
4. No modifica archivos bajo `.github/workflows/`. Los cambios de automatización exigen fusión humana.
5. El **último commit exacto** de la rama tiene un workflow **CI** de tipo **pull_request**, terminado con **success**, y el job obligatorio `Typecheck, lint, tests and build` también está correcto.
6. GitHub confirma que no existen conflictos de integración y que la rama sigue apuntando al mismo commit revisado.

El sistema utiliza `squash` para mantener el historial limpio. No se fusionan PRs con CI pendiente, fallido o cancelado. Los checks para cada revisión se evalúan otra vez si cambia el commit.

## Cómo bloquear la fusión

- **Borrador:** en la PR, selecciona **Convert to draft**. No se fusionará aunque el CI sea verde. Cuando quieras habilitarla, pulsa **Ready for review**.
- **Etiqueta:** añade **`no-merge`** a la PR. Puedes dejarla en estado normal de revisión, pero seguirá bloqueada. Para permitir fusionarla tras pasar el CI, quita la etiqueta.

La etiqueta la crea el workflow cuando se instala por primera vez en `main`, si no existe. No requiere activar el auto-merge nativo del repositorio.

## Qué ocurre cuando falla

El flujo de auto-merge dejará la PR abierta, con el registro visible en [GitHub Actions](https://github.com/Jonamarti/humanoid-physics-lab/actions/workflows/auto-merge.yml). Si una PR no se fusiona, verifica CI, Draft, etiquetas, origen de la rama, conflictos y si modifica workflows.

**Limitación:** esta automatización no reemplaza reglas de protección de ramas ni una revisión de seguridad. La propia política modifica el repositorio por GitHub Actions con permisos `contents:write` y `pull-requests:write`; los cambios a workflows se excluyen por ese motivo. Para proyectos públicos abiertos a desconocidos, conservar la restricción de forks.

## Verificaciones realizadas

- PR #21: fusionada manualmente para instalar la política (modificaba workflows).
- PR #22: MuJoCo WASM, CI verde, fusionada **automáticamente**.
- PR #23: Three.js, permaneció Draft durante desarrollo; marcada Ready tras CI verde y fusionada **automáticamente**.
- PR #25: prueba de la etiqueta `no-merge` sobre una PR documental; su historial y las ejecuciones de Actions sirven como evidencia del comportamiento.
