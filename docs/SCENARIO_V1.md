# Escenario `hpl.scenario.v1`

Referencia del formato de escenario (issue #74, PR 2). Tipos: `src/scenario/schema.ts`.
Validador: `src/scenario/validate.ts` (`validateScenario(json, { fileName? })`).

Convenciones: unidades SI (m, kg, s, N, grados donde se indica); ejes X adelante, Y izquierda, Z arriba;
claves en inglés y `snake_case`. **Se rechaza cualquier campo desconocido** y cualquier número no finito.
El validador se detiene en el primer fallo y lanza `ScenarioValidationError` con la ruta del campo y el motivo,
por ejemplo `effectors[1].force_n[0]: debe ser un número finito, no NaN`.

## Ejemplo completo

```json
{
  "schema": "hpl.scenario.v1",
  "id": "cylinder_obstacle_low",
  "duration_s": 3.0,
  "record_hz": 125,
  "ground": { "friction": 0.6 },
  "actors": [
    { "id": "uke", "kind": "rigid_cylinder", "mass_kg": 75, "height_m": 1.75, "radius_m": 0.18 }
  ],
  "props": [
    { "id": "obstacle", "kind": "fixed_box", "size_m": [0.06, 0.4, 0.05], "pos_m": [0.26, 0, 0.05] }
  ],
  "effectors": [
    { "id": "pull", "kind": "prescribed_force", "actor": "uke",
      "body": "cylinder", "local_point_m": [0, 0, 0.525],
      "frame": "world", "force_n": [600, 0, 180],
      "start_s": 0.3, "duration_s": 0.15, "waveform": "half_sine" }
  ],
  "expect": { "outcome": "fall", "direction_deg": 0, "tolerance_deg": 30 }
}
```

## Campos

### Raíz

| Campo | Tipo | Unidad | Regla |
|---|---|---|---|
| `schema` | cadena | | Exactamente `hpl.scenario.v1`. Se comprueba antes que el resto. |
| `id` | cadena | | Solo `[a-z0-9_]`, no vacía. Si se pasa `fileName` al validador, igual al nombre del fichero sin `.json`. |
| `duration_s` | número | s | > 0 y <= 30. |
| `record_hz` | número | Hz | Entre 30 y 500 y divisor exacto de la frecuencia física (500 Hz): 50, 100, 125, 250, 500, etc. |
| `ground` | objeto | | Obligatorio. |
| `actors` | lista | | Al menos un actor. |
| `props` | lista | | Puede estar vacía. |
| `effectors` | lista | | Puede estar vacía. |
| `expect` | objeto | | Opcional. |

Los `id` de actores, props y efectores siguen la misma regla `[a-z0-9_]` y no pueden repetirse dentro de su lista.

### `ground`

| Campo | Unidad | Regla |
|---|---|---|
| `friction` | adimensional | >= 0. Rozamiento deslizante. |

### `actors[]` (`kind: "rigid_cylinder"`)

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | Ver arriba. |
| `kind` | | `rigid_cylinder` (único tipo en esta fase). |
| `mass_kg` | kg | > 0. |
| `height_m` | m | > 0. |
| `radius_m` | m | > 0. |

Cuerpos referenciables por los efectores: `rigid_cylinder` tiene `cylinder`.

### `props[]` (`kind: "fixed_box"`)

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | Ver arriba. |
| `kind` | | `fixed_box` (único tipo en esta fase). |
| `size_m` | m | Tres aristas `[x, y, z]`, todas > 0. |
| `pos_m` | m | Centro `[x, y, z]` en el mundo. |

### `effectors[]` (`kind: "prescribed_force"`)

| Campo | Unidad | Regla |
|---|---|---|
| `id` | | Ver arriba. |
| `kind` | | `prescribed_force` (único tipo en esta fase). |
| `actor` | | `id` de un actor existente. |
| `body` | | Cuerpo existente de ese actor. |
| `local_point_m` | m | Punto de aplicación `[x, y, z]` en el sistema local del cuerpo. |
| `frame` | | `world`, `actor_initial` (ejes del actor en t=0) o `actor_current`. |
| `force_n` | N | Fuerza `[x, y, z]` expresada en `frame`. Valor máximo de la forma de onda. |
| `start_s` | s | >= 0, desde el comienzo del escenario (tras el asentamiento). |
| `duration_s` | s | > 0 y <= 30. |
| `waveform` | | `half_sine` o `constant`. |

### `expect`

| Campo | Unidad | Regla |
|---|---|---|
| `outcome` | | `standing`, `tilted` o `fall`. |
| `direction_deg` | grados | Opcional. Solo con `outcome: "fall"`. 0 = +X, 90 = +Y. Exige `tolerance_deg`. |
| `tolerance_deg` | grados | Opcional, solo junto a `direction_deg`. > 0 y <= 180. |

## Notas y límites

- **`record_hz` y el ejemplo de la issue #74.** El ejemplo de la issue usa `record_hz: 120`, pero su propia regla
  exige que `record_hz` sea divisor exacto de 500 Hz, y 500/120 no es entero. El validador aplica la regla
  (rechaza 120) y el ejemplo de este documento usa 125. Si se prefiere admitir 120 Hz habrá que cambiar la regla
  (por ejemplo, muestrear al primer paso físico posterior a cada instante de grabación) y decidirlo aparte.
- Reglas añadidas por esta PR más allá de la lista de la issue, por coherencia: unicidad de `id` dentro de cada
  lista, al menos un actor, `ground.friction` >= 0, `start_s` >= 0, aristas > 0 y las reglas de `expect` de
  arriba. Los valores admitidos de `waveform` distintos de `half_sine` (`constant`) también son una elección de
  esta PR.
- No se comprueba que el efector termine antes del fin del escenario ni que los props no se solapen con los actores.
- Las fases siguientes añadirán tipos de actor, efector y prop; entonces se ampliarán `schema.ts`, `ACTOR_BODIES`
  y este documento.
