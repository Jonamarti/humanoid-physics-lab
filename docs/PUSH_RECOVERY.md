# Empujones físicos y recuperación sin pasos — prototipo MuJoCo

**Estado experimental:** se ha reproducido el escenario nominal de diez segundos de pie y se ha añadido el primer banco de impulsos en esternón. **La recuperación no se debe presentar como biomecánicamente general ni validada con sujetos humanos.** La simulación y el humanoide son sintéticos.

## Protocolo repetible

- Modelo: `public/models/humanoid_v1.xml` (masa nominal 75 kg, libre, sin arnés ni pelvis fija).
- Control postural: `src/control/standingBaseline.ts`; torques internos limitados por articulación; realimentación del centro de masas y velocidad de tobillos.
- Impacto: `sternum_impact`, un **site unido al cuerpo torácico**, no a la raíz. El motor aplica `F` y `τ=(p_site-p_COM_torax)×F` como fuerzas físicas externas; después las limpia en cada paso del integrador. No hay traducción manual del cuerpo ni animación del golpe.
- Perfil: media onda seno `F(t)=F_peak*sin(pi*(t-1.0)/0.12)` N para `1.0 < t < 1.12` s; fuera de ese intervalo, fuerza cero. El impulso ideal `J=2*F_peak*0.12/pi` N·s. Todos los ensayos usan timestep `0.002 s`.
- Direcciones de referencia del mundo: +X delante, -X atrás, +Y izquierda, -Y derecha.
- Horizonte: **6 segundos** de simulación física, con observaciones cada **20 pasos** (`0.04 s`). La fuerza dura 0,12 s; después se deja que el cuerpo responda libremente.

## Matriz exploratoria inicial

El ensayo independiente en GitHub Actions del 9-11-2026 con `tests/unit/standingPushSweep.test.ts` produjo:

| Dirección | 20 N (1.53 N·s) | 60 N (4.58 N·s) | 120 N (9.17 N·s) | 240 N (18.33 N·s) |
|---|---|---|---|---|
| Delante | recupera | recupera | recupera | recupera |
| Atrás | recupera | recupera | recupera | **cae** |
| Izquierda | recupera | recupera | recupera | recupera |
| Derecha | recupera | recupera | recupera | recupera |

**15 de 16** combinaciones recuperan la posición según los siguientes criterios *operacionales*:

- Altura mínima de pelvis **superior a 0,86 m** durante el ensayo.
- Inclinación máxima de la raíz **inferior a 25°**.
- Desplazamiento horizontal final de cada pie **inferior a 2 cm**.
- En el último segundo, centro de masas a menos de **3 cm** de la posición nominal, velocidad horizontal inferior a **0,08 m/s** e inclinación inferior a **8°**, en todas las muestras recogidas.

Estos umbrales son de ingeniería para este banco de ensayos; no son límites fisiológicos universales ni indican protección ante caídas reales. La resolución temporal del registro (40 ms) puede perder picos entre muestras; en futuros ensayos se registrará cada timestep.

El caso `backward, peak=240 N` registró aproximadamente **0,124 m de pelvis mínima**, una inclinación máxima de **87,87°** y unos **45 cm** de deriva de pies. Se clasifica como fallo/caída real. La asimetría entre delante y atrás es observable: **no se debe disimular** aumentando arbitrariamente par de tobillos ni afirmar que el controlador funciona en cualquier condición.

Registro original de la matriz: [GitHub Actions — 16 experimentos](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42623670812). Ese run falló por una aserción numérica en el *endpoint* de un pulso (`~1e-13 N`, fuerza residual flotante); la física de los 16 casos sí se ejecutó y registró. El código posteriormente corrige esa tolerancia y los CI siguientes de la rama pasan.

## Probar desde navegador

1. Abrir [Humanoid Physics Lab](https://jonamarti.github.io/humanoid-physics-lab/).
2. Elegir **Humanoide — equilibrio activo experimental (10 s)** y pulsar **Cargar simulación**.
3. En el panel **Probar un empujón real**, escoger dirección y fuerza.
4. Pulsar **Reiniciar y probar empujón (6 s)**. El mismo motor físico de la web aplica el pulso a `t=1 s` y actualiza las posiciones visibles.
5. Al terminar, observar la altura de pelvis, inclinación, desviación del COM y deriva de pies. Contrastar, por ejemplo, **120 N atrás** frente a **240 N atrás**.

El diagnóstico de navegador muestrea por fotograma renderizado y no tiene la resolución del test CI; sus cifras y veredictos pueden diferir levemente cerca del umbral.

## Seguridad de interpretación y pendientes

- El contador global de contactos `data.ncon` **no equivale a fuerzas normales por pie**; el soporte geométrico tampoco es un centro de presiones medido. La aceptación definitiva de #11 requiere contactos y fuerzas reales por pie.
- La reacción del humanoide es el resultado conjunto del controlador interno y de MuJoCo; no existe una política de pasos. Un impacto excesivo puede producir caída.
- La duración y amplitud dependen de la postura y masa sintéticas y de las constantes actuales. No extrapolar directamente a judo real.
- Falta: refinar el umbral hacia atrás, verificar otras duraciones/masas y convergencia de timestep, almacenar artefactos JSON/CSV, registrar esfuerzos articulares y CoP, introducir variaciones posturales y posteriormente pasos de recuperación físicos.
- El script de prueba de cilindro/obstáculo pre-seoi sigue aislado en [CYLINDER_BENCHMARK.md](CYLINDER_BENCHMARK.md).

## Cómo comprobarlo

```sh
npm ci
npm run typecheck
npm run lint
npm test
npm run build
```

Las pruebas con MuJoCo WASM son reales, corren en GitHub Actions y no requieren Blender, modelos privados ni GPU local. Véanse `tests/unit/standingPush.test.ts`, `standingPushSweep.test.ts`, `standingBackwardThreshold.test.ts` y `standingRuntime.test.ts`.

**Referencia de progreso:** #9 bipedestación nominal, #11 recuperación sin pasos, #13 recuperación con pasos, #16 automatización de benchmarks.
