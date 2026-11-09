# Editor avanzado de fuerzas 3D (experimental)

La herramienta web ofrece un modo simple de empujón histórico y un **editor avanzado** independiente. La rama de implementación es `feat/multi-force-editor-glyphs`.

- Añade hasta 12 fuerzas con sitios concretos del MJCF público: esternón, pelvis, hombros y antebrazos. También permite seleccionar un punto arbitrario sobre la geometría visible y guardar su offset local respecto al centro de masas del segmento.
- Fuerzas en newtons mediante componentes **mundiales MuJoCo** `Fx/Fy/Fz`, con +X anterior, +Y izquierda y +Z arriba. Direcciones negativas indican el sentido opuesto.
- Formas temporales: pulso de medio seno y fuerza constante. Cada fuerza tiene inicio y duración independientes. Los parámetros se congelan al ejecutar y se restablecen con **Reiniciar**.
- Suma correctamente los esfuerzos sobre cada cuerpo a través de `xfrc_applied` con momento físico `r × F`. MuJoCo borra los esfuerzos por timestep para que no persistan por accidente. El render no gobierna el estado de la simulación.
- Antes/después de aplicar, orientación visual ámbar; durante el paso con fuerza aplicada, resaltado rojo y halo por fuerza. Los símbolos de clase **⊗ alejándose de la cámara** y **⊙ acercándose a la cámara** aparecen cuando el vector es casi paralelo a la mirada; en los demás casos se usa flecha. Rotar la cámara cambia la notación, nunca la fuerza física.
- La leyenda usa el símbolo en el sentido clásico del dibujo: **⊗ hacia dentro de la pantalla** y **⊙ hacia fuera de la pantalla**.
- El impulso se indica vectorialmente en N·s. No equivale a predecir daño o derribo humano real.

## Limitaciones para una siguiente iteración

1. **Implementado inicialmente:** botón «Elegir punto sobre el cuerpo» para seleccionar la geometría del humanoide por raycasting. Se calcula el anclaje físico local al COM del segmento; permanece sujeto al mismo cuando el cuerpo gira.
2. **Implementado inicialmente:** botón «Editar esta flecha» y arrastre de punta de flecha para cambiar su **dirección** con ratón o dedo. La magnitud en newtons no cambia al arrastrar; se introduce en los campos Fx/Fy/Fz.
3. **Pendiente:** manipulación de magnitud por arrastre, handles específicos por ejes, snapping a ejes y ajustes de accesibilidad/ergonomía móvil.
4. **Pendiente:** guardar/cargar secuencias de fuerzas y anclajes en JSON.
5. **Verificado en CI y navegador** (9 de noviembre de 2026): TypeScript, ESLint, Vitest y compilación en PR #47. Smoke de Chrome/Three.js (rama temporal `feat/force-editor-browser-smoke`) ejecutó dos fuerzas con tiempos independientes y grabó el visor, sin excepciones; tras finalizar el ensayo los controles volvieron a habilitarse. En Chrome táctil de 390 px se activaron y editaron dos fuerzas y se comprobó `document.scrollWidth=390 px`, sin desbordamiento horizontal. [Ejecución y artefactos](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42691480739). La selección por raycasting sobre partes corporales y arrastre de flechas todavía requiere una evaluación táctil manual específica de precisión/usabilidad; los ensayos automatizados no la sustituyen.
6. Física de interacción entre dos humanoides: necesita modelar el contacto explícito y sigue fuera de alcance.

## Comparación con Physics Sandbox

Physics Sandbox está únicamente en el ordenador local del usuario. Su comparación queda aplazada por petición expresa, sin bloquear el desarrollo. Mantener un único motor físico (MuJoCo) en este laboratorio.
