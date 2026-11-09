# Editor avanzado de fuerzas 3D (experimental)

La herramienta web ofrece un modo simple de empujón histórico y un **editor avanzado** independiente. La rama de implementación es `feat/multi-force-editor-glyphs`.

- Añade hasta 12 fuerzas con sitios concretos del MJCF público: esternón, pelvis, hombros y antebrazos. Por ahora no admite puntos arbitrarios sobre la superficie ni arrastrar manejadores en el visor.
- Fuerzas en newtons mediante componentes **mundiales MuJoCo** `Fx/Fy/Fz`, con +X anterior, +Y izquierda y +Z arriba. Direcciones negativas indican el sentido opuesto.
- Formas temporales: pulso de medio seno y fuerza constante. Cada fuerza tiene inicio y duración independientes. Los parámetros se congelan al ejecutar y se restablecen con **Reiniciar**.
- Suma correctamente los esfuerzos sobre cada cuerpo a través de `xfrc_applied` con momento físico `r × F`. MuJoCo borra los esfuerzos por timestep para que no persistan por accidente. El render no gobierna el estado de la simulación.
- Antes/después de aplicar, orientación visual ámbar; durante el paso con fuerza aplicada, resaltado rojo. Los símbolos de clase **⊗ alejándose de la cámara** y **⊙ acercándose a la cámara** aparecen cuando el vector es casi paralelo a la mirada; en los demás casos se usa flecha. Rotar la cámara cambia la notación, nunca la fuerza física.
- La leyenda usa el símbolo en el sentido clásico del dibujo: **⊗ hacia dentro de la pantalla** y **⊙ hacia fuera de la pantalla**.
- El impulso se indica vectorialmente en N·s. No equivale a predecir daño o derribo humano real.

## Limitaciones para una siguiente iteración

1. Manipulación directa sobre el modelo (raycasting, selección de cuerpo, arrastre de origen y punta).
2. Offsets de contacto arbitrarios en coordenadas locales de cada segmento, no solo sites predefinidos.
3. Guardar/cargar y compartir secuencias en JSON.
4. Pruebas E2E de navegador y visuales en móvil. Validar build y tests localmente antes de integrar; no asumir CI disponible.
5. Física de interacción entre dos humanoides: sigue pendiente y necesita tratamiento de contacto explícito.

## Comparación con Physics Sandbox

El repositorio Physics Sandbox no se encontró con ese nombre en la instalación GitHub accesible durante esta revisión. No se ha copiado código sin una auditoría reproducible. Investigar integradores, benchmarks de sistemas rígidos, herramientas de graficación, análisis numérico y métodos de validación tras identificar el repositorio correcto. Mantener un único motor físico (MuJoCo) en este laboratorio.
