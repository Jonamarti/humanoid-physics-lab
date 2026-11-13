# Diagnóstico aislado del sentido de giro de Uke

**Entrada web:** `/throw-axis-lab.html` (en desarrollo: `http://localhost:5173/throw-axis-lab.html`; en Pages: `https://jonamarti.github.io/humanoid-physics-lab/throw-axis-lab.html`). También hay un enlace desde la página principal.

## Objetivo y aislamiento

Comparar visualmente la trayectoria original de una técnica guionizada con una hipótesis de inversión del giro horizontal de Uke, manteniendo exactamente iguales las articulaciones de Tori. No cambia `src/scenario/`, `src/uke/`, `src/effectors/`, ni las grabaciones o el motor de MuJoCo. Es un ensayo **cinemático** en navegador, no una simulación dinámica; no modifica ningún fichero fuente tras importar.

El repositorio HPL es público y AnimSpec contiene material privado: **no subir aquí JSON reales, FBX, BLEND ni vídeos de AnimSpec**. Los presets integrados son aproximaciones esquemáticas declaradas, inspiradas solo en duraciones y posiciones generales; no son renders ni reproducciones exactas de los rigs. Los modelos visualizados son esqueletos de 23 puntos y no las mallas Mixamo.

## Comparar con Uke y Tori EXACTAMENTE como están en AnimSpec

En el checkout local de AnimSpec que contiene la PR #99:

```sh
python scripts/guion_derribo.py tai_otoshi /tmp/tai-original.json
python scripts/guion_derribo.py o_soto_gari /tmp/osoto-original.json
```

En Windows, cambiar las rutas `/tmp/...` por rutas locales válidas.

Abrir HPL `/throw-axis-lab.html`, elegir la técnica y seleccionar el JSON local. El formato esperado es `{t: number[], tori: [fotograma][23][3], uke: [fotograma][23][3]}`, en coordenadas GrappleMap (X, Y-arriba, Z). La página lo convierte a X-delante, Y-izquierda, Z-arriba con `(x,-z,y)`. El usuario elige uno de dos modos:

1. **Tori y Uke completos:** dibuja ambos esqueletos a partir de las 23 articulaciones originales; Tori es idéntico en ambas ventanas.
2. **Uke + apoyos de Tori:** sustituye el esqueleto de Tori por segmentos de rodilla/tobillo derechos, pelvis/cuello o caderas. La distancia mostrada es entre líneas ideales del esqueleto, no distancia entre mallas, ni penetración, presión o contacto físico. La pierna de tai-otoshi es una barrera; la pierna de o-soto-gari siega y no es un pivote rígido.

Modos frontal, lateral, cenital y tres cuartos; pausa, avance por fotograma, velocidad real, trazas y parámetros de inversión.

## Hipótesis A/B

La copia alternativa aplica una rotación alrededor del eje Z del mundo sobre **cada punto de Uke**, respecto a su centro de pelvis. Estima el yaw a partir de la línea de hombros de cada fotograma, lo desenvuelve y aplica `deltaYaw = -2 * yaw * intensidad`. Un valor 100 % invierte el yaw estimado (respecto a su orientación inicial), conservando todas las distancias internas y el pitch/roll relativos. Con intensidad cero o «Sin corrección» ambos paneles deben coincidir exactamente.

**No se considera una solución definitiva**: al rotar todo Uke (también manos y pies), cambia sus puntos de contacto y agarres con Tori. Además, la estimación de yaw por hombros puede mezclar giro axial y ladeo; cuando hay rolido intenso, debe contrastarse con las curvas originales `uke_giro` del guion. Es un diagnóstico reversible, no un cambio de las curvas ni del algoritmo.

## Segunda fase: simulaciones dinámicas con MuJoCo

La misma entrada ofrece «Ensayo físico aislado · MuJoCo». El adaptador puro `public/throw-axis-physics.js` transforma los
fotogramas de Tori del JSON importado en escenarios `hpl.scenario.v1`. Abre otra pestaña de la web principal con
`#scn=...`: **es el editor habitual de HPL el que valida, ejecuta y reproduce el resultado con `runScenario` y MuJoCo
WASM** (paso físico de 2 ms). Ningún resultado se persiste ni se añade a la galería.

- **Cuerpo aproximado completo:** 14 segmentos de Tori, posturas de pelvis/torso/pies/manos derivadas del guion y dos
  agarres limitados a 300 N. El motor resuelve el giro de Uke como consecuencia de colisiones y agarres; el ángulo original
  de Uke **no** se utiliza como consigna.
- **Apoyo simplificado:** solo Uke y una barra (espinilla derecha), almohadilla dorsal o almohadilla de cadera. El
  obstáculo sigue una secuencia aproximada de posiciones de Tori con fuerza limitada; dos tirones declarados se aplican
  a manga y esternón. La barra es transversal (eje Y), **no** sigue la orientación angular exacta de una espinilla oblicua.
- **Control sin apoyo:** conserva exactamente los mismos dos tirones y los parámetros de Uke, pero elimina la geometría
  del apoyo. Así permite identificar el efecto del contacto. El control solo está disponible en el modo simplificado,
  porque sin el cuerpo de Tori también desaparecen sus agarres.

Las fuerzas de los tirones son **hipótesis aproximadas**, no mediciones ni fuerzas extraídas de AnimeSpec, que es
cinemático; la transformación de muñecas y tobillos de AnimSpec al cuerpo paramétrico de HPL también es aproximada.
Por tanto el ensayo físico **no reproduce exactamente** el derribo guionizado, aunque puede ayudar a depurar la dirección,
los puntos de contacto y el sentido de los pares. Se debe comparar desde la cámara cenital y frontal, mirar el informe de
contactos y no interpretar una coincidencia como validación biomecánica.

## Ensayo de fulcro unilateral (Uke vertical, tirón simétrico)

Entrada en la misma página. Se construye un escenario aislado con el **modelo humanoide normal de 75 kg y 1,75 m**, `attitude: tense`, `waist_stiffness: 1`, dos fuerzas prescritas iguales (+X, en hombro o manga) desde 0,30 s durante 0,42 s con medio seno y un obstáculo `fixed_box` frente a un solo pie/pierna. Cambia la zona de contacto sin hacer crecer una columna desde el suelo: 13, 33, 55 o 78 cm. Tres intensidades predefinidas (260, 650 y 1200 N **por hombro**), ambos lados de bloqueo, y control sin obstáculo. Se abren como escenarios `#scn` del motor HPL ya existente; **no** se modifica `runScenario` ni se obliga a que Uke gire de un modo concreto. La primera versión sigue usando una barrera geométrica, no una articulación que inmovilice el pie: un pie puede resbalar, saltar o esquivar el tope.

En un modelo ideal sin desplazamiento del punto de apoyo, el momento sobre Z de fuerzas simétricas +X, con un único apoyo a Y=-0,105 m, se aproxima por `τz = -0,21 · F_hombro` N·m (y se invierte con apoyo izquierdo). **Es una predicción externa, no un par aplicado por el motor**. Cuando no hay contacto, no se puede atribuir una rotación a ese tope.

### Batería MuJoCo de GitHub Actions, 13 noviembre 2026

Pruebas de integración en `tests/unit/fulcrumExperiments.test.ts`; cada prueba ejecuta `runScenario` con paso físico 2 ms y registro cada 20 ms. Resultados del job CI 809 (conclusión *success*); valores son de este modelo aproximado y estas fuerzas, no datos biomecánicos de una persona ni la secuencia real de AnimSpec.

| Modelo (siempre tracción hacia delante) | ¿Contacto con tope? | Primera zona de tope registrada | Primer impacto de caída | Altura máxima de pie I/D sobre inicial |
|---|---|---|---|---|
| Sin tope, 650 N | No | — | Cabeza | 146 / 146 cm |
| Tobillo derecho 13 cm, 260 N | **No** | — | Antebrazo izquierdo | 18 / 14 cm |
| Tobillo derecho 13 cm, 650 N | **Sí, 1,16 s** | Pie y espinilla derechos | Antebrazo derecho | 32 / 16 cm |
| Tobillo izquierdo 13 cm, 650 N | **Sí, 1,16 s** | Pie y espinilla izquierdos | Antebrazo izquierdo | 16 / 32 cm |
| Tobillo derecho 13 cm, 1200 N | **No** | — | Cabeza | 163 / 154 cm |
| Espinilla derecha 33 cm, 650 N | **Sí, 0,72 s** | Espinilla derecha | Brazo derecho | 107 / 97 cm |
| Rodilla derecha 55 cm, 650 N | **Sí, 0,72 s** | **Muslo** derecho (no rodilla aislada) | Cabeza | 137 / 138 cm |
| Muslo derecho 78 cm, 650 N | **Sí, 0,74 s** | Muslo derecho, pelvis y muslo izquierdo | Cabeza | 130 / 139 cm |

En el caso medio 13 cm, la dirección de caída medida fue +6,52° (tope derecho) y −6,52° (tope izquierdo); la simetría es una comprobación fuerte de que el lado del apoyo afecta a la salida. **Pero el contacto llega después de un segundo**, por lo que todavía no demuestra que el tobillo sea el fulcro durante toda la caída. Tampoco cumple aún el comportamiento gradual solicitado: con fuerzas suaves no se alcanza el tope; con las mayores el humanoide lo evita y en la caída golpea con la cabeza. En la postura `staggered_left` (pie derecho trasero), el tope ajustado tampoco llegó a contactar en la primera batería.

**Decisión técnica:** la hipótesis de par lateral está respaldada por la simulación de bloqueo medio y su reflexión, pero **no está validado** el modelo de derribo ni la sujeción de un pie trasero. No fusionar automáticamente ni aplicar estos parámetros al juego. El siguiente experimento debe ajustar el tope unilateral para que contacte en la fase de kuzushi, medir deslizamiento del pie bloqueado, comprobar que el tronco evita colisión con Tori, y evitar impactos con la cabeza antes de considerar válidas las curvas.

## Ensayo de kuzushi circular prolongado con fulcro transitorio — 13/11/2026

Añadido a `/throw-axis-lab.html` y a `public/fulcrum-scenarios.js` como **escenarios alternativos**, sin alterar el solver `runScenario` ni los ensayos anteriores:

- Fuerza `pull` continua por puntos de control, entre t=0,30 y 2,60 s (2,30 s), o entre 0,30 y 3,38 s (3,08 s). Primero +X/+Z (kuzushi), luego +X, luego +X/-Z (continuación del giro); la fuerza termina suavemente. No es animación posicional de Uke.
- Tracción en manga derecha, ajustable: simétrica / 1,45× / 1,85× durante la fase final, con ensayo control de tracción recta de igual duración; se puede tirar del hombro en vez de la manga.
- Tope unilateral **sin unión rígida** (fixed_box). Variante temporal con `obstacle: pad` con 4 kg y servo limitado, que se retira lateralmente por una curva a partir de ~t=1,46 s; el contacto puede cesar *antes* de la retirada si Uke lo sobrepasa.
- Brazo izquierdo y derecho tienen curvas temporales medibles. HPL calcula por su cuenta contacto, fuerzas, inclinación, giro y posiciones; si Uke no toca el tope, nunca se afirma que haya pivotado sobre él.

Batería real MuJoCo WASM, CI #819 en verde, y repetición con almohadilla móvil más estrecha (CI #822 en verde). Los valores son **resultados de simulación** y no una técnica ya validada.

| Ensayo | Contacto con tope | Primer contacto suelo | Cabeza en suelo | Cara frontal del pecho al tocar suelo* |
|---|---|---|---|---|
| Recto prolongado, tope espinilla derecha, 1,45× manga | 1,208–1,232 s, espinilla derecha | antebrazo derecho | sí, 2,45 m/s | −0,936 |
| Circular prolongado, mismo tope, 1,45× manga | 1,040–1,056 s, espinilla derecha | antebrazo derecho | sí, 4,55 m/s | −0,918 |
| Circular, manga derecha 1,85×, tope espinilla | 1,040–1,056 s, espinilla derecha | antebrazo derecho | sí, 2,99 m/s | −0,886 |
| Circular, tope móvil estrecho que se retira | 0,624–0,976 s, pie/espinilla derechos | antebrazo derecho; después pecho y pelvis | **no registrado** | −0,623 |
| Circular, tope tobillo derecho estacionario | ninguno | brazo izquierdo | sí, 4,02 m/s | no se utiliza el fulcro |

*La componente vertical de la dirección frontal del pecho en el primer contacto de pecho, medida con `xmat[6]` para el cuerpo `chest`. **Negativa significa que el pecho mira hacia abajo, por lo que no constituye caída dorsal** (aproximación con rig físico HPL). El tope móvil elimina el contacto con la otra pierna en la nueva configuración, pero la recepción sigue siendo **prona/no dorsal** y el impacto del pecho fue ~5,94 m/s: **NO APTO** para dar por corregido Tai Otoshi.

**Hallazgo biomecánico esencial:** la separación ocurre de forma espontánea; un fulcro no necesita contacto continuo. El obstáculo fijo de espinilla solo hizo contacto ~16 ms antes de separarse. La fuerza circular cambia drásticamente la trayectoria, pero con esta parametrización el giro no pasa a una recepción sobre espalda. Hacen falta pruebas de los pares de rotación, control de orientación dorsal y ajuste de dirección y temporización del tirón, comparando siempre el escenario sin obstáculo. No deducir que una fuerza mayor o más prolongada asegura ukemi.

**Vídeo diagnóstico:** la conversación acompaña un MP4 con comparación recto/circular reconstruida de las posiciones del motor a 125 Hz, submuestreadas para visualización. El modelo de trazo son cuerpos/centros de segmento, no la malla definitiva; la interpolación entre frames no recalcula física.

## Qué revisar en cada técnica

- **Tai-otoshi:** a partir de 1,57 s, dirección de la cabeza respecto a pelvis, trayectoria de hombros, sobrepaso de la barrera de la pierna derecha de Tori y lado de caída. Comparar la vista cenital y la frontal; comprobar la mano que retiene la manga.
- **O-soto-gari:** entre 1,03 y 1,75 s, trayectoria de pelvis y cabeza sobre el lado segado, apoyo de pierna derecha de Uke, giro hacia atrás y espacio con la pierna segadora de Tori. Una distancia pequeña no demuestra que haya siega efectiva.

## Criterios de revisión antes de tocar AnimSpec

- El control sin corrección coincide exactamente en ambos lados.
- Se reproduce el fallo original con JSON real y se identifica el primer fotograma divergente.
- La hipótesis mejora el sentido de giro **sin empeorar claramente** agarres, distancia al apoyo o trayectoria del cráneo.
- Después, una corrección en AnimSpec debe evaluarse por separado con render Blender, pruebas nuevas de sentido de rotación y contacto, y las referencias visuales de cada técnica. Hasta entonces las grabaciones son inmutables.
- La simulación dinámica real con fuerzas, pares, balance, fricción y contacto MuJoCo es una **fase posterior**, no atribuible a esta prueba visual.

## Comprobaciones manuales rápidas

1. Abrir la entrada con el navegador y cambiar entre dos técnicas.
2. Seleccionar «Sin corrección» y verificar igualdad geométrica de las vistas.
3. Activar inversión completa, poner vista cenital, avanzar durante la caída y comprobar el giro lateral.
4. Cambiar a modo de apoyos, probar pierna y espalda; comparar las distancias durante el mismo fotograma.
5. Importar ambos JSON originales (solo locales), repetir 2–4 y anotar el primer fotograma en que el lado mejora o empeora.
6. Confirmar que ninguna grabación de `public/recordings/` ni escenario de `scenarios/` ha sido modificada en el diff.

**Comprobación:** se han añadido tests de validación del modelo y de ejecución MuJoCo para Tai-otoshi y O-soto-gari. La comprobación definitiva con los JSON de AnimeSpec y una inspección desde el navegador sigue pendiente. El experimento no autoriza modificar ni regenerar las grabaciones originales.
