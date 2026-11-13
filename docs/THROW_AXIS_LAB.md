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
