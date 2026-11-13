# Editor de escenarios en la web

Sirve para probar por ensayo y error sin instalar nada: abrir un derribo de la galería, cambiar rozamientos, cuerpos y la
curva de fuerzas, simularlo en el navegador y ver qué le pasa a Uke. Está pensado para personas y para modelos que no
conocen el código.

Web: <https://jonamarti.github.io/humanoid-physics-lab/>

## Uso

1. Elige una grabación en **Grabaciones de ejemplo** (o abre un enlace `?rec=<id>`).
2. Despliega **Editar y simular este escenario**, debajo de los controles de reproducción.
3. Cambia valores y pulsa **Simular con estos valores**. Tarda alrededor de medio segundo.
4. Arriba se reproduce el resultado; debajo del botón aparece el informe del ensayo.
5. **Enlace a este escenario editado** lleva todo el escenario dentro de la URL: se puede pegar en un chat y quien lo abra
   ve el mismo ensayo, ya simulado. **Descargar JSON** guarda el escenario para `npm run sim`. **Restablecer** vuelve al
   original.

La física es la misma de `npm run sim` (`runScenario`): mismo modelo, mismo paso de 2 ms. Nada se envía a un servidor.

## Qué se puede editar

| Bloque | Campos | Notas |
|---|---|---|
| General | Duración, rozamiento del suelo | El rozamiento del suelo es el de los pies de Uke |
| Uke | Masa, altura, fuerza muscular, actitud | `en tensión` mantiene la postura; `relajado` no tiene objetivos |
| Tori | Activar o quitar a Tori; masa y rozamiento de sus pies; casilla del cuerpo; altura, rozamiento con Uke, qué agarra cada mano (con tope y suelta) y tablas de posturas, pies y manos | Ver [TORI_BODY.md](TORI_BODY.md). Sin Tori solo actúan las fuerzas directas y los obstáculos |
| Fuerzas directas sobre Uke | «+ Impacto» y «+ Fuerza constante»; en cada una: punto, tipo, ejes, componentes X/Y/Z, inicio, duración | El punto se elige de una lista o tocando a Uke en el visor 3D |
| Añadir obstáculo | «+ Barra», «+ Poste», «+ Almohadilla» | Cada obstáculo, tirón, mano y fuerza tiene su botón «Quitar» |
| Obstáculo | Rozamiento con Uke, distancia, altura, radio, topes de fuerza, tramos de movimiento | Es la cadera, la pierna o el pie de Tori |
| Tirón (`pull`) | Topes de fuerza y de potencia, y una tabla con la fuerza en cada instante | Ver abajo |
| Mano (`hand`) | Tope de fuerza, cierre, suelta, tramos del objetivo | Mano con muelle: se guía la posición |

Un campo en gris muestra el valor por defecto que se usará si se deja vacío.

### La tabla de un tirón

Cada fila es un instante: **Instante (s)**, **Fuerza (N)**, **Elevación (°)** y **Lateral izq. (N)**. Entre filas la fuerza se
interpola en línea recta; antes de la primera fila y desde la última no hay fuerza. Para editar la fuerza «fotograma a
fotograma» basta con añadir filas, por ejemplo una cada 0,1 s.

Elevación, vista desde Uke al empezar (no gira con él):

| Elevación | Hacia dónde tira |
|---|---|
| 90° | arriba |
| 45° | delante y arriba |
| 0° | delante |
| −45° | delante y abajo |
| −90° | abajo |
| −110° a −135° | abajo y hacia atrás (hacia los pies de Tori, que está delante de Uke) |

Ningún punto puede superar el tope de fuerza (500 N por defecto): si hace falta más, sube antes el tope.

### El JSON completo

**JSON completo del escenario (avanzado)** enseña el escenario entero y permite pegar otro; **Aplicar el JSON al
formulario** lo carga. Ahí se pueden cambiar cosas que el formulario no enseña (sitio de agarre, forma del obstáculo,
`record_hz`). Los campos están en [SCENARIO_V1.md](SCENARIO_V1.md).

## Cómo leer el informe

- **Resultado**: `fall`, `standing`, `stepped` o `unresolved`, cuándo y qué parte toca primero el suelo.
- **Suelo, por orden de llegada**: partes de Uke que golpean el suelo y con cuánta fuerza. Un buen volteo llega con brazo,
  hombro o espalda; uno malo, con la cabeza.
- **Cabeza**: si toca el suelo. El valor es «al menos», porque los contactos se graban cada 10 ms; si el pico real supera
  10 kN aparece además en **Artefactos**.
- **Obstáculo**: qué partes de Uke toca la cadera y durante cuánto. Si toca la pelvis medio segundo, lo ha cargado; si
  solo roza los muslos unas centésimas, se le ha escurrido.
- **Fuerza**: máximo de cada mano frente a su tope.
- **Tori**: `viable` o `INVIABLE` y por qué (resbalaría, se quedaría sin apoyo, tira más de lo que puede).
- **Artefactos**: avisos del detector (golpes de más de 10 kN, penetraciones, temblor).

## Enlaces con el escenario dentro (para automatizar)

`https://jonamarti.github.io/humanoid-physics-lab/#scn=<B>`, donde `<B>` es el JSON del escenario en UTF-8 codificado en
base64 para URL (`+` → `-`, `/` → `_`, sin `=` finales). Al abrirlo, la web lo valida, lo simula y muestra el informe. Si el
escenario no es válido, el mensaje dice el campo y el motivo. En Node:

```js
const hash = "#scn=" + Buffer.from(JSON.stringify(scenario)).toString("base64url");
```

Los escenarios de la galería están publicados en `scenarios/<id>.json` junto a la web (por ejemplo
`.../humanoid-physics-lab/scenarios/uke_tense_75_175_hip_roll.json`): son el punto de partida más cómodo.

## Recetas de prueba

Sobre `uke_tense_75_175_hip_roll` (volteo con tirón en arco):

- **¿Cuánto rozamiento necesita la cadera?** Baja «Rozamiento con Uke» del obstáculo de 0,8 a 0,5 y a 0,2. Mira en el
  informe si la barra sigue tocando la pelvis o pasa a los muslos.
- **¿Qué pasa si la cadera entra baja?** Baja «Altura del centro, Z» de 0,90 a 0,72.
- **¿Y si no se acompaña con las manos?** Quita las filas de 1,0 y 1,2 s de los dos tirones.
- **¿Y con menos fuerza?** Baja la columna «Fuerza (N)» de todas las filas.
- **¿Y con un Uke más pesado?** Sube la masa de Uke a 100 kg sin tocar nada más.

Sobre `uke_tense_75_175_tori_body` (Tori con cuerpo y agarres):

- **¿Qué hace el cuerpo?** Desmarca «Cuerpo de Tori» y simula: sin cuerpo no hay agarres y Uke se queda de pie.
- **¿Y si suelta la solapa?** Pon «Suelta a (s)» de la mano derecha en 1,05.
- **¿Y si la manga va pegada al cuerpo?** En la tabla de la mano izquierda, baja «Hacia la cabeza» de las filas 2 a 4 a 0.
- **¿Y si agarra la muñeca en vez de la manga?** Cambia «Mano izquierda agarra» a «muñeca derecha».
- **¿Y si entra más bajo?** Baja «Pelvis Z» de la cuarta postura.

Sin Tori (pulsa «Quitar a Tori» y quita los tirones, o abre `uke_tense_75_175_side_impact_post`):

- **Un golpe donde quieras.** «+ Impacto», «Elegir el punto en el visor 3D», toca a Uke y simula.
- **Un empuje sostenido contra algo.** «+ Fuerza constante» y «+ Poste»; mueve el poste con X e Y.

## Límites

- **El cuerpo de Tori es una marioneta** de segmentos sueltos con fuerza limitada, sin equilibrio propio; sus límites están
  en [TORI_BODY.md](TORI_BODY.md).
- Elegir el punto en el visor pausa la reproducción y usa la postura de ese instante. El punto queda fijo en el cuerpo tocado.
- La simulación bloquea la pestaña mientras calcula (medio segundo en un portátil; más en un móvil).
- El visor web dibuja el cuerpo, el obstáculo y las flechas de las manos. No dibuja las fuerzas de contacto ni el centro de
  masas (eso solo sale en el vídeo de `scripts/render_recording.py`).
- El enlace con el escenario ocupa alrededor de 1700 caracteres; algún programa de mensajería puede cortarlo.
- Probado en el navegador de escritorio y con pantalla emulada de 375 px. Sin probar en un móvil real ni en Safari.

## Referencias de tempo y de fuerza

Búsqueda hecha el 12-11-2026; ninguna cifra de aquí está calibrada contra el modelo.

- **Fuerza de tirón.** El perfil de partida (800 N entre las dos manos, 500 N de tope por mano) viene de la referencia que
  ya usaba el proyecto: tirón máximo en laboratorio de unos 1050 a 1100 N en total y de 470 a 620 N por manga en judocas
  de élite juveniles (Yilmaz, 2015). Varios estudios coinciden en que la mano de la manga tira más que la de la solapa; el
  escenario actual usa la misma fuerza en las dos.
- **Tempo.** No he encontrado duraciones absolutas en segundos de cada fase. Los estudios cinemáticos localizados comparan
  velocidades entre niveles: la duración de la fase de desequilibrio y entrada no distingue a los mejores, sí la velocidad
  hacia delante del centro de masas y la velocidad angular del tren superior. El escenario dura 1,2 s desde el primer tirón
  hasta la suelta, elegido para que el modelo complete el giro, no medido.
- **Rozamiento de la tela.** No he encontrado un coeficiente medido para judogi contra judogi. Los 0,8 (tela) y 0,5 (piel)
  son puntos de partida sin calibrar.

Lo que falta: medir los tempos sobre vídeo de referencia a velocidad real y ajustar los instantes de la curva.

Fuentes consultadas:

- [Comparación cinemática entre atletas de élite y de segundo nivel](https://lida.sport-iat.de/twm/Record/4077470?lng=en)
- [Factores angulares de la rapidez entre atletas de élite y universitarios](https://lida.sport-iat.de/ta/Record/4034009)
- [Fuerza de tirón de cada mano, Journal of Sports Science and Medicine 17, 465](https://www.jssm.org/jssm-17-465.xml-Fulltext)
