# Vídeo sin navegador

`scripts/render_recording.py` convierte una grabación `hpl.recording.v1` (`.json` o `.json.gz`) en un vídeo MP4 y, opcionalmente, en una hoja de 8 fotogramas clave. Usa el renderizador sin ventana de MuJoCo de Python. Referencia: issue #74, PR 6.

## Uso

```sh
uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow \
  python scripts/render_recording.py out/cyl_low/recording.json.gz out/cyl_low/video.mp4 \
  --view side --size 960x540 --sheet out/cyl_low/hoja.png
```

| Opción | Efecto |
|---|---|
| `--view front\|side\|top\|three_quarter` | Cámara (por defecto `side`; en ella +X va a la derecha). |
| `--slow 0.25` | Segunda pasada a ×0,25 tras la de velocidad real. `0` la desactiva. |
| `--size 960x540` | Tamaño (par). |
| `--follow` / `--no-follow` | Fuerza que la cámara siga (o no) al actor. Por defecto decide sola (ver abajo). |
| `--fps 30` | Fotogramas por segundo del vídeo. |
| `--sheet hoja.png` | Rejilla de 8 fotogramas clave repartidos en el tiempo, con su instante. |

El vídeo es H.264 `yuv420p` con `faststart`, reproducible en el móvil. Los fotogramas se envían a ffmpeg por tubería: no quedan fotogramas intermedios en disco.

## Qué hace (y qué no)

- **Nunca simula.** Carga `model.mjcf` de la grabación, pone `qpos` de cada muestra, llama a `mj_forward` y renderiza. Una prueba lo comprueba con un modelo sin gravedad cuyas muestras describen una caída.
- Remuestrea de `record_hz` a `--fps` eligiendo la muestra más cercana. El vídeo incluye la última muestra.
- Luminoso: al cargar el MJCF (sin tocar la grabación) se inyecta un cielo degradado claro, un suelo claro con cuadrícula, más luz ambiente y una luz direccional. Si el MJCF ya trae skybox o material de suelo se respeta.
- Cámara: se calcula un encuadre inicial con la caja envolvente del primer cuerpo con articulación libre (el actor) en toda la grabación, más los cuerpos fijos. Si cabe sin alejar demasiado la cámara, queda fija y se percibe el desplazamiento real; si no, sigue al actor. `--follow` / `--no-follow` fuerzan una u otra.
- Rótulo persistente abajo a la izquierda, `<id>: <N> N (pico <N> N)`, mientras el efector está activo y 0,5 s después (no depende de caer en el fotograma exacto).
- La hoja incluye al menos un fotograma en el pico de cada efector; el resto se reparten.
- Flechas por efector (rojo; el color es fijo por `kind` si la muestra lo trae) con su valor en newtons, y flechas de contacto en amarillo. Escala: 1 m = 1000 N, con longitud mínima de 8 cm y radio de 1,8 cm para que una fuerza pequeña siga siendo visible. Las etiquetas llevan recuadro semitransparente y su tamaño depende de la resolución. La flecha nace en el punto de aplicación y apunta en la dirección de la fuerza. Se omiten las menores de 2 N.
- Tiempo de la grabación y velocidad (×1 o ×0,25) en la esquina superior izquierda.
- Equilibrio (si las muestras traen `com`, `xcom` y `support_polygon`, ver [RECORDING_V1.md](RECORDING_V1.md)): capa 2D proyectada desde la cámara, siempre visible aunque el cuerpo tape el punto. Centro de masas: círculo azul con una plomada y un anillo en su proyección sobre el suelo. Punto de captura: círculo magenta sobre el suelo, unido a la proyección del centro de masas por una línea. Polígono de apoyo sobre el suelo: relleno translúcido y contorno, verde con el punto de captura dentro y naranja fuera. Leyenda abajo a la derecha con el margen con signo (`sin apoyo` si ningún pie está cargado). Sin esos campos (cilindro, grabaciones antiguas) no se dibuja nada.
- El encuadre de un actor de varios cuerpos usa la altura de todo el árbol de cuerpos y no solo la de su raíz (antes un humanoide salía con la cabeza cortada).
- Tolerante: sin `effectors`, `contacts`, `qvel`, `peaks` ni `summary` funciona igual. Un `schema` distinto de `hpl.recording.v1` termina con código 2 y un mensaje claro.

## Datos de prueba y pruebas

```sh
uv run --quiet --python 3.12 --with mujoco --with numpy python scripts/make_sample_recording.py out/sample.json.gz
uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow python scripts/test_render_recording.py
```

`make_sample_recording.py` genera con MuJoCo de Python una grabación con el formato de la issue (cilindro de 75 kg, 1,75 m, radio 0,18 m, obstáculo fijo, medio seno a 1,40 m de altura, fuerzas de contacto reales de `mj_contactForce` rotadas al mundo). Es un dato de prueba: no sustituye a la salida de `npm run sim`.

## Extensión opcional del formato

Cada entrada de `samples[].effectors` puede llevar `kind` (por ejemplo `prescribed_force`) para elegir el color. Si falta, se usa el color por defecto.

## Limitaciones

- Las etiquetas de texto son un recuadro de Pillow proyectado desde la cámara, no etiquetas de MuJoCo; los contactos muy juntos comparten etiqueta.
