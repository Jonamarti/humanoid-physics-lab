# Modelo paramétrico de Uke

Issue #75, PR 1. Código: `src/uke/generate.ts` (`generateHumanoidMJCF`, `computeHumanoidSpec`) y `src/uke/rangeOfMotion.ts`.
Pruebas: `tests/unit/ukeGenerate.test.ts`. No modifica ningún XML de `public/models/`.

## Qué hace

`generateHumanoidMJCF({ mass_kg, height_m, build?, segment_lengths_m? }, options?)` devuelve un MJCF completo (suelo incluido)
con raíz libre en la pelvis y bisagras internas. Con 75 kg y 1,76 m reproduce `public/models/humanoid_step_v2.xml`: misma jerarquía,
mismos nombres de cuerpos, articulaciones (y orden) y sitios, masas por cuerpo, posiciones de articulación y altura de pelvis (0,99 m).
Usa `worldOptionTag()` para `<option>` (no las opciones de v2) y, por geometría, los parámetros de contacto de v2
(`solref="0.015 1" solimp="0.9 0.95 0.001"`, rozamiento `1.0 0.025 0.002`; el deslizante se cambia con `options.friction`).
`options.extraWorldXml` permite insertar props en `<worldbody>` y `options.jointDampingByName` fija el `damping` (implícito) de cada articulación.
El MJCF lleva cuatro `<contact><exclude>` (abdomen con ambos muslos y pelvis con ambas piernas): con las tallas gruesas esas geometrías se solapan (medido: 10 kN entre el abdomen y los muslos de 150 kg con las rodillas flexionadas).

Añadidos respecto a v2: mano (esfera de radio 4 cm, 0,45 kg descontados del antebrazo), articulación `neck_yaw` (añadida la última del
cuello, para no cambiar el orden de composición de las existentes), rangos articulares nuevos y sitios de agarre e impacto
(`nape`, `sternum`, `belt_front`, `belt_back`, `left/right_sleeve_elbow`, `left/right_wrist`, `left/right_triceps`, `left/right_armpit`,
`left/right_knee_back`, `left/right_ankle`, `head_front`, `head_side_left/right`, `abdomen_front`). Los sitios de v2 se conservan.
`sternum` y `sternum_impact` están en el mismo punto.

Rango admitido: 50 a 150 kg y 1,50 a 2,05 m; fuera de él lanza `RangeError`.

## Longitudes

Las de v2 por `height_m / 1,76`. `segment_lengths_m` (claves `thigh`, `shin`, `foot`, `upper_arm`, `forearm`, `trunk`, `neck`) manda sobre
ese factor, con un límite de 0,5 a 2 veces el valor de v2. Definiciones: `foot` es la longitud total de la caja del pie (0,27 m);
`trunk` va del origen de la pelvis a la articulación del cuello (0,11 + 0,23 + 0,24 = 0,58 m) y reparte su escala entre pelvis, abdomen y
pecho; `neck` va de la articulación del cuello al centro de la cabeza (0,065 m). Alturas de pelvis y de cadera, y tamaño de la caja del pie
en vertical, escalan con `height_m / 1,76`.

## Masas (aproximación)

Masas de v2 por `mass_kg / 75`. Con `imc = mass / height²` mayor que 27, la masa «normal» es `27 · height²` y el exceso se reparte así:
el 60 % (`build`, por defecto 0,6) va a pelvis, abdomen y pecho y el 40 % restante a todos los segmentos en proporción a v2.
**Desviación documentada:** dentro del tronco, el exceso no se reparte en proporción a las masas de v2 (14:9:16) sino 0,7 / 0,25 / 0,05
(pelvis, abdomen, pecho). Con el reparto proporcional el centro de masas de 150 kg / 2,00 m salía al 60,0 % de la altura, fuera del 53–59 %
exigido; con 0,4/0,4/0,2 al 59,5 % y con 0,5/0,35/0,15 al 59,2 %. Con 0,7/0,25/0,05 sale al 58,5 %. Es una regla única, no un ajuste por talla.
Es una aproximación: las proporciones de segmento de v2 (tipo De Leva) son las de un adulto joven y atlético, y no representan a un
cuerpo real de 150 kg.

## Grosor

Radios de v2 por `sqrt((mass/height) / (75/1,76))`. Los del tronco (pelvis, abdomen, pecho, en x e y) llevan además el factor de complexión
`sqrt(masa_tronco / masa_tronco_si_todo_escalara_igual)`, que vale 1 con imc ≤ 27. Las semiaxes verticales de pelvis, pecho y cabeza y los
largos de cápsula usan el factor de longitud. **Desviación:** la cabeza escala con `height_m / 1,76` y no con el factor de grosor, porque
una cabeza de 24 cm de ancho para 150 kg no es plausible. El hombro se separa de la línea media en proporción al ancho del pecho
para que el brazo no nazca dentro del tronco, y las caderas a 0,7 veces el semiancho de la pelvis.

## Rangos articulares

`rangeOfMotion.ts` define la tabla en grados y la convierte a radianes; en roll y yaw de articulaciones pares el lado derecho es el espejo.
Signos: eje +Y positivo = flexión de tronco y cuello, extensión de cadera y hombro, flexión plantar; la rodilla usa el eje −Y.

| Articulación | Rango izquierdo (grados) | v2 (rad) |
|---|---|---|
| `lumbar_pitch` / `roll` | −25 a 60 / ±25 | ±0,50 / ±0,35 |
| `thorax_pitch` / `roll` / `yaw` | −20 a 35 / ±20 / ±40 | ±0,45 / ±0,35 / ±0,55 |
| `neck_pitch` / `roll` / `yaw` | ±45 / ±40 / ±80 | −0,60 a 0,55 / ±0,35 / no existía |
| `shoulder_pitch` | −178 (flexión) a 60 (extensión) | ±1,9 |
| `shoulder_roll` | −45 (aducción) a 178 (abducción) | ±1,5 |
| `shoulder_yaw` | −70 a 90 | ±1,6 |
| `elbow` | −150 a 0 | −2,4 a 0 |
| `hip_pitch` | −120 (flexión) a 30 (extensión) | −1,65 a 1,25 |
| `hip_roll` / `hip_yaw` | −30 a 45 / ±45 | ±0,70 / ±0,75 |
| `knee` | −140 a 0 | −2,45 a 0 |
| `ankle_pitch` | −30 (dorsiflexión) a 50 | ±0,65 |
| `ankle_roll` | −35 (inversión) a 15 (eversión) | ±0,38 |

La flexión del hombro pasa de 1,9 rad a 3,1 rad (178°).

**Fuente: PENDIENTE DE VERIFICAR CON FUENTE.** Al escribir estos valores no se consultó ninguna tabla. Son valores estándar de goniometría
clínica de adultos de memoria, del orden de los de la AAOS («Joint Motion: Method of Measuring and Recording», 1965) y de Norkin y White,
«Measurement of Joint Motion: A Guide to Goniometry». Hay que contrastarlos con la fuente y anotar aquí la edición y la página antes de
darlos por buenos. Las diferencias con v2 más relevantes: la extensión de cadera baja de 1,25 a 0,52 rad, y las del cuello, el hombro y el
tobillo cambian de forma notable.

## Qué no se ha comprobado

- Que los rangos coincidan con una fuente publicada (ver arriba).
- Que la distribución de masa y el grosor correspondan a personas reales con complexiones extremas.
- Que el modelo se sostenga de pie con control: eso es de la PR siguiente (`gains.ts`, `attitudes.ts`).
