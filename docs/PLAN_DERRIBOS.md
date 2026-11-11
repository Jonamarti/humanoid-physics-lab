# Plan: laboratorio físico de derribos, golpes y reacciones de Uke

**Estado:** propuesta del 11-11-2026, pendiente de aprobación de Jon. Nada de este plan está implementado.
**Para quién:** cualquier agente que vaya a implementar una fase. Lee el documento entero antes de tocar código.

## 1. Objetivo

Que Jon (o un agente) describa cómo actúa Tori y el motor muestre cómo reacciona Uke de forma físicamente creíble.

Flujo de uso:

1. Jon da una instrucción («agarra las dos mangas, tira hacia arriba y delante, mete la cadera») o una referencia en vídeo o capturas.
2. Un agente la traduce a un **escenario**: agarres, fuerzas por fases, obstáculos y barridos, con tiempos y magnitudes.
3. El motor simula. Uke (masa, altura y actitud configurables) reacciona: aguanta, recoloca los pies, da un paso o cae.
4. Salen una **grabación**, los vectores de fuerza aplicados, las métricas y un vídeo.
5. Jon mira el vídeo y dice si es natural o en qué falla.

Debe servir para judo y grappling: agarres de ropa (manga, solapa, cinturón) y de cuerpo (muñeca, cuello, tríceps, axila, cintura, corva, tobillo); tirones, empujones, pescas; caderas, hombros, segados, barridos, double leg y single leg.

Debe servir también para el encadenado del juego de dojo: Uke en postura defensiva no cae con un derribo directo; Tori empuja, Uke da un paso para no caer y en ese instante el derribo sí entra.

### El motor es un generador de animaciones

Decisión de Jon (11-11-2026): los juegos **no** ejecutan este motor. Se usa fuera del juego para generar la animación, que se exporta y se guarda en AnimSpec; Unity solo reproduce clips. El motivo de simular es obtener variantes sin animar a mano: no cae igual alguien de 70 kg que de 150 kg, ni alguien de 1,70 m que de 2,00 m. Un juego (o Jon a mano) pide «esta técnica para este cuerpo» y recibe el clip.

Consecuencias:

- No hay requisito de tiempo real ni de rendimiento en el juego; importa la calidad y la repetibilidad.
- El rango de cuerpos es parte del motor desde la fase 2: de 50 a 150 kg y de 1,50 a 2,05 m.
- La masa y la fuerza son parámetros separados: una persona de 150 kg no tiene el doble de fuerza que una de 75 kg.

### Fuera de alcance

- Física en vivo dentro de Unity.

- Marcha autónoma de Uke (caminar varios pasos). Solo hace falta un paso de recuperación.
- Simulación de tela. El judogi es un punto de agarre con holgura y fuerza máxima.
- Daño, dolor, lesión y KO.
- Lucha en el suelo tras la caída.

## 2. Decisión: continuar, no empezar de cero

Se continúa sobre Humanoid Physics Lab (HPL). Se cambia el rumbo, no la base.

| Se conserva | Motivo |
|---|---|
| MuJoCo WASM, reloj fijo, `MujocoSimulation` | Funciona y es determinista |
| Modelo 3D `humanoid_step_v2.xml` (cadera, hombro y columna con tres ejes) | Evita el fallo del Uke plano de AnimSpec |
| De pie 10 s y equilibrio sin pasos | Es la base de la reacción de Uke |
| Editor de fuerzas 3D, flechas, COM y huella de apoyo | Es la visualización que pide el objetivo |
| Ensayo del cilindro con obstáculo y su ecuación de vuelco | Es el primer derribo y su validación analítica |
| Suite de pruebas, CI y despliegue en Pages | Permite ver resultados desde el móvil |

| Se aparca (sin borrar) | Motivo |
|---|---|
| Marcha rectilínea, antepié articulado v3, propulsión del pie (#39, #40, #49, #62, #63, #66) | No hace falta para derribos; es lo más difícil |
| Rama `experiment/70-load-aware-recovery` y PR #71 | Queda como evidencia; el paso reactivo se retoma en la fase 4 con otro enfoque |
| `aparcado/fisica/` de AnimSpec | Uke plano y Tori de fuerza infinita; sirve de referencia de protocolos y cifras |

Reparto entre repositorios (ya propuesto en `ARCHITECTURE_BOUNDARY.md`):

- **HPL (público):** toda la física. No conoce nombres de técnicas; solo agarres, fuerzas, obstáculos y cuerpos.
- **AnimSpec (privado):** biblioteca de técnicas con nombre (`seoi_nage`, `osoto_gari`…) que se traducen a escenarios de HPL, referencias de vídeo, y conversión de la grabación a los rigs de Blender y Unity.

## 3. Idea central: Tori abstracto

El fallo del seoi nage de AnimSpec (medido el 11-11-2026) fue un Tori cinemático de fuerza infinita: picos de contacto de 4,9 kN y agarres de 2,2 kN que lanzaban a Uke en la dirección que marcara el muelle.

Aquí Tori empieza siendo solo **lo que le hace a Uke**, con límites humanos. Es la generalización del cilindro con obstáculo.

| Efector | Qué es | Ejemplos |
|---|---|---|
| **Agarre** | Punto del cuerpo de Uke donde Tori aplica una fuerza con perfil temporal y tope | Manga, solapa, muñeca, nuca, tríceps, corvas |
| **Obstáculo** | Sólido que bloquea a Uke a una altura; fijo o movido con fuerza limitada | Cadera u hombro de Tori como fulcro, pie que bloquea un tobillo |
| **Segado o barrido** | Obstáculo que se mueve contra una pierna de Uke con velocidad y fuerza máximas | Osoto gari, de ashi barai, ganchos |
| **Empuje** | Fuerza repartida en una zona, sin agarre | Empujón al pecho, hombro contra la cadera en double leg |

Reglas de los efectores:

- Toda fuerza tiene tope declarado y se registra. Punto de partida: unos 500–600 N sostenidos por mano y unos 1100 N de tiro total (Yilmaz 2015, ya citado en AnimSpec). Son valores a calibrar, no verdades.
- Un agarre tiene holgura (la tela cede unos centímetros antes de tirar) y se suelta o resbala al superar su tope.
- **Presupuesto de Tori:** en cada instante se suma lo que Tori ejerce y se comprueba que un Tori de su masa podría sostenerlo con sus pies (reacción horizontal ≤ rozamiento × carga vertical). Si no, el ensayo se marca como «Tori inviable». Así se detecta fuerza imposible sin simular aún el cuerpo de Tori.
- Ninguna fuerza externa sobre la raíz de Uke, ningún «pegamento», ningún par de corrección. Si Uke no se comporta bien, se arregla el modelo o el controlador.

El cuerpo completo de Tori llega en la fase 6, cuando los efectores ya den derribos creíbles.

### Actitud de Uke

Parámetro del escenario, no código distinto por técnica:

| Actitud | Comportamiento |
|---|---|
| `relajado` | Tono bajo, sin equilibrio activo (muñeco de trapo con límites articulares) |
| `tenso` | Mantiene la postura con pares limitados, sin recolocar pies |
| `reactivo` | Equilibrio de tobillo y cadera, y paso de recuperación |
| `defensivo` | Como `reactivo`, con base ancha, centro bajo y más resistencia |

Más adelante se puede añadir caída protectora (ukemi) como reacción al perder el equilibrio.

## 4. Contrato de datos

Dos ficheros JSON versionados. Unidades SI; ejes +X delante, +Y izquierda, +Z arriba (`COORDINATES.md`).

**Escenario (`hpl.scenario.v1`), entrada:**

- `uke`: masa, altura, actitud, postura inicial (pies, flexión, orientación).
- `tori`: masa y posición de los pies (para el presupuesto), sin cuerpo en v1.
- `suelo`: rozamiento.
- `efectores`: lista de agarres, obstáculos, segados y empujes. Cada uno con cuerpo de Uke y punto local, o nombre semántico del catálogo (`manga_d`, `solapa_i`, `nuca`, `triceps_d`, `corva_i`…).
- `fases`: tramos con nombre libre (`desequilibrio`, `entrada`, `proyeccion`) que fijan, por efector, dirección, magnitud, forma de onda e instante de suelta. La dirección declara su marco: mundo, Uke o «de Uke hacia Tori».

**Grabación (`hpl.recording.v1`), salida:**

- Metadatos: versión del esquema, del motor y del modelo, parámetros y semilla.
- Por muestra: tiempo, posición y orientación de la raíz, ángulos articulares, velocidades.
- Fuerzas: cada efector (punto y vector), contactos, reacción del suelo por pie.
- Equilibrio: centro de masas, punto de captura, polígono de apoyo, estado del controlador.
- Métricas y veredicto (sección 6).

La reproducción de una grabación nunca recalcula física.

## 5. Fases

Cada fase es una issue con sus criterios. No se empieza una fase sin cerrar la anterior, salvo la 4, que puede ir en paralelo a la 3.

### Fase 0. Orden previo

- AnimSpec no tiene créditos de GitHub Actions y sigue privado: sus checks en rojo se ignoran y sus pruebas se ejecutan en local antes de fusionar. La física se valida en el CI de HPL, que es público.
- Subir o descartar el trabajo local de AnimSpec (`impacto_pecho`, índice de animaciones).
- Dejar la PR #71 de HPL como borrador de evidencia y limpiar los `.log` de la raíz.
- Fusionar las dos ADR de separación (PR #97 de AnimSpec y su pareja en HPL) con este plan enlazado.
- Reetiquetar las issues de HPL: aparcadas, y nuevas por fase.

**Hecho cuando:** `main` de ambos repos está limpio, el CI de HPL pasa y las issues reflejan este plan.

### Fase 1. Contrato, ejecución sin navegador y reproducción

1. Esquemas `hpl.scenario.v1` y `hpl.recording.v1` con validación (rechazar versiones desconocidas y valores no finitos).
2. Comando Node: `npm run sim -- escenario.json --out carpeta`. Vitest ya ejecuta MuJoCo WASM en Node, así que no hace falta navegador.
3. Modo reproducción en el visor: carga una grabación y la dibuja con flechas, centro de masas y apoyo, con pausa, cámara lenta y fotograma a fotograma.
4. Script de captura que genera vídeo y hoja de fotogramas a partir de una grabación (sin física en vivo, para que sea repetible).
5. Pasar el ensayo del cilindro con obstáculo a escenarios de este formato.

Pruebas:

- Mismo escenario dos veces → grabación idéntica byte a byte.
- Escenario inválido → error claro, sin simular.
- La reproducción coincide con la simulación en todas las muestras.
- Cilindro: obstáculo sobre el centro de masas no vuelca; obstáculo bajo vuelca; el umbral queda entre 1 y 2 veces la energía de la ecuación de pivote (lo ya observado en AnimSpec).
- Fuerza especular (izquierda/derecha) → trayectoria especular.

**Evidencia:** vídeo de tres cilindros (10, 80 y 120 cm) generado solo con el comando y el script de captura.

### Fase 2. Uke paramétrico, tono y caída

1. Generador de MJCF por masa y altura con proporciones de De Leva (tabla pública; no copiar el código de AnimSpec).
2. Límites de par por articulación con un parámetro de fuerza propio, separado de la masa.
3. Actitudes `relajado` y `tenso`.
4. Clasificador de resultado: de pie, recolocado, caído; dirección de caída, primera parte que toca el suelo, tiempo y velocidad de impacto.
5. Golpes como impulso en un punto: pecho, cabeza, abdomen (para la patada frontal y los puñetazos ya animados).

Pruebas:

- Masa total y altura del centro de masas correctas para cuatro tallas (55 kg/1,55 m, 75 kg/1,75 m, 100 kg/1,95 m, 150 kg/2,00 m).
- Las cuatro tallas aguantan 10 s de pie en `tenso`.
- **Prueba de simetría:** fuerza sagital centrada sobre Uke simétrico → desvío lateral del centro de masas < 2 cm y giro < 3°. Es la prueba que habría detectado el fallo del seoi nage.
- Uke `relajado` sin fuerzas: la energía mecánica nunca aumenta (nadie inyecta energía).
- Caída desde una inclinación inicial: el tiempo coincide con el péndulo invertido dentro de una tolerancia.
- Mismo impulso al pecho: Uke más pesado se desplaza menos.

**Evidencia:** patada frontal al pecho con tres impulsos × cuatro tallas, vista frontal y lateral.

### Fase 3. Efectores de Tori y familias de derribo

1. Agarre con holgura, tope y suelta; catálogo semántico de puntos de agarre.
2. Obstáculo fijo y obstáculo móvil con fuerza limitada.
3. Presupuesto de Tori y aviso de «Tori inviable».
4. Un escenario genérico por familia mecánica, con Uke `tenso` y luego `relajado`:

| Familia | Mecanismo | Técnicas que cubre |
|---|---|---|
| Fulcro alto | Tirón delante-arriba + obstáculo a la altura de la cadera | O goshi, seoi nage, harai goshi |
| Segado atrás | Empuje atrás sobre una pierna + segado de esa pierna | Osoto gari, ouchi gari, kouchi gari |
| Barrido | Descarga vertical + fuerza lateral en el pie | De ashi barai, okuri ashi barai |
| Bloqueo de pie | Tirón + obstáculo en el tobillo | Sasae, hiza guruma, tropiezos |
| Piernas | Tirón de corvas + empuje en la cadera | Double leg, single leg |
| Tirón abajo | Fuerza en la nuca o el brazo hacia abajo y un lado | Snap down, arm drag |

Pruebas, las mismas para cada familia:

- **Nominal:** Uke cae hacia la dirección prevista ± 30°.
- **Por debajo del umbral:** con la mitad de fuerza no cae.
- **Ablación:** al quitar el elemento clave (el obstáculo, el segado o el desequilibrio previo) el derribo falla. Demuestra que el mecanismo funciona por la razón correcta.
- **Monotonía:** Uke más pesado necesita más impulso; fulcro por encima del centro de masas no vuelca.
- **Barrido:** el pie empieza a deslizar cuando la fuerza supera rozamiento × (peso − descarga), con error < 10 %.
- **Límites:** ningún efector supera su tope; el presupuesto de Tori se cumple en el caso nominal.
- **Simetría:** la versión especular da el resultado especular.

Las técnicas con nombre, sus tiempos y sus referencias viven en AnimSpec como fichas que apuntan a estos escenarios.

**Evidencia:** por familia, vídeo nominal a velocidad real y a ×0,25 con flechas, más los casos «sin fuerza suficiente» y «ablación».

### Fase 4. Uke reactivo y paso de recuperación

Puede avanzar en paralelo a la fase 3. Trabajo de control difícil: asignarlo al modelo más capaz.

1. Actitud `reactivo`: estrategia de tobillo y cadera (ya existe) y **un** paso de recuperación.
2. Regla de elección del pie: da el paso el **menos cargado**, y se permite cruzar las piernas. La prohibición actual de cruce (`recoveryPlanner.ts`) es el bloqueo más probable de la rama #70.
3. No esperar una transferencia de peso completa antes de levantar el pie: en la reacción humana el pie sale mientras el cuerpo cae hacia él.
4. Actitud `defensivo`.

Límite de esfuerzo: dos intentos con evidencia (cruce permitido; después ley de colocación de pie tipo SIMBICON, es decir, objetivo de cadera corregido por posición y velocidad del centro de masas). Si ninguno da un paso estable, parar y decidir con Jon entre controlador aprendido por refuerzo o Uke sin paso.

Pruebas:

- Matriz de 8 direcciones × 3 impulsos × 4 tallas. Pequeño: recupera sin paso. Medio: un paso y 2 s estable. Grande: cae.
- Los umbrales entre «sin paso», «paso» y «caída» quedan cerca de la teoría del punto de captura (AnimSpec midió 25 N·s recupera y 38 cae hacia delante, con 41 teórico).
- El pie que sale es el menos cargado en el instante de la decisión.
- Sin fuerzas sobre la raíz en ninguna muestra.

**Evidencia:** vídeo de la matriz en mosaico y tabla de resultados con los fallos incluidos.

### Fase 5. Encadenados y postura defensiva

1. Escenarios de dos tiempos: acción previa de Tori → reacción de Uke → derribo.
2. Ventana de oportunidad medida: intervalo en que Uke tiene un pie en el aire o el punto de captura cerca del borde del apoyo.

Prueba principal:

- **A:** derribo directo sobre Uke `defensivo` → no cae.
- **B:** empujón, Uke da el paso, mismo derribo dentro de la ventana → cae.
- **C:** mismo derribo fuera de la ventana (tarde) → no cae.

**Evidencia:** los tres casos en un mismo vídeo, lado a lado.

### Fase 6. Tori con cuerpo (opcional, después)

- Segundo humanoide que sigue una animación de AnimSpec con pares limitados y recibe la reacción de Uke.
- Agarres como restricciones entre dos cuerpos con tope de fuerza.
- Prueba: el resultado sobre Uke se parece al del Tori abstracto de la fase 3 y Tori no pierde el equilibrio en el caso nominal.

### Fase 7. Salida al juego

- Conversor de grabación a rig en AnimSpec (Blender) y de ahí a Unity.
- Petición por lotes: técnica × lista de cuerpos (masa, altura, fuerza, actitud) → un clip por combinación, guardado en AnimSpec con el escenario y la versión del motor que lo generaron.
- Prueba: el clip importado reproduce la grabación (posición de pelvis, manos y pies dentro de una tolerancia) y los pies no atraviesan el suelo ni patinan más que en la simulación.
- Cada clip pasa la revisión visual de Jon antes de darse por bueno.

## 6. Métricas comunes

Toda grabación las calcula, para comparar fases y detectar retrocesos:

- Resultado: de pie / paso y estable / caída, con dirección, tiempo y primera parte que toca.
- Desequilibrio: instante y dirección (las ocho direcciones) en que el punto de captura sale del apoyo.
- Desvío respecto a la dirección prevista del derribo, lateral y en giro.
- Tiempo con los dos pies en el aire y rotación total de Uke.
- Velocidad de impacto de cabeza, espalda y cadera.
- Pico e impulso de cada efector, trabajo total aportado por Tori y cumplimiento de su presupuesto.
- Pares articulares de Uke frente a sus límites (saturaciones).

## 7. Referencias de vídeo

- Fuente para judo con judogi: lista del Kodokan y los vídeos de referencia locales de seoi nage. Viven en el lado privado.
- Por técnica, el agente escribe una **ficha**: duración de cada fase, puntos de agarre, dirección de cada tirón por fase, altura del fulcro, cómo cae Uke (dirección, giro, qué toca primero) y marcas de tiempo del vídeo en que se ve cada cosa.
- La comparación con la referencia es una lista de comprobación cualitativa (duración total, dirección de caída, giro, postura al aterrizar), no una medida métrica: no hay captura de movimiento.
- Si la simulación y la referencia discrepan, se anota en qué, sin retocar fuerzas hasta que «parezca».

## 8. Paquete de evidencias de cada entrega

Cada PR que cambie física entrega a Jon:

1. Vídeo frontal y lateral, a velocidad real y a ×0,25, con flechas de fuerza, centro de masas y apoyo.
2. Hoja de fotogramas clave en una imagen.
3. Tabla de métricas, con los casos que fallan.
4. Tres o cuatro frases: qué se ve bien, qué no es natural, y qué no se ha comprobado.

Los fotogramas intermedios se borran tras codificar y verificar el vídeo (hay poco disco).

## 9. Reglas para los agentes

- CI en verde no es prueba de física. La prueba son las métricas y el vídeo.
- No relajar un umbral para que algo pase. Si un criterio es incorrecto, se discute y se cambia aparte.
- Un cambio por comparación, siempre contra la misma batería de casos.
- Los resultados negativos se registran una vez y no se repiten.
- Nada de barridos ciegos de ganancias: antes de un barrido, escribir la hipótesis y qué resultado la refutaría.
- Sin fuerzas artificiales sobre Uke fuera de los efectores declarados.
- HPL es público: sin nombres de técnicas en el núcleo, sin datos ni modelos de personajes privados, sin código copiado de AnimSpec.
- Cada lección que afecte a los dos repos se anota en ambos.

## 10. Quién hace qué

| Trabajo | Modelo recomendado |
|---|---|
| Diseño de efectores y presupuesto de Tori (fase 3, puntos 1–3) | El más capaz |
| Controlador reactivo y paso (fase 4) | El más capaz |
| Revisión de cada PR de física y de sus vídeos | El más capaz |
| Esquemas, comando, reproducción y captura (fase 1) | Uno más económico |
| Generador paramétrico y clasificador (fase 2) | Uno más económico |
| Escenarios por familia a partir de la plantilla, fichas de técnica (fases 3 y 7) | Uno más económico |

## 11. Riesgos

- **El paso reactivo puede no salir con control clásico.** Dos intentos previos fallaron (AnimSpec en octubre, HPL #70). Por eso tiene límite de esfuerzo y no bloquea las fases 1–3.
- **El Tori abstracto puede dar derribos que un Tori real no podría ejecutar.** El presupuesto lo acota, pero la validación final es la fase 6.
- **Sin tela, los agarres de judogi son aproximados.** La holgura y el tope cubren lo esencial; se revisa si los vídeos lo delatan.
- **Sin datos de fuerza reales por técnica.** Las magnitudes son estimaciones calibradas con literatura general y con las referencias de vídeo.
- **Captura de vídeo desde el visor web** depende de Playwright y del entorno Windows; si resulta frágil, valorar un render sin navegador.
