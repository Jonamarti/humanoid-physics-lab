# Dos humanoides articulados: primer hito físico

Continuación de la PR #106, relacionada con #79. Rama `codex/articulated-attacker`. Es una base para controlar físicamente al atacante; todavía no implementa un derribo autónomo ni un paso reactivo.

## Qué cambia

El escenario admite dos actores `humanoid`, cada uno con su árbol articulado completo, masa, proporciones, actitud y raíz libre. El segundo usa el prefijo `<id>_` en los nombres internos del MJCF. Los sitios de los efectores y agarres se declaran con nombres semánticos sin prefijo y el identificador del actor. Se conservan las referencias históricas del primer actor.

`placement: {xy_m: [x,y], heading_deg: ...}` coloca la raíz antes del asentamiento de 0,5 s. No es una trayectoria ni se vuelve a escribir durante la simulación. La colocación del segundo actor es obligatoria. Se rechaza mezclar dos actores articulados con el Tori abstracto de `scenario.tori` y se detectan colisiones de nombres con props/obstáculos. Los parámetros de contacto del segundo humanoide se explicitan para que los valores del primero no se propaguen a él.

Los controladores escriben únicamente en las 28 bisagras de su propio actor. El ejecutor limpia las fuerzas generalizadas una vez y después ejecuta ambos controladores. Cada uno usa su COM real y su propia masa; el equilibrio proyecta el momento deseado sobre los ejes físicos de los tobillos, incluyendo orientaciones de 90 y 180 grados. Las dos raíces se comprueban separadamente y deben tener fuerza generalizada de control cero. No hay mocap, seguimiento externo de pelvis ni segmentos libres independientes.

## Agarres físicos

`actor_grips` conecta dos sitios realmente simulados:

```json
{
  "id": "wrist",
  "source": { "actor": "tori", "site": "right_wrist" },
  "target": { "actor": "uke", "site": "left_wrist" },
  "engage_s": 0,
  "release_s": 2,
  "max_force_n": 200,
  "max_power_w": 100
}
```

Reutiliza el muelle con holgura de #106: adquisición por proximidad, ventana de captura, límite de fuerza y potencia sobre el receptor, deslizamiento por sobrecarga o separación y suelta. Los parámetros opcionales son `capture_radius_m`, `capture_window_s`, `break_distance_m` y `slip_after_s`. No mueve brazos para alcanzar el sitio. Un sitio lejano produce `missed`, no una fuerza a distancia. Fuente y destino deben ser actores distintos; los identificadores de agarre no pueden repetir los de efectores.

Se aplican ambas fuerzas con sus momentos reales sobre los dos antebrazos. Las fuerzas son opuestas en el mismo punto del mundo; para una fuerza central esto equivale a aplicarlas en ambos extremos y conserva el momento total. Se integra el trabajo de los dos lados. La prueba integrada lee `xfrc_applied` y comprueba el balance real de fuerzas y momentos, descontando únicamente el efector externo declarado. No se limita a comparar dos números de telemetría.

El escenario `wrist_load` es un ensayo de carga: aplica explícitamente un pulso externo al esternón del segundo actor. Esa fuerza no representa una técnica generada por el atacante. La conexión transmite un pico de 11,04 N y un impulso lateral de 0,246 N·s, con 9,87 cm de separación inicial. El caso sin agarre también mueve algo al primer actor por contacto; por eso no se atribuye todo su movimiento al agarre.

## Grabación y revisión

- `meta.humanoid_actors`: id, prefijo, cuerpo raíz y control articular.
- `samples[].actors.<id>`: COM, velocidad, momento angular, pares articulares, reacciones de ambos pies, punto de captura, apoyo y margen.
- `summary.actors.<id>`: veredicto independiente, alturas, inclinación, estado del controlador, impactos, diagnósticos y artefactos.
- Los campos históricos superiores conservan el primer actor. `summary.artifacts` agrega los defectos de ambos, con el id en `where`; así `--strict` no oculta un fallo del segundo actor.
- El balance energético se etiqueta `energy_scope: world`: incluye todos los pares y los trabajos de ambos extremos de los agarres. No es energía aislada por actor.
- El visor permite elegir el actor medido. El render nativo dibuja COM/apoyo de ambos y los identifica; la cámara considera ambas raíces.

Las escenas dobles usan control/contactos a 500 Hz independientemente de `record_hz`, incluso si se pide el modo histórico. Incluyen la última muestra física aunque la duración no coincida con un múltiplo del intervalo de grabación. Se ha comprobado igualdad exacta de qpos/qvel y acumulados de agarres a 50 y 125 Hz en Node. El asentamiento no activa agarres ni pulsos.

## Evidencia y resultados negativos

Escenarios en `scenarios/articulated_refs/`; matriz en `tests/manual/articulatedBench.test.ts`; datos compactos en [experiments/articulated-20261115](experiments/articulated-20261115/README.md). Grabaciones y vídeos locales en `out/articulated/`.

La matriz tiene 12 escenas y 24 filas por actor: postura de 10 s, carga con agarre, captura fallida, segundo actor pasivo, masa de 120 kg, altura de 1,95 m, orientación de 90 grados, fricción de 0,2, sin agarre, sin carga, fuera de alcance y caída con protección opcional.

- En la postura nominal de 10 s ambos permanecen de pie: inclinación máxima del tronco 0,157 grados, penetración del pie 0,635 mm y fuerza generalizada sobre ambas raíces cero. Las variantes de masa/talla/orientación/suelo también quedan de pie, sin artefactos detectados.
- La pareja enfrentada en `natural_grip` a 1,04 m tiene 18,69 cm entre muñecas tras asentar. Falla la captura de 12 cm, como debe. Acercar las pelvis puede generar colisiones y separar o cruzar los brazos: no constituye un controlador de alcance. Los intentos geométricos a 0,80 y 0,94 m tampoco adquirieron ese agarre; no se ampliaron los radios para esconderlo.
- El segundo actor relajado cae mientras el primero sigue estable. La caída tiene defectos: pico de contacto 10,94 kN y penetración del suelo 1,28 cm.
- La protección opcional trasladada al segundo actor empeora ese caso: pico 31,21 kN, penetración 1,86 cm, velocidad articular y temblor señalados. No se presenta como mejora general de caídas ni se activa por defecto. Este resultado contradice una extrapolación de los beneficios observados en las dos pistas asistidas de #106.

Los umbrales de artefactos y rangos anatómicos no se han relajado. Que una prueba pase significa que detecta el comportamiento esperado y registra sus defectos; no convierte una caída defectuosa en validación física.

## Reproducir

```powershell
npm.cmd run sim -- scenarios/articulated_refs/wrist_load.json --out out/articulated_cli
$env:HPL_ARTICULATED_BENCH='1'
npm.cmd exec -- vitest run tests/manual/articulatedBench.test.ts --configLoader runner --maxWorkers=1
```

Para generar vídeos, usar el comando documentado en `docs/VIDEO.md` sobre `out/articulated/wrist_load.json.gz`. El render muestra primero velocidad real y luego x0,25.

## Próximo hito y criterios

1. Alcance de manos y cambios de postura mediante objetivos articulares limitados, con ambas pelvis libres. Primero adquirir un agarre enfrentado sin interpenetración inicial; después probar que la reacción desestabiliza al atacante.
2. Transferencia de peso y contacto de pierna/cadera ejecutados por articulaciones del atacante. Mantener controles sin agarre, fuera de alcance y sin contacto corporal para identificar el mecanismo.
3. Repetir las dos familias de derribos con el atacante articulado y comparar con las referencias asistidas congeladas de #106. No declarar equivalencia antes de medir fuerza, impulso, energía, contactos, límites y vídeo frontal/lateral.
4. Investigar por separado la protección que empeora la caída pasiva. El paso reactivo continúa sin validación y no se sustituye por un cambio de etiqueta en el clasificador.

No se integra en Última Ronda ni se modifican los trabajos locales de AnimSpec. Este hito no cierra #77 ni #79.
