# Evidencias de derribos — 15-11-2026

Borrador experimental. [Informe técnico](../../REVISION_FISICA_DERRIBOS_20261115.md). Base: `2f06812586e6bfc06178cd7c20fb16156cdd0b51`. Node v23.7.0; MuJoCo WASM 3.17.0; 500 Hz físicos.

78 filas: 22 de línea base, 22 con las mismas pistas tras agarres/masa y 34 de las referencias finales. El segundo grupo es un estado intermedio, anterior al arreglo de orden de control/grabación; no se presenta como la versión final. Los distintos grupos no constituyen una ablación de una única variable. Las regresiones específicas aíslan los fallos de masa, reacción y muestreo.

Huella normalizada de las fuentes físicas y visuales finales: `cb9838bfdabac6be899e9bdda0ca96c20534e273f49b7ecdf6c15eb121d5f770`. Receta en metrics.json. Las grabaciones y vídeos permanecen localmente en `out/takedowns/delivery` y en las campañas hermanas. No se publican como nuevas grabaciones aprobadas.

[Datos JSON](metrics.json) · [CSV](metrics.csv) · [Mutaciones detectadas](mutation-check.json). La mediana observada del ejecutor en las 34 referencias fue 1.525 s para 6 s simulados en esta máquina; incluye modelo/grabación en memoria y no incluye render. No es un benchmark móvil.

En ambos intentos fuera de alcance: antes hay caída y 400 N por mano; después, de pie, cero fuerza de agarre y cero contacto con Tori. En las referencias nominales finales, la excursión máxima de límite es 1,35° (siega) y 1,09° (piernas). Las versiones con protección reducen los picos globales a 9,68 y 8,40 kN, pero aumentan la penetración del suelo a 2,72 y 2,92 cm. Esos defectos permanecen señalados.

La siega nominal también mide 2,30 cm de penetración Uke–Tori; este diagnóstico adicional se informa aparte y no se confunde con el detector histórico de autocolisión. No hay recuperación con paso validada. Pasivo sin atacante también cae: no se cuenta como éxito técnico.

| Caso | Resultado | Manos N (izq/der) | Suelo cm | Excursión articular ° | Presupuesto Tori viable | Artefactos |
|---|---|---:|---:|---:|---|---|
| siega_exterior_nominal | fall | 353/400 | 1.53 | 1.35 | false | foot_slip|contact_force|ground_penetration|tremor |
| siega_exterior_passive | fall | 338/367 | 1.13 | 0.70 | true | tremor|ground_penetration |
| siega_exterior_defensive | fall | 301/354 | 1.21 | 1.06 | false | foot_slip|contact_force|ground_penetration |
| siega_exterior_heavy | fall | 303/368 | 1.74 | 1.53 | false | foot_slip|contact_force|ground_penetration |
| siega_exterior_light | fall | 357/400 | 1.44 | 1.34 | false | foot_slip|contact_force|ground_penetration |
| siega_exterior_tall | fall | 332/400 | 1.77 | 1.40 | false | contact_force|ground_penetration |
| siega_exterior_slippery | fall | 307/400 | 1.65 | 1.53 | false | contact_force|ground_penetration |
| siega_exterior_weak_grip | fall | 40/40 | 2.04 | 1.32 | false | ground_penetration|contact_force |
| siega_exterior_no_grips | fall | 0/0 | 2.47 | 1.26 | true | contact_force|ground_penetration |
| siega_exterior_out_of_reach | standing | 0/0 | 0.06 | 0.00 | true | ninguno |
| siega_exterior_no_attacker | standing | 0/0 | 0.06 | N/A | N/A | ninguno |
| siega_exterior_protective | fall | 359/400 | 2.72 | 1.24 | false | foot_slip|ground_penetration |
| siega_exterior_small_attacker | fall | 258/298 | 1.67 | 1.42 | false | foot_slip|contact_force|ground_penetration |
| siega_exterior_slow_action | fall | 345/385 | 1.53 | 1.41 | false | foot_slip|ground_penetration |
| siega_exterior_oblique | fall | 289/348 | 1.81 | 1.48 | false | contact_force|ground_penetration|tremor |
| siega_exterior_proportions | fall | 350/400 | 1.50 | 1.45 | false | contact_force|ground_penetration |
| siega_exterior_passive_no_attacker | fall | 0/0 | 0.81 | N/A | N/A | contact_force |
| entrada_piernas_nominal | fall | 283/295 | 1.80 | 1.09 | false | contact_force|ground_penetration |
| entrada_piernas_passive | fall | 93/237 | 1.34 | 0.92 | false | ground_penetration |
| entrada_piernas_defensive | fall | 252/205 | 1.50 | 0.46 | false | contact_force|ground_penetration |
| entrada_piernas_heavy | fall | 286/298 | 1.85 | 1.18 | false | contact_force|ground_penetration |
| entrada_piernas_light | fall | 260/289 | 1.96 | 0.86 | false | ground_penetration |
| entrada_piernas_tall | fall | 248/276 | 2.05 | 1.23 | false | contact_force|ground_penetration |
| entrada_piernas_slippery | fall | 250/290 | 1.72 | 1.14 | false | contact_force|ground_penetration |
| entrada_piernas_weak_grip | fall | 40/40 | 1.97 | 0.79 | false | contact_force|ground_penetration |
| entrada_piernas_no_grips | fall | 0/0 | 1.96 | 0.77 | false | ground_penetration|contact_force |
| entrada_piernas_out_of_reach | standing | 0/0 | 0.06 | 0.00 | true | ninguno |
| entrada_piernas_no_attacker | standing | 0/0 | 0.06 | N/A | N/A | ninguno |
| entrada_piernas_protective | fall | 319/295 | 2.92 | 0.82 | false | ground_penetration |
| entrada_piernas_small_attacker | fall | 209/261 | 1.61 | 1.44 | false | contact_force|ground_penetration |
| entrada_piernas_slow_action | fall | 265/289 | 2.03 | 1.02 | false | ground_penetration|contact_force |
| entrada_piernas_oblique | fall | 320/310 | 1.69 | 1.45 | false | ground_penetration |
| entrada_piernas_proportions | fall | 224/310 | 1.71 | 1.05 | false | contact_force|ground_penetration |
| entrada_piernas_passive_no_attacker | fall | 0/0 | 1.22 | N/A | N/A | ground_penetration |
