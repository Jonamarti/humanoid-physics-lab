# Evidencia de la corrección de caída de Uke

Base anterior: commit `b8cf20d0cc49a1f24b4973f0f1885795623b83df` (#107). Huella de fuentes finales: `dde09196f16dd015680404d873a31104f2cc6d256dd70437a8583b10d5c642c7`.

- [before_after.json](before_after.json): 40 observaciones, diez escenarios con/sin reflejo antes y después.
- [experiments.csv](experiments.csv): 140 observaciones de las siete etapas, incluidos ensayos rechazados.
- [HYPOTHESES.md](HYPOTHESES.md): hipótesis registradas antes de cada ensayo.
- [integrity.json](integrity.json): mismo escenario, MJCF y tiempos en las veinte comparaciones; diez controles sin reflejo idénticos byte a byte.
- [mutation-check.json](mutation-check.json): las pruebas detectan extensión desde postura plegada, falta de amortiguación y rigidez mantenida tras aterrizar; archivos restaurados exactamente.
- [media-check.json](media-check.json): diez vídeos de 1920x540, a 24 fps, cámara compartida, reproducción x1 y x0,25; todos los fotogramas decodificados.
- [validation.json](validation.json): manifiesto y comprobaciones.

La suite completa pasa: 743 correctas, 13 omitidas y 3 pendientes. El banco manual se ejecutó aparte. Tipos, estilo y build pasan. La CLI estricta del pasivo nominal termina con exit 0 y ningún artefacto, sin cambiar los umbrales. El comparador pasa cuatro comprobaciones, incluida la prohibición de mj_step durante el render.

Se han revisado las diez hojas, con los picos incluidos, no la totalidad de los vídeos en movimiento. La vista frontal de las proyecciones tiene oclusión por Tori; se añaden vistas oblicuas. Los contactos y polígonos del render permiten inspección, no certifican naturalidad humana. El polígono mostrado es de apoyo de pies; sin apoyo de pies no significa ausencia de contacto corporal con el suelo.

El caso pasivo de 75 kg baja de 31,21 a 4,93 kN y de 1,86 a 0,684 cm; el de 120 kg baja de 57,21 a 8,05 kN y de 2,84 a 0,905 cm. Se conserva el empujón recuperable y el apoyo frontal de brazos. Persisten penetraciones de brazos en otras referencias.

Grabaciones, trazas y vídeos completos: `out/fall-response/`. Índice local: `out/fall-response/delivery/index.html`. Las entradas publicadas están en `tests/fixtures/fallResponseCases.json`. No se suben vídeos ni grabaciones a Git.

Arquitectura, límites y reproducción: [UKE_FALL_RESPONSE_20261116.md](../../UKE_FALL_RESPONSE_20261116.md).
