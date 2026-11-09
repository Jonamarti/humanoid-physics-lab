# Ensayo clásico: cilindro y obstáculo de altura variable (pre-seoi)

**Estado:** banco de pruebas educativo para un único cuerpo rígido en MuJoCo WASM, sin control muscular ni movimiento humano. La finalidad es estudiar torques, vuelco, fricción y colisiones antes de añadir un humanoide activo.

## Origen del protocolo

Este benchmark reconstruye **independientemente** los parámetros de unas pruebas anteriores conservadas en AnimSpec, en una carpeta privada de trabajo físico aparcado. No se ha copiado el código fuente ni ningún modelo, render o archivo privado. La implementación TypeScript se publica bajo la licencia del laboratorio.

El objetivo histórico era comprender por qué un cuerpo apoyado contra una barrera baja o contra la espalda/cadera de Tori puede pivotar, deslizar o volcar según la altura del obstáculo y la dirección del tirón. **Un cilindro no equivale a una persona:** no puede dar un paso, doblarse, agarrarse ni reaccionar activamente.

## Modelo y fuerza

- Cilindro homogéneo libre: masa **75 kg**, altura **1,75 m**, radio **0,18 m**.
- Gravedad **9,81 m/s²** y rozamiento deslizante de referencia **0,6**, geometría MuJoCo nativa.
- Obstáculo fijo, profundidad **0,12 m**, separado **0,02 m** del borde del cilindro.
- Alturas seleccionables: **0,10, 0,50, 0,80, 0,90 y 1,20 m**.
- Fuerza hacia delante **+X**, con componente vertical `Fz = ratio * Fx`, aplicada a una altura local de **1,40 m** (en la orientación actual del cilindro).
- Comienza a `t = 0,30 s` y dura por defecto **0,15 s**; por defecto `Fx=600 N` y `ratio=0,3`.
- Timestep físico de **0,001 s**, independiente del ritmo de render.
- Al tratarse de un cuerpo libre, la fuerza aplicada en punto excéntrico añade un par `τ = (p - CM) × F`, a través de `data.xfrc_applied` de MuJoCo. Cada paso limpia el wrench anterior antes de aplicar el correspondiente.
- `Fx` se refiere a la componente **horizontal**; el módulo total de fuerza es `Fx * sqrt(1+ratio²)`. El impulso horizontal de fuerza constante es `Jx = Fx * duración`.

## Comparación con la predicción energética

Se muestra una comparación orientativa de un **pivote ideal, fijo** en el borde superior del obstáculo. A partir de la fuerza integrada en el tiempo, se calcula el momento angular impulsivo `L`, la inercia `I_pivote = I_CM + m * |r_CM-pivote|²` y la energía cinética rotacional `L²/(2 I_pivote)`. La barrera gravitatoria se estima a partir de la elevación del CM hasta la vertical del pivote.

**La predicción es aproximada**: durante la simulación real el punto de contacto puede deslizarse, cambiar, perderse o experimentar choques. El cuerpo puede girar alrededor del pie, rebotar, pasar por encima o quedar apoyado. No interpretar discrepancias entre energía ideal y MuJoCo como un error por sí solas.

Se considera «vuelco» cuando la orientación del eje del cilindro supera **90° hacia delante**. La UI muestra el máximo ángulo **muestreado por fotogramas de render**, por lo que no garantiza capturar cada máximo intermedio; un futuro runner de simulación deberá registrar ángulo **en cada timestep**. Tampoco supone haber validado ningún seoi-nage humano.

## Uso en web

1. Abrir el laboratorio en GitHub Pages.
2. En **Escenario físico**, seleccionar **Cilindro + obstáculo**.
3. Ajustar **fuerza horizontal**, **duración**, **altura del obstáculo** y **componente vertical**.
4. Pulsar **Cargar simulación** para compilar el escenario con esos parámetros; **Iniciar** aplica el tirón a los 0,30 s.
5. Girar y acercar la cámara con el ratón o desde el móvil; **Pausar** y **Reiniciar** para repetir el ensayo.

Cambiar parámetros exige pulsar **Cargar simulación** de nuevo: la ejecución usa una copia fija del ensayo para evitar cambios a mitad del impulso.

## Pruebas autónomas

`tests/unit/cylinderBenchmark.test.ts` verifica valores y dimensiones, límites de parámetros, ventana de fuerza, signos de giro, predicción de pivote y carga en MuJoCo real de las cinco alturas históricas. Compara también una trayectoria con tirón y otra sin fuerza en WASM (sin mocks).

**Próximos pasos:** integrar métricas por cada timestep y un barrido automático de `Fx=[300,600,1000,1500]` N, `ratio=[0.3,1]`, alturas históricas; generar informe CSV con contacto, deslizamiento, ángulo máximo, energía y diferencias. Después incorporar perfil `kuzushi → tsukuri → kake` en lugar del único pulso; no necesitará que el usuario abra Blender.

**Trabajo en paralelo:** #8–#9 sigue siendo el objetivo principal de control activo del humanoide. Este experimento cumple un peldaño de #10 y #16 sin sustituirlos.
