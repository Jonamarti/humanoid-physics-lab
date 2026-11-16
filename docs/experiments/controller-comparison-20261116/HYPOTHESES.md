# Hipótesis: guía del paso y descarga de apoyo
Antes de ejecutar: la primera batería deja el pie de avance cerca del suelo y no confirma 3 cm de despeje con apoyo contrario suficiente. Hipótesis: el objetivo lateral de pelvis (4,5 cm) y el despeje pedido (6 cm) son insuficientes para este tracker al seguir referencias nuevas.

Un único ensayo adicional: misma duración, avance de 12 cm, límites, motor, semilla y cuerpo; guía lateral de 9 cm y elevación de 10 cm. No se cambian ganancias ni umbrales. Se compara MPC y ProtoMotions con esta misma segunda referencia, conservando la batería original. Se rechaza la hipótesis si no hay descarga, despeje medido, aterrizaje y estabilidad final sin artefactos. Un éxito sería del seguimiento de esta guía; no demostraría recuperación reactiva ante empujones ni marcha general.


## Referencias naturales solicitadas por el usuario

El usuario pide brazos bajos y un paso observable. Se mantiene el mismo modelo y tracker, sin cambiar ganancias, masas, contactos ni límites:

1. Natural v1: aducción de brazos, codos levemente flexionados, balanceo contralateral, avance solicitado de 28 cm. Avanza 20,95 cm y termina estable, pero la revisión de toda la suela no confirma 3 cm. La altura del tobillo era un proxy incorrecto.
2. Natural v2: separar elevar, avanzar y bajar, manteniendo la elevación durante el avance; orientar los dedos hacia arriba en la referencia. Avanza 23,51 cm, despeje máximo 2,78 cm, estable. Sigue sin superar 3 cm.
3. Natural v3: solo aumentar el objetivo de elevación de 10 a 14 cm para compensar el error observado del tracker, sin relajar el criterio real. Avanza 26,69 cm; la suela se eleva 3,67 cm, con apoyo contrario cargado; aterriza y conserva dos segundos estables. La deriva lateral de 5,20 cm excede la tolerancia de 5 cm y el objetivo completo sigue incompleto.

La guía cinemática usa una raíz de referencia, pero esta no se escribe en la raíz física. Solo genera observaciones objetivo para el tracker. El movimiento simulado sale de los pares limitados. La IK de los objetivos naturales es aproximada; no se utiliza como prueba de contacto real.

La comparación inicial de nueve casos no se reemplaza por estas variantes. Los vídeos y revisiones de todas las versiones quedan en carpetas distintas. No se ha entrenado un controlador nuevo ni resuelto todavía recuperación reactiva o marcha encadenada.
