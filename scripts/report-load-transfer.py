"""Render measured issue-70 JSON using matplotlib; no synthetic trajectories.

uv run --with matplotlib python scripts/report-load-transfer.py RESULTS_ROOT OUTPUT_DIR
"""
import argparse
import base64
import csv
import html
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("results", type=Path)
parser.add_argument("output", type=Path)
parser.add_argument("--repo-evidence", type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
variants = ["baseline", "latency-40ms", "lookahead-200ms", "forecast200-latency40",
            "forecast200-latency40-ramp80", "forecast200-latency40-hip012",
            "forecast200-latency40-hip024", "forecast200-latency40-hipMinus012",
            "forecast200-latency40-ankleDamping01"]
summaries = {v: json.loads((args.results/v/"summary.json").read_text(encoding="utf-8"))
             for v in variants if (args.results/v/"summary.json").exists()}
rows, inspect, root_samples = [], {}, 0
for variant, campaign in summaries.items():
    inspect[variant] = {}
    for r in campaign["rows"]:
        raw = json.loads((args.results/variant/(r["label"]+".json")).read_text(encoding="utf-8"))
        trace, s, intent = raw["trace"], r["step"], r["intent"]
        root_samples += len(trace)
        assert all(all(x == 0 for x in t["control"]["freeRootApplied"] +
                       t["control"]["pelvisAppliedWrench"]) for t in trace)
        flight = [t for t in trace if t["phase"] in ["LIFT", "SWING", "LAND"]]
        start = next((t for t in trace if intent and t["timeS"] >= intent["timeS"]-1e-8), None)
        def displacement(t):
            return [t["swingSole"][i]-start["swingSole"][i] for i in range(3)]
        row = dict(variant=variant, case=r["label"], peakN=r["peakN"], impulseNs=r["impulseNs"],
                   detectionS=r["firstWarningS"], actualCaptureExitS=r["firstActualCaptureExitS"],
                   requestS=intent["timeS"] if intent else None,
                   support=intent["supportSide"] if intent else None,
                   supportAtRequestN=r["intentLoads"]["supportN"] if intent else None,
                   swingAtRequestN=r["intentLoads"]["swingN"] if intent else None,
                   requestedStepM=sum(x*x for x in intent["deltaXY"])**.5 if intent else None,
                   shiftDurationS=r["shiftDurationS"], maxLoadFractionInShift=r["maxSupportFractionDuringShift"],
                   fsmDwellS=s["longestPreLiftReadyDurationS"] if s else 0,
                   strictTransferS=r["strictTransferDurationS"], readySamples=r["maxReadySamples"],
                   liftS=r["firstLiftS"], swingS=r["firstSwingS"], validFlightS=r["maxFlightS"],
                   soleRiseInLiftM=max((displacement(t)[2] for t in flight), default=None),
                   soleRiseIncludingFallM=s["maxSoleRiseM"] if s else None,
                   soleTravelIncludingFallM=s["maxSoleTravelXYM"] if s else None,
                   landingS=s["firstLandingTimeS"] if s else None,
                   supportMarkerDriftM=r["supportDriftM"] if intent else None,
                   landingMarkerDriftM=r["landingDriftM"] if s and s["stepContactConfirmed"] else None,
                   targetErrorM=s["targetErrorXYM"] if s else None,
                   finalSpeedMps=r["finalSpeedMps"], minPelvisM=r["minPelvisM"], maxTiltDeg=r["maxTiltDeg"],
                   postStabilityS=r["postStabilityS"], outcome=r["finalOutcome"], complete=r["complete"])
        rows.append(row)
        inspect[variant][r["label"]] = [[round(t["timeS"], 3),t["phase"],
            *[round(x, 6) for x in [t["leftN"],t["rightN"],t["com"][0],t["com"][1],
                t["capture"][0],t["capture"][1],*t["velocity"][:2],t["tiltDeg"],t["pelvisM"],
                t["fsmReadyDurationS"],t["strictReadyDurationS"]]]] for t in trace]

with (args.output/"comparacion.csv").open("w",newline="",encoding="utf-8-sig") as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
(args.output/"campaign.json").write_text(json.dumps(dict(rows=rows,rootZeroSamples=root_samples),indent=2),encoding="utf-8")

def read_trial(variant, label):
    return json.loads((args.results/variant/(label+".json")).read_text(encoding="utf-8"))

phases={p:i for i,p in enumerate(["MONITORING","WARNING","NO_SAFE_STEP","STAND","SHIFT_WEIGHT",
                                "LIFT","SWING","LAND","RECOVER","COMPLETE","ABORT","FALL"])}
plt.rcParams.update({"font.size":9,"axes.grid":True,"grid.alpha":.2})
fig,axs=plt.subplots(5,2,figsize=(14,12),sharex=True,layout="constrained")
for col,(variant,title) in enumerate([("baseline","Baseline"),("forecast200-latency40","Anticipación 200 ms + latencia 40 ms")]):
    raw=read_trial(variant,"left-300");ts=raw["trace"];t=[p["timeS"] for p in ts]
    axs[0,col].set_title(title+" · empujón +Y 300 N")
    for key,label in [("leftN","Izquierdo: libre solicitado"),("rightN","Derecho: apoyo elegido")]:
        axs[0,col].plot(t,[p[key] for p in ts],label=label,lw=1)
    axs[0,col].set_ylabel("Carga vertical (N)");axs[0,col].set_ylim(0,1800);axs[0,col].legend(fontsize=8)
    for key,label in [("com","COM Y"),("capture","Capture Y")]:
        axs[1,col].plot(t,[p[key][1] for p in ts],label=label)
    axs[1,col].plot(t,[p["control"]["referenceComXY"][1] for p in ts],label="Referencia COM Y",ls="--")
    axs[1,col].set_ylabel("Posición lateral (m)");axs[1,col].legend(fontsize=8)
    axs[2,col].plot(t,[100*(p["readiness"]["supportFraction"] if p["readiness"] else 0) for p in ts],label="Fracción apoyo elegido")
    axs[2,col].axhline(72,color="r",ls="--",lw=1,label="Puerta 72 %");axs[2,col].set_ylim(-2,102)
    axs[2,col].set_ylabel("Carga de apoyo (%)");axs[2,col].legend(fontsize=8)
    axs[3,col].plot(t,[p["tiltDeg"] for p in ts],label="Inclinación (°)")
    axs[3,col].axhline(25,color="r",ls="--",lw=1);axs[3,col].set_ylabel("Inclinación (°)")
    ax2=axs[3,col].twinx();ax2.plot(t,[p["pelvisM"] for p in ts],color="green",label="Pelvis (m)");ax2.set_ylabel("Pelvis (m)")
    axs[4,col].step(t,[phases[p["phase"]] for p in ts],where="post")
    axs[4,col].set_yticks(list(phases.values()),list(phases));axs[4,col].set_ylabel("FSM");axs[4,col].set_xlabel("Tiempo físico (s)")
    for ax in axs[:,col]:
        ax.set_xlim(.9,3.1)
        ax.axvspan(1,1.16,color="orange",alpha=.12)
        ax.axvline(raw["summary"]["intent"]["timeS"],color="black",ls=":",lw=1)
fig.suptitle("MuJoCo v3 · 75 kg · Δt=2 ms · datos resueltos a 50 Hz · ninguna transferencia/despegue válidos")
fig.savefig(args.output/"comparacion-lateral.png",dpi=150);plt.close(fig)

def n(x, digits=3): return "—" if x is None else f"{x:.{digits}f}"
def table(data, keys):
    return '<div class="scroll"><table><thead><tr>'+''.join(f'<th>{html.escape(k)}</th>' for k in keys)+\
      '</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(n(r[k]) if isinstance(r.get(k),(float,int)) and not isinstance(r.get(k),bool) else str(r.get(k) if r.get(k) is not None else '—'))+'</td>' for k in keys)+'</tr>' for r in data)+'</tbody></table></div>'

comparison=[]
for r in summaries["baseline"]["rows"]:
    v=next(x for x in summaries["forecast200-latency40"]["rows"] if x["label"]==r["label"])
    comparison.append({"Caso":r["label"],"Impulso N·s":r["impulseNs"],"Baseline":r["finalOutcome"],
                       "Solicitud baseline s":r["intent"]["timeS"] if r["intent"] else None,
                       "Variante":v["finalOutcome"],"Solicitud variante s":v["intent"]["timeS"] if v["intent"] else None,
                       "Transferencia válida s":v["strictTransferDurationS"],"LIFT":v["firstLiftS"]})
image64=base64.b64encode((args.output/"comparacion-lateral.png").read_bytes()).decode()
validation={}
if (args.output/"validation.json").exists():validation=json.loads((args.output/"validation.json").read_text(encoding="utf-8-sig"))
github={}
if (args.output/"github.json").exists():github=json.loads((args.output/"github.json").read_text(encoding="utf-8-sig"))
report=f'''<!doctype html><html lang="es"><meta charset="utf-8"><title>Humanoid Physics Lab · Issue 70 · Informe medido</title>
<style>body{{max-width:1250px;margin:32px auto;padding:0 22px;font:16px/1.6 system-ui;color:#173047;background:#f6f9fc}}h1,h2{{line-height:1.25}}table{{border-collapse:collapse;font-size:13px;background:white}}td,th{{padding:8px;border:1px solid #ccd7e3;text-align:left;white-space:nowrap}}th{{background:#e8eff6}}.scroll{{overflow:auto}}img{{width:100%}}.negative{{padding:18px;background:#fff1e4;border-left:5px solid #c96018}}pre{{white-space:pre-wrap;background:#e9eff5;padding:14px}}video{{width:49%;min-width:300px}}select,input{{padding:8px;margin:8px}}a{{color:#13569d}}</style>
<h1>Humanoid Physics Lab · Recuperación física de uke</h1><p>11 de noviembre de 2026 · Base <code>90727886d51e117fc0ca47716878ac0ecbee376d</code> · Issue <a href="https://github.com/Jonamarti/humanoid-physics-lab/issues/70">#70</a>.</p>
<p class="negative"><b>Resultado negativo: 0 recuperaciones físicas completas en {len(rows)} ensayos ({len(summaries)} campañas de 12 casos).</b> Cada campaña conserva 8 casos estables sin solicitud de paso y 4 caídas. No se alcanzó el nivel 1, ni los hitos A, B o C. Una suela elevada durante la caída no se contabiliza como vuelo.</p>
<h2>A. Diagnóstico: hechos e hipótesis</h2>
<p>El planificador ya penaliza levantar el pie más cargado. El fallo lateral no se explica por ignorar ese dato: exige apoyo bilateral instantáneo y un destino alcanzable sin cruce; no predice si el apoyo elegido conservará carga durante la transferencia. En +Y, la geometría favorece mover el pie izquierdo hacia fuera y usar el derecho, aunque el derecho esté perdiendo carga.</p>
<p>Baseline +Y 300 N: primera advertencia en 1,28 s (izquierdo 700,05 N, derecho 35,49 N). El derecho queda en 0 N en 1,30 s. El planificador emite NO_SAFE_STEP desde 1,40 s y acepta una solicitud en 1,50 s durante un pico transitorio: derecho 1580,13 N, izquierdo 919,79 N. En 1,52 y 1,54 s, el apoyo elegido vuelve a 0 N; en 1,54 s el pie que se pretendía levantar soporta 603,41 N. La FSM bloquea correctamente el despegue.</p>
<p>Con anticipación de 200 ms y latencia de 40 ms, la solicitud llega en 1,16 s, con derecho 162,89 N e izquierdo 565,10 N (22,4 % en el futuro apoyo). El COM aún avanza lateralmente a 0,388 m/s. Adelantar la solicitud 340 ms no obtiene carga unilateral sostenida. La referencia del COM se dirige al lado correcto (Y negativo para apoyo derecho), pero el COM continúa hacia Y positivo. Las saturaciones de tobillo de 160 N·m aparecen también en el pie descargado.</p>
<p><b>Hipótesis pendientes:</b> el reparto bilateral de momentos y la rigidez/amortiguación del tobillo descargado pueden contribuir a la pérdida de contacto. Reducir solo la amortiguación activa a 0,1 tampoco produce transferencia. Estos datos no demuestran que los 300 N sean físicamente irrecuperables, ni que falte fuerza muscular: hace falta un análisis de viabilidad dinámica de contacto.</p>
<h2>B. Implementación y alcance</h2><p>Se añadieron opciones exclusivamente experimentales de latencia, pronóstico lineal de COM, rampa lateral, postura bilateral de cadera y amortiguación de tobillo; todas mantienen los valores anteriores por defecto. La latencia de decisión se separó del horizonte predictivo. Se conserva la configuración al reiniciar. Se añadió telemetría de referencia, pares limitados y fuerzas sobre la raíz, un benchmark reproducible, pruebas de determinismo y un grabador frontal/lateral.</p>
<p>Archivos: <code>src/control/reactiveStepCoordinator.ts</code>, <code>recoveryPlanner.ts</code>, <code>experimentalStep.ts</code>, <code>src/physics/mujoco.ts</code>, pruebas unitarias y <code>tests/manual/loadAwareRecoveryBench.test.ts</code>, scripts de captura/informe. En <code>src/main.ts</code> se corrige además el refresco del editor de fuerzas tras preparar v3: los campos quedaban deshabilitados después del cambio de modelo.</p>
<p>No cambian XML, masas, fricción nominal, colisiones, timestep, límites de par ni umbrales de aceptación. Permanecen 72/28 %, total &gt;250 N, velocidad &lt;1,2 m/s, pelvis &gt;0,82 m, inclinación &lt;25° y 80 ms continuos; también se conserva la espera mínima de 250 ms en SHIFT_WEIGHT. El criterio adicional de COM sobre el pie elegido se comprueba independientemente: la puerta actual usa el polígono conjunto cargado.</p>
<h2>C. Comparación de los 12 escenarios</h2><p>Pulso de medio seno entre 1,00 y 1,16 s en esternón; 6 s físicos por ensayo; 75 kg, Δt=0,002 s. Variante comparada: anticipación 200 ms + latencia 40 ms. NO_STEP indica equilibrio sin solicitar paso; no equivale a COMPLETE. Los impulsos se integraron sobre la fuerza efectivamente aplicada a 500 Hz.</p>
{table(comparison,list(comparison[0]))}
<p>Árbol A/B: latencia 40 ms frente a baseline; anticipación 200 ms frente a baseline; después latencia 40 ms frente a anticipación 200 ms; rampa 80 ms frente a esa combinación; cadera +0,12 rad frente a la combinación; +0,24 rad frente a +0,12; −0,12 rad frente a +0,12; amortiguación 0,1 frente a la combinación sin cadera. Se varía un parámetro por comparación. No se repite la hipótesis de hombros.</p>
<details><summary>Todos los ensayos y métricas principales ({len(rows)} filas)</summary>{table(rows,['variant','case','requestS','supportAtRequestN','swingAtRequestN','shiftDurationS','maxLoadFractionInShift','fsmDwellS','strictTransferS','liftS','swingS','finalSpeedMps','minPelvisM','maxTiltDeg','outcome'])}</details>
<h2>D. Evidencia física</h2><img alt="Cargas, COM, captura, inclinación, pelvis y FSM sincronizados" src="data:image/png;base64,{image64}">
<p>Las trazas completas y sus qpos están en <code>physics/&lt;campaña&gt;/&lt;caso&gt;.json</code>; contienen la evolución a 0/20/40/80/120 ms desde la solicitud, fuerzas normales, COM/velocidad/captura, rodillas reales, pares y puertas de la FSM. <a href="comparacion.csv">CSV de todas las métricas</a> · <a href="campaign.json">JSON agregado</a> · <a href="comparacion-lateral.png">Gráfica exportable</a>.</p>
<p>Se verificaron {root_samples} muestras con fuerzas generalizadas de raíz y wrench externo de pelvis exactamente nulos. Los invariantes del controlador también comprueban la raíz en cada paso de integración. Los contactos leídos tienen el retardo de ≤2 ms del integrador existente. La deriva de marcadores de suela se registra como proxy conservador de desplazamiento; no se presenta como medición exacta del deslizamiento material. Sin aterrizaje, la deriva de aterrizaje es N/A. No existe un aterrizaje válido en esta batería.</p>
<video controls preload="metadata" src="videos/baseline-left300-front.webm"></video><video controls preload="metadata" src="videos/early-left300-side.webm"></video>
<p>También se grabaron ambas vistas de frontal 200 N (FALL) y frontal 120 N (sin paso). Las grabaciones ejecutan MuJoCo de nuevo a timestep fijo: no reproducen pies animados ni modifican qpos. Los gráficos en vídeo recortan la escala de presentación; los JSON y PNG conservan los valores medidos.</p>
<h3>Inspector de muestras a 50 Hz</h3><select id="variant"></select><select id="case"></select><input id="sample" type="range" min="0" max="299" value="75"><pre id="sample-data"></pre>
<h2>E. Hitos y límites</h2><p>Nivel 1: 0 intervalos válidos de 80 ms; máximo de muestras listas consecutivas: 0. Nivel 2: ningún LIFT/SWING legítimo. Niveles 3/4: ningún aterrizaje ni COMPLETE. A no alcanzado, por lo que no hay éxito que repetir tres veces. B pendiente: no se han seleccionado fuerzas exitosas ni ejecutado sus perturbaciones ±10 %. C pendiente: no se define un porcentaje de robustez ni se reclasifican retrospectivamente caídas como irrecuperables.</p>
<p>Las 12 trayectorias por defecto coinciden exactamente con el baseline previo a estos cambios en todas las qpos muestreadas a 50 Hz. La prueba de reset repite también la trayectoria experimental completa de forma determinista. La repetibilidad de una caída no acredita recuperación.</p>
<h2>Regresiones y comprobaciones</h2><pre>{html.escape(json.dumps(validation,ensure_ascii=False,indent=2))}</pre>
<p>V3 nominal: 10 s, pelvis mínima 0,9896 m, inclinación máxima 0,0494°, apoyo bilateral 10 s y carga final 735,75 N. V2 nominal: COMPLETE con ambos pies, deslizamiento de aterrizaje 1,22/1,21 cm, apoyo 0,59 cm y error objetivo 4,96/4,98 cm. En fricción 0,9 y 1,1 persisten fallos conocidos; no se presentan como robustez adquirida. No hay nuevas confirmaciones falsas de aterrizaje en los ensayos reactivos.</p>
<h2>F. GitHub y despliegue</h2><pre>{html.escape(json.dumps(github,ensure_ascii=False,indent=2))}</pre><p>La PR #62 sigue abierta, draft, no-merge y sin fusionar. La investigación no se despliega ni se fusiona. Los umbrales intactos y el CI verde no certifican estabilidad biomecánica.</p>
<h2>G. Siguiente bloqueo y experimento verificable</h2><p>El bloqueo es sostener físicamente el apoyo elegido mientras se frena el COM exterior. La selección por carga instantánea y huella futura acepta apoyos transitorios. Próximo experimento: registrar fuerzas, COP, ángulos/velocidades y saturación de tobillo a 500 Hz alrededor de la solicitud; calcular la viabilidad de transferencia a 120 ms con contactos y límites de par. Comparar un único cambio de reparto de momentos según cargas frente a la anticipación ya medida. Predefinir aceptación: ≥72 % durante 80 ms, COM sobre apoyo elegido, mismos límites de velocidad/altura/inclinación y sin desplazamiento artificial de raíz. Si no hay apoyo y objetivo compatibles, mantener NO_SAFE_STEP; no contar suprimir la solicitud como un paso recuperado.</p>
<script>const data={json.dumps(inspect,separators=(',',':'))}; const v=document.querySelector('#variant'),c=document.querySelector('#case'),s=document.querySelector('#sample');Object.keys(data).forEach(x=>v.add(new Option(x,x)));function show(){{const a=data[v.value][c.value][+s.value];document.querySelector('#sample-data').textContent=JSON.stringify(Object.fromEntries(['t','fase','izquierdaN','derechaN','COM_X','COM_Y','CP_X','CP_Y','velX','velY','inclinacionDeg','pelvisM','dwellFSM','dwellEstricto'].map((x,i)=>[x,a[i]])),null,2)}}function cases(){{c.replaceChildren();Object.keys(data[v.value]).forEach(x=>c.add(new Option(x,x)));c.value='left-300';show()}}v.onchange=cases;c.onchange=show;s.oninput=show;cases();</script></html>'''
(args.output/"informe.html").write_text(report,encoding="utf-8")
if args.repo_evidence:
    args.repo_evidence.mkdir(parents=True,exist_ok=True)
    compact=re.sub(r'<h3>Inspector.*?<h2>E\.', '<h2>E.', report, flags=re.S)
    compact=re.sub(r'<script>.*?</script>', '', compact, flags=re.S)
    compact=re.sub(r'<video.*?</video>', '', compact, flags=re.S)
    compact=compact.replace('data:image/png;base64,'+image64,'comparacion-lateral.png')
    (args.repo_evidence/"informe.html").write_text(compact,encoding="utf-8")
    for filename in ["comparacion.csv","campaign.json","comparacion-lateral.png"]:
        (args.repo_evidence/filename).write_bytes((args.output/filename).read_bytes())
print(json.dumps({"trials":len(rows),"rootZeroSamples":root_samples,"complete":sum(r["complete"] for r in rows)}))
