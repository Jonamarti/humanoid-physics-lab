import "./styles.css";
import { impulseFromConstantForce } from "./physics/impulse";
import { GRAVITY_DEMO_MJCF } from "./physics/demoModel";
import { FixedClock } from "./physics/fixedClock";
import { MujocoSimulation, type PhysicsState } from "./physics/mujoco";
import { BallViewer } from "./renderer/ballViewer";
import { forceDisplayForFrame } from "./renderer/forceDisplay";
import { installForceEditor } from "./editor/forceEditor";
import { projectBalanceToGround } from "./renderer/balanceProjection";
import { STEP_STABILITY_LABELS } from "./control/stepStabilityDiagnostics";
import { CYLINDER, cylinderMJCF, cylinderForceAt, cylinderForwardTiltDeg, idealPivotPrediction, localGripPoint, validateCylinderTrial, type CylinderTrial } from "./physics/cylinderBenchmark";
import { expectedPushImpulseNs, pushForceAtTime, rootTiltDegrees, validatePushTrial, type PushDirection, type PushTrial } from "./physics/standingPush";

const app = document.querySelector<HTMLDivElement>("#app");
if (!app) throw new Error("No existe el contenedor de la aplicacion");

app.innerHTML = `
  <main class="layout">
    <header class="header">
      <span class="eyebrow">Proyecto abierto · TypeScript · Control determinista</span>
      <h1>Humanoid Physics Lab</h1>
      <p>Laboratorio para desarrollar equilibrio activo, pasos de recuperación y caídas articuladas con física real.</p>
    </header>
    <section class="columns" aria-label="Estado del laboratorio">
      <article class="panel preview">
        <div id="physics-viewport" class="viewport" role="group" aria-label="Visor 3D: giro y zoom táctil de cuerpos físicos MuJoCo"></div>
        <div class="status"><span class="dot"></span> MuJoCo WASM + Three.js — escenario pasivo y primer controlador de equilibrio</div>
        <div class="controls" aria-label="Control de simulación">
          <label for="physics-model">Escenario físico</label>
          <select id="physics-model" aria-label="Escenario físico">
            <option value="ball">Esfera — prueba de gravedad</option>
            <option value="humanoid">Humanoide — ragdoll pasivo, sin equilibrio</option>
            <option value="standing">Humanoide — equilibrio activo experimental (10 s)</option>
            <option value="cylinder">Cilindro + obstáculo — ensayo anterior al seoi-nage</option>
          </select>
          <p id="physics-status" role="status" aria-live="polite">Motor sin cargar.</p>
          <p id="physics-readout" class="result">t = 0.000 s · altura 1.500 m</p>
          <p id="balance-readout" class="muted" hidden></p>
          <p id="ground-projection-readout" class="muted" hidden></p>
          <div class="button-row">
            <button id="physics-load" type="button">Cargar simulación</button>
            <button id="physics-toggle" type="button" disabled>Iniciar</button>
            <button id="physics-reset" type="button" disabled>Reiniciar</button>
          </div>
        </div>
      </article>
      <article class="panel controls">
        <div id="standing-push-controls" hidden>
          <h2>Probar un empujón real</h2>
          <div id="multi-force-editor"></div>
          <p class="muted">El impacto se aplica en el esternón del humanoide, mediante una fuerza externa en MuJoCo. No se fija la pelvis ni se mueve ningún pie artificialmente.</p>
          <label for="push-direction">Dirección</label>
          <select id="push-direction">
            <option value="forward">Hacia delante (+X)</option>
            <option value="backward" selected>Hacia atrás (-X)</option>
            <option value="left">Hacia la izquierda (+Y)</option>
            <option value="right">Hacia la derecha (-Y)</option>
            <option value="forward-left">Diagonal: delante-izquierda (+X,+Y)</option>
            <option value="forward-right">Diagonal: delante-derecha (+X,-Y)</option>
            <option value="backward-left">Diagonal: atrás-izquierda (-X,+Y)</option>
            <option value="backward-right">Diagonal: atrás-derecha (-X,-Y)</option>
          </select>
          <label for="push-peak">Fuerza máxima del empujón</label>
          <input id="push-peak" type="number" min="0" max="10000" step="10" value="240" inputmode="decimal" aria-describedby="push-force-hint" />
          <p class="muted" id="push-force-hint">Fuerza pico de 0 a 10.000 N; no es un límite seguro para personas. La flecha ámbar indica el vector elegido (antes y después del empujón).</p>
          <label for="push-duration">Duración del empujón (ms)</label>
          <input id="push-duration" type="number" min="10" max="1000" step="10" value="120" inputmode="numeric" />
          <p class="muted">La fuerza sigue un pulso sinusoidal: al aumentar la duración, aumenta el tiempo de aplicación y también el impulso.</p>
          <p id="push-phase" class="force-phase" role="status" aria-live="off">LISTO · 0 N aplicados</p>
          <div class="force-meter" aria-hidden="true"><div id="force-meter-fill" class="force-meter-fill"></div></div>
          <p class="muted">Cuando actúa, la flecha se ilumina y aparece un halo rojo. El indicador muestra la fuerza medida en los pasos físicos del fotograma.</p>
          <p class="muted" id="push-vector-readout" role="status" aria-live="off">Vector del empujón: selecciona una dirección y ejecuta el ensayo.</p>
          <button id="push-run" type="button" disabled>Reiniciar y probar empujón (6 s)</button>
          <p class="muted" id="push-readout">La fuerza se aplica entre t=1,00 y 1,12 s. La recuperación se observa durante el resto de la simulación.</p>
          <p class="muted">El diagnóstico visual es aproximado. Las pruebas automatizadas registran resultados cada 0,04 s.</p>
          <section class="step-experiment" aria-label="Ensayo experimental de un paso físico">
            <h2>Primer paso físico experimental</h2>
            <p class="muted">Ensaya un movimiento voluntario con transferencia real de peso, flexión de pierna y contacto. Aún NO es una recuperación automática ante empujones.</p>
            <label for="step-foot">Pie que se mueve</label>
            <select id="step-foot">
              <option value="left">Izquierdo</option>
              <option value="right">Derecho</option>
            </select>
            <label for="step-direction">Dirección del paso</label>
            <select id="step-direction">
              <option value="outward">Hacia fuera, lateral</option>
              <option value="forward" selected>Hacia delante</option>
              <option value="backward">Hacia atrás</option>
              <option value="diagonal">Diagonal hacia delante y fuera</option>
            </select>
            <label for="step-profile">Coordinación de tronco, cadera y rodilla</label>
            <select id="step-profile">
              <option value="0">Anterior — sin inclinación coordinada</option>
              <option value="1" selected>Experimental — tronco adelante, elevar muslo, flexionar rodilla</option>
            </select>
            <p class="muted">El perfil experimental adelanta ligeramente el centro de masas y flexiona la rodilla hacia atrás al levantar el muslo; los contactos y las caídas siguen siendo físicos. No es todavía una marcha estable ni una reacción automática al empujón.</p>
            <label for="step-cartesian">Control cartesiano de pie (experimental)</label>
            <select id="step-cartesian">
              <option value="0">Desactivado — control articular anterior</option>
              <option value="0.2" selected>20 % — corrección leve (mejor ensayo observado)</option>
              <option value="0.5">50 % — corrección media</option>
              <option value="1">100 % — corrección máxima experimental</option>
            </select>
            <p class="muted">El 20 % es el ensayo con rodilla corregida que consigue adelantar y apoyar el pie sin caerse; todavía acaba en ABORT por precisión y deslizamiento. Otras ganancias pueden provocar caídas.</p>
            <button id="step-run" type="button" disabled>Reiniciar y ensayar un paso (5,2 s)</button>
            <p id="step-readout" class="muted" role="status" aria-live="off">El paso todavía no se ha ensayado.</p>
            <p id="step-gates-readout" class="muted">Condiciones pendientes: primero debe despegar y apoyar físicamente.</p>
          </section>
        </div>
        <div id="rigid-force-controls">
        <h2>Ensayo de fuerza y vuelco</h2>
        <p class="muted" id="force-description">En el modo cilindro, estos parámetros se aplican a la física real. En los otros modos solo calculan el impulso.</p>
        <label for="force">Fuerza constante (N)</label>
        <input id="force" type="number" value="600" min="0" max="3000" step="25" />
        <label for="duration">Duración (ms)</label>
        <input id="duration" type="number" value="150" min="0" max="1000" step="10" />
        <p class="result" id="impulse" role="status" aria-live="polite">Impulso horizontal: 90.0 N·s</p>
        <div id="cylinder-controls" hidden>
          <label for="obstacle-height">Altura del obstáculo</label>
          <select id="obstacle-height">
            <option value="0.10">10 cm — obstáculo bajo</option>
            <option value="0.50">50 cm</option>
            <option value="0.80" selected>80 cm — altura aproximada de cadera</option>
            <option value="0.90">90 cm</option>
            <option value="1.20">120 cm — encima del centro de masas</option>
          </select>
          <label for="upward-ratio">Componente vertical del tirón</label>
          <select id="upward-ratio">
            <option value="0.3">Hacia arriba: 30 % de F horizontal</option>
            <option value="1">Hacia arriba: 100 % de F horizontal</option>
            <option value="0">Horizontal: 0 %</option>
          </select>
          <p class="muted" id="pivot-prediction">Predicción analítica pendiente.</p>
          <p class="muted">Para cambiar parámetros de una simulación ya cargada, pulsa <strong>Cargar simulación</strong> otra vez.</p>
        </div>
        <p class="muted">Este cilindro es un cuerpo rígido: no camina ni tiene articulaciones. La predicción de pivote es idealizada; la simulación resuelve el contacto con MuJoCo. Equilibrio humano: #8–#9.</p>
        </div>
      </article>
    </section>
    <footer><a href="https://github.com/Jonamarti/humanoid-physics-lab" rel="noopener noreferrer">Código en GitHub</a> · <a href="https://github.com/Jonamarti/humanoid-physics-lab/issues/1" rel="noopener noreferrer">Hoja de ruta</a></footer>
  </main>
`;

function required<T extends Element>(selector: string): T {
  const element = document.querySelector<T>(selector);
  if (!element) throw new Error("Falta el control " + selector);
  return element;
}

const forceInput = required<HTMLInputElement>("#force");
const durationInput = required<HTMLInputElement>("#duration");
const impulseOutput = required<HTMLElement>("#impulse");
const obstacleInput = required<HTMLSelectElement>("#obstacle-height");
const upwardInput = required<HTMLSelectElement>("#upward-ratio");
const cylinderControls = required<HTMLElement>("#cylinder-controls");
const predictionOutput = required<HTMLElement>("#pivot-prediction");
const forceDescription = required<HTMLElement>("#force-description");
const loadButton = required<HTMLButtonElement>("#physics-load");
const toggleButton = required<HTMLButtonElement>("#physics-toggle");
const resetButton = required<HTMLButtonElement>("#physics-reset");
const physicsStatus = required<HTMLElement>("#physics-status");
const physicsReadout = required<HTMLElement>("#physics-readout");
const balanceReadout = required<HTMLElement>("#balance-readout");
const groundProjectionReadout = required<HTMLElement>("#ground-projection-readout");
const physicsModel = required<HTMLSelectElement>("#physics-model");
const pushPanel = required<HTMLElement>("#standing-push-controls");
const rigidPanel = required<HTMLElement>("#rigid-force-controls");
const pushDirection = required<HTMLSelectElement>("#push-direction");
const pushPeak = required<HTMLInputElement>("#push-peak");
const pushDuration = required<HTMLInputElement>("#push-duration");
const pushPhase = required<HTMLElement>("#push-phase");
const forceMeterFill = required<HTMLElement>("#force-meter-fill");
const pushVectorReadout = required<HTMLElement>("#push-vector-readout");
const pushRun = required<HTMLButtonElement>("#push-run");
const pushReadout = required<HTMLElement>("#push-readout");
const stepFoot = required<HTMLSelectElement>("#step-foot");
const stepDirection = required<HTMLSelectElement>("#step-direction");
const stepCartesian = required<HTMLSelectElement>("#step-cartesian");
const stepProfile = required<HTMLSelectElement>("#step-profile");
const stepRun = required<HTMLButtonElement>("#step-run");
const stepReadout = required<HTMLElement>("#step-readout");
const stepGatesReadout = required<HTMLElement>("#step-gates-readout");

type Scenario = "ball" | "humanoid" | "standing" | "cylinder";
const selectedScenario = (): Scenario => {
  if (physicsModel.value === "humanoid") return "humanoid";
  if (physicsModel.value === "standing") return "standing";
  if (physicsModel.value === "cylinder") return "cylinder";
  return "ball";
};

function selectedTrial(): CylinderTrial {
  const trial = {
    obstacleHeightM: Number(obstacleInput.value),
    horizontalForceN: forceInput.valueAsNumber,
    upwardRatio: Number(upwardInput.value),
    durationS: durationInput.valueAsNumber / 1000,
  };
  validateCylinderTrial(trial);
  return trial;
}

/** These values become fixed in activePush when the user launches a trial.
 * Editing a field never alters a pulse already being integrated. */
function selectedPushTrial(): PushTrial {
  const trial: PushTrial = {
    direction: pushDirection.value as PushDirection,
    peakForceN: pushPeak.valueAsNumber,
    startS: 1,
    durationS: pushDuration.valueAsNumber / 1000,
  };
  if (!pushPeak.validity.valid || !pushDuration.validity.valid) {
    throw new RangeError("La magnitud o la duración del empujón están fuera de rango");
  }
  validatePushTrial(trial);
  return trial;
}

function validPushInputs(): boolean {
  try {
    selectedPushTrial();
    return true;
  } catch {
    return false;
  }
}

function updateCalculation(): void {
  const cylinderSelected = selectedScenario() === "cylinder";
  const standingSelected = selectedScenario() === "standing";
  pushPanel.hidden = !standingSelected;
  rigidPanel.hidden = standingSelected;
  pushRun.disabled = !!forceEditor?.enabled || !simulation || mode !== "standing" || !validPushInputs();
  stepRun.disabled = !simulation || mode !== "standing" || !!forceEditor?.enabled;
  cylinderControls.hidden = !cylinderSelected;
  forceDescription.textContent = cylinderSelected
    ? "Fuerza horizontal aplicada a 1,40 m de altura desde t = 0,30 s. Cargar simulación activa el ensayo; cambiar parámetros requiere recargarlo."
    : "La calculadora solo muestra unidades; no aplica fuerzas a la esfera ni al humanoide.";
  try {
    const trial = selectedTrial();
    const impulse = impulseFromConstantForce(trial.horizontalForceN, trial.durationS);
    impulseOutput.textContent = "Impulso horizontal: " + impulse.toFixed(1) + " N·s";
    if (cylinderSelected) {
      const prediction = idealPivotPrediction(trial);
      predictionOutput.textContent = "Predicción de pivote ideal: " +
        (prediction.tips ? "vuelca" : "no vuelca") +
        " · energía inicial " + prediction.initialKineticEnergyJ.toFixed(0) +
        " J · barrera " + prediction.potentialBarrierJ.toFixed(0) +
        " J (no equivale al resultado MuJoCo).";
    }
  } catch (error) {
    impulseOutput.textContent = "Parámetros inválidos: comprueba los límites del ensayo.";
    predictionOutput.textContent = error instanceof Error ? error.message : String(error);
  }
}

let viewer: BallViewer | undefined;
const viewport = document.querySelector<HTMLElement>("#physics-viewport");
if (viewport) {
  try {
    viewer = new BallViewer(viewport);
  } catch (error) {
    physicsStatus.textContent = "WebGL no disponible: " +
      (error instanceof Error ? error.message : String(error)) + ". Las métricas físicas siguen disponibles.";
  }
}

let simulation: MujocoSimulation | undefined;
let loadedStandingModel: "nominal" | "gait" = "nominal";
let mode: Scenario = "ball";
let activeTrial: CylinderTrial | undefined;
let activePush: PushTrial | undefined;
let activeStepTrial = false;
let activeStepFoot: "left" | "right" = "left";
let initialPushCom: readonly [number, number] | undefined;
let initialPushFeet: Map<number, readonly number[]> | undefined;
let minimumPushPelvisM = Infinity;
let maximumPushTiltDeg = 0;
let settledAfterPush = true;
let playing = false;
let lastFrame: number | undefined;
let clock = new FixedClock(0.002, 50);
let maxSampledForwardTiltDeg = 0;
let horizontalImpulseAppliedNs = 0;
let frameHandle: number | undefined;

function present(state: PhysicsState, forceAppliedThisFrameN: readonly [number,number,number] = [0,0,0]): void {
  const z = state.qpos[2];
  balanceReadout.hidden = mode !== "humanoid" && mode !== "standing";
  groundProjectionReadout.hidden = balanceReadout.hidden;
  if (mode === "cylinder") {
    maxSampledForwardTiltDeg = Math.max(maxSampledForwardTiltDeg, cylinderForwardTiltDeg(state.qpos));
    physicsReadout.textContent = "t = " + state.time.toFixed(3) +
      " s · CM z = " + (z?.toFixed(3) ?? "?") +
      " m · inclinación máxima muestreada = " + maxSampledForwardTiltDeg.toFixed(1) +
      "° · impulso aplicado = " + horizontalImpulseAppliedNs.toFixed(1) + " N·s";
    if (simulation) viewer?.setArticulatedPoses(simulation.geometryPoses());
    viewer?.clearForceArrow();
    return;
  }
  physicsReadout.textContent = "t = " + state.time.toFixed(3) +
    " s · " + ((mode === "humanoid" || mode === "standing") ? "pelvis" : "altura") +
    " " + (z?.toFixed(3) ?? "?") + " m";
  if (mode === "humanoid" || mode === "standing") {
    if (simulation) {
      viewer?.setArticulatedPoses(simulation.geometryPoses());
      const b = simulation.humanoidBalanceObservation();
      const ground = simulation.humanoidGroundSupportObservation();
      const projection = projectBalanceToGround(b.comWorldM, ground.loadedSupportPolygonXY);
      viewer?.setBalanceProjection(b.comWorldM, ground.loadedSupportPolygonXY);
      const label = projection.status === "inside" ? "DENTRO" :
        projection.status === "outside" ? "FUERA" : "SIN APOYO VERIFICADO";
      const friction = (n: number | null): string =>
        n === null ? "sin datos" : (100*n).toFixed(0)+" %";
      groundProjectionReadout.textContent =
        "Proyección del centro de masas en el suelo: " + label +
        " · carga pie izquierdo: " + ground.left.verticalLoadN.toFixed(0) +
        " N · pie derecho: " + ground.right.verticalLoadN.toFixed(0) +
        " N · utilización estimada del rozamiento (I/D): " +
        friction(ground.left.frictionUtilizationMean) + " / " +
        friction(ground.right.frictionUtilizationMean) +
        ". La proyección no garantiza equilibrio dinámico; la fricción es una medida del solver, no prueba por sí sola deslizamiento.";
      let previewTrial: PushTrial | undefined;
      if (mode === "standing" && !activeStepTrial) {
        try { previewTrial = activePush ?? selectedPushTrial(); }
        catch { previewTrial = undefined; }
      }
      if (forceEditor?.enabled) {
        viewer?.clearForceArrow();
        forceEditor.present(simulation, viewer, state.time);
      } else if (previewTrial) {
        const display = forceDisplayForFrame(state.time, previewTrial,
          activePush ? forceAppliedThisFrameN : [0,0,0]);
        const isActive = !!activePush && display.phase === "active";
        viewer?.setForceArrow(simulation.siteWorldPosition("sternum_impact"),
          display.displayForceN, isActive);
        const phase = !activePush ? "PREPARADO" :
          display.phase === "active" ? "APLICANDO FUERZA" :
          display.phase === "applied" ? "FUERZA APLICADA" : "PENDIENTE";
        pushPhase.textContent = phase + " · " + display.activeMagnitudeN.toFixed(1) +
          " N ahora · " + (previewTrial.durationS * 1000).toFixed(0) + " ms de duración";
        pushPhase.classList.toggle("is-active", isActive);
        forceMeterFill.style.width = (isActive ? display.activeFraction * 100 : 0).toFixed(1) + "%";
        pushVectorReadout.textContent =
          "Dirección configurada (X, Y, Z): " +
          display.referenceForceN.map(v => v.toFixed(1)).join(", ") + " N" +
          (isActive ? " · fuerza muestreada: " +
          display.displayForceN.map(v => v.toFixed(1)).join(", ") + " N" : "");
      } else {
        viewer?.clearForceArrow();
        forceMeterFill.style.width = "0%";
        pushPhase.classList.remove("is-active");
        pushPhase.textContent = activeStepTrial ? "ENSAYO DE PASO (sin empujón)" : "Fuerza o duración inválida";
      }
      if (mode === "standing" && activeStepTrial) {
        const snapshot = simulation.experimentalStepStatus();
        if (snapshot) {
          stepReadout.textContent = "Fase: " + snapshot.phase +
            " · elevación máxima del pie: " + (snapshot.maxSoleRiseM*100).toFixed(1) +
            " cm · desplazamiento hacia objetivo: " +
            (snapshot.currentTargetProgressM*100).toFixed(1) +
            " cm · aterrizaje verificado: " +
            (snapshot.stepContactConfirmed ? "sí" : "no") +
            " · deriva máxima del pie de apoyo: " +
            (snapshot.maxSupportFootSlipM * 100).toFixed(1) + " cm" +
            " · deslizamiento tras aterrizar: " +
            (snapshot.maxLandingSlipM * 100).toFixed(1) + " cm" +
            " · error hasta destino: " +
            (Number.isFinite(snapshot.targetErrorXYM) ?
              (snapshot.targetErrorXYM * 100).toFixed(1) : "—") +
            " cm · equilibrio continuo: " +
            snapshot.settledDurationS.toFixed(2) + " / 1,00 s" +
            " · velocidad horizontal precontacto (aprox.): " +
            (snapshot.preLandingTangentialSpeedMps === null ? "sin medir" :
              snapshot.preLandingTangentialSpeedMps.toFixed(2) + " m/s") +
            " · velocidad de bajada precontacto (aprox.): " +
            (snapshot.preLandingDownwardSpeedMps === null ? "sin medir" :
              snapshot.preLandingDownwardSpeedMps.toFixed(2) + " m/s") +
            " · fricción del contacto inicial: " +
            (snapshot.frictionUtilizationAtLanding === null ? "sin medir" :
              (snapshot.frictionUtilizationAtLanding*100).toFixed(0) + " %") +
            " · flexión de rodilla REAL: " +
            (-simulation.jointAngleRad(activeStepFoot+"_knee")*180/Math.PI).toFixed(1) + "°" +
            " · inclinación articular del tronco REAL: " +
            ((simulation.jointAngleRad("lumbar_pitch") +
              simulation.jointAngleRad("thorax_pitch"))*180/Math.PI).toFixed(1) + "°" +
            " · " + snapshot.reason;
          const gate=snapshot.stability;
          stepGatesReadout.textContent = gate
            ? (gate.failures.length===0
              ? "Condiciones instantáneas cumplidas; falta sostenerlas 1 segundo."
              : "Condiciones que impiden completar: " +
                gate.failures.map(k=>STEP_STABILITY_LABELS[k]).join("; ") +
                " · velocidad COM: "+gate.comSpeedXYMps.toFixed(3)+" m/s")
            : "Condiciones pendientes: primero debe despegar y apoyar físicamente.";
        }
      }
      if (mode === "standing" && activePush && initialPushCom) {
        minimumPushPelvisM = Math.min(minimumPushPelvisM, state.qpos[2]!);
        maximumPushTiltDeg = Math.max(maximumPushTiltDeg, rootTiltDegrees(state.qpos));
        const delta = Math.hypot(b.comWorldM[0] - initialPushCom[0], b.comWorldM[1] - initialPushCom[1]);
        const speed = Math.hypot(b.comVelocityWorldMps[0], b.comVelocityWorldMps[1]);
        if (state.time > 5 && (delta > 0.03 || speed > 0.08 || rootTiltDegrees(state.qpos) > 8)) {
          settledAfterPush = false;
        }
        pushReadout.textContent = "Empujón máximo " + activePush.peakForceN + " N · impulso teórico " +
          expectedPushImpulseNs(activePush).toFixed(2) + " N·s · inclinación máxima muestreada " +
          maximumPushTiltDeg.toFixed(1) + "° · pelvis mínima " + minimumPushPelvisM.toFixed(3) +
          " m · desviación de COM actual " + (delta * 100).toFixed(1) + " cm.";
      }
      const fmt = (n: number) => n.toFixed(3);
      balanceReadout.textContent =
        "Centro de masas (x,y,z): " + b.comWorldM.map(fmt).join(", ") + " m" +
        " · velocidad horizontal: " + fmt(b.comVelocityWorldMps[0]) + ", " +
        fmt(b.comVelocityWorldMps[1]) + " m/s" +
        " · punto de captura estimado: " + b.capturePointXY.map(fmt).join(", ") + " m" +
        " · proyección del CM sobre huellas cercanas al suelo: " +
        (b.comProjectionInsideCandidateSupport ? "dentro" : "fuera") +
        " (no mide presión real ni demuestra estabilidad).";
    }
  } else if (state.qpos.length >= 3) {
    viewer?.setSpherePosition([state.qpos[0]!, state.qpos[1]!, state.qpos[2]!]);
  }
}

function animate(timestampMs: number): void {
  frameHandle = window.requestAnimationFrame(animate);
  if (!playing || !simulation) {
    lastFrame = timestampMs;
    return;
  }
  try {
    const elapsed = lastFrame === undefined ? 0 : Math.min((timestampMs - lastFrame) / 1000, 0.1);
    lastFrame = timestampMs;
    const steps = clock.consume(elapsed);
    if (!steps) return;
    const activeSimulation = simulation;
    let state: PhysicsState;
    if (mode === "cylinder" && activeTrial) {
      const trial = activeTrial;
      state = activeSimulation.step(steps, (timeS) => {
        const force = cylinderForceAt(timeS, trial);
        if (force[0] !== 0 || force[2] !== 0) {
          // Generated MJCF contains world body (0) and cylinder body (1).
          activeSimulation.applyForceAtBodyLocalPoint(1, localGripPoint(), force);
          horizontalImpulseAppliedNs += force[0] * CYLINDER.timestepS;
        }
      });
    } else if (mode === "standing" && forceEditor?.running) {
      const editor = forceEditor;
      editor.beginFrame();
      state = activeSimulation.step(steps, timeS => editor.applyStep(activeSimulation, timeS));
      present(state);
    } else if (mode === "standing" && activePush) {
      const trial = activePush;
      let peakApplied: readonly [number, number, number] = [0,0,0];
      state = activeSimulation.step(steps, timeS => {
        const force = pushForceAtTime(timeS, trial);
        if (force.some(v => v !== 0)) {
          activeSimulation.applyWorldForceAtSite("sternum_impact", force);
          if (Math.hypot(...force) > Math.hypot(...peakApplied)) peakApplied = force;
        }
      });
      present(state, peakApplied);
    } else {
      state = activeSimulation.step(steps);
    }
    if (!(mode === "standing" && (activePush || forceEditor?.running))) present(state);
    if (mode === "standing" && forceEditor?.running && state.time >= forceEditor.horizonS) {
      playing = false;
      forceEditor.finish();
      toggleButton.textContent = "Iniciar";
      physicsStatus.textContent = "Secuencia múltiple completada; resultados físicos, no validación biomecánica.";
    }
    if (mode === "standing" && activePush && state.time >= 6) {
      playing = false;
      toggleButton.textContent = "Iniciar";
      const b = activeSimulation.humanoidBalanceObservation();
      const footDrift = initialPushFeet
        ? Math.max(...Array.from(initialPushFeet, ([id, before]) => {
            const now = activeSimulation.geometryPoses().find(p => p.id === id)?.position;
            return now ? Math.hypot(now[0] - before[0]!, now[1] - before[1]!) : Infinity;
          }))
        : Infinity;
      const recovered = minimumPushPelvisM > 0.86 && maximumPushTiltDeg < 25 &&
        footDrift < 0.02 && settledAfterPush;
      const offset = initialPushCom
        ? Math.hypot(b.comWorldM[0] - initialPushCom[0], b.comWorldM[1] - initialPushCom[1])
        : Infinity;
      physicsStatus.textContent = recovered
        ? "Prueba con empujón: recuperó la postura sin un paso, según métricas muestreadas."
        : "Prueba con empujón: NO recuperó la postura con los límites de este ensayo.";
      pushReadout.textContent += " Deriva máxima final de pies: " + (footDrift * 100).toFixed(1) +
        " cm · error final de COM " + (offset * 100).toFixed(1) + " cm.";
    }
    if (mode === "standing" && activeStepTrial && state.time >= 5.2) {
      playing = false;
      toggleButton.textContent = "Iniciar";
      const status = activeSimulation.experimentalStepStatus();
      physicsStatus.textContent = status?.phase === "COMPLETE"
        ? "Paso físico comprobado: despegue, avance hacia objetivo y aterrizaje cargado."
        : status?.phase === "FALL"
        ? "El intento de paso terminó en una caída física."
        : "Paso experimental no completado: " + (status?.reason ?? "sin diagnóstico");
    }
    if (mode === "standing" && !activePush && !activeStepTrial && !forceEditor?.running && state.time >= 10) {
      playing = false;
      toggleButton.textContent = "Iniciar";
      physicsStatus.textContent = state.qpos[2]! >= 0.79
        ? "Diez segundos simulados: pelvis alta y controlador activo. Ensayo nominal sin empujones; la recuperación aún no está validada."
        : "Diez segundos simulados, pero el humanoide perdió altura: el escenario no superó la evaluación.";
    }
    if (mode === "cylinder" && state.time >= 3) {
      playing = false;
      toggleButton.textContent = "Iniciar";
      physicsStatus.textContent = maxSampledForwardTiltDeg > 90
        ? "Ensayo completado: el cilindro ha volcado hacia delante (>90° muestreados)."
        : "Ensayo completado: no se observó vuelco hacia delante (>90°).";
    }
  } catch (error) {
    playing = false;
    toggleButton.textContent = "Iniciar";
    physicsStatus.textContent = "Error físico: " +
      (error instanceof Error ? error.message : String(error));
  }
}

function clearLoadedScenario(): void {
  playing = false;
  simulation?.dispose();
  simulation = undefined;
  loadedStandingModel = "nominal";
  activeTrial = undefined;
  activePush = undefined;
  activeStepTrial = false;
  stepRun.disabled = true;
  stepReadout.textContent = "El paso todavía no se ha ensayado.";
  stepGatesReadout.textContent = "Condiciones pendientes: primero debe despegar y apoyar físicamente.";
  initialPushCom = undefined;
  initialPushFeet = undefined;
  minimumPushPelvisM = Infinity;
  maximumPushTiltDeg = 0;
  settledAfterPush = true;
  pushRun.disabled = true;
  pushReadout.textContent = "El empujón comienza en t=1,00 s y dura 0,12 s.";
  pushVectorReadout.textContent = "Vector del empujón: selecciona una dirección y ejecuta el ensayo.";
  pushPhase.textContent = "LISTO · 0 N aplicados";
  pushPhase.classList.remove("is-active");
  forceMeterFill.style.width = "0%";
  viewer?.clearForceArrow();
  viewer?.clearMultiForces();
  forceEditor?.reset();
  viewer?.clearBalanceProjection();
  viewer?.showSphere();
  clock.reset();
  maxSampledForwardTiltDeg = 0;
  horizontalImpulseAppliedNs = 0;
  toggleButton.disabled = true;
  resetButton.disabled = true;
  toggleButton.textContent = "Iniciar";
}

physicsModel.addEventListener("change", () => {
  clearLoadedScenario();
  physicsStatus.textContent = "Escenario seleccionado. Pulsa Cargar simulación.";
  updateCalculation();
});

for (const control of [forceInput, durationInput, obstacleInput, upwardInput]) {
  control.addEventListener("input", () => {
    updateCalculation();
    if (simulation && mode === "cylinder") {
      physicsStatus.textContent = "Parámetros editados; pulsa Cargar simulación para aplicarlos al cilindro.";
    }
  });
}

loadButton.addEventListener("click", async () => {
  loadButton.disabled = true;
  physicsModel.disabled = true;
  physicsStatus.textContent = "Descargando e iniciando MuJoCo WASM...";
  try {
    clearLoadedScenario();
    const nextMode = selectedScenario();
    let xml = GRAVITY_DEMO_MJCF;
    let trial: CylinderTrial | undefined;
    if (nextMode === "humanoid" || nextMode === "standing") {
      const response = await fetch(import.meta.env.BASE_URL + "models/humanoid_v1.xml");
      if (!response.ok) throw new Error("No se puede cargar humanoid_v1.xml: HTTP " + response.status);
      xml = await response.text();
    } else if (nextMode === "cylinder") {
      trial = selectedTrial();
      xml = cylinderMJCF(trial.obstacleHeightM);
    }
    simulation = await MujocoSimulation.create(xml);
    mode = nextMode;
    activeTrial = trial;
    if (mode === "standing") simulation.enableExperimentalStanding();
    clock = new FixedClock(mode === "cylinder" ? CYLINDER.timestepS : 0.002, 60);
    if (mode === "humanoid" || mode === "standing" || mode === "cylinder") {
      viewer?.showArticulated(simulation.geometrySpecs());
    } else {
      viewer?.showSphere();
    }
    present(simulation.state());
    physicsStatus.textContent = mode === "humanoid"
      ? "Humanoide pasivo sin control de equilibrio: cae bajo gravedad."
      : mode === "standing"
        ? "Equilibrio activo experimental: tobillos y articulaciones aplican pares limitados para mantener la postura durante 10 s, sin empujones."
        : mode === "cylinder"
        ? "Cilindro listo: el tirón comienza a t = 0,30 s, desde una altura de 1,40 m."
        : "Esfera libre: gravedad y contacto real con el suelo.";
    toggleButton.disabled = false;
    resetButton.disabled = false;
    pushRun.disabled = !!forceEditor?.enabled || mode !== "standing" || !validPushInputs();
    forceEditor?.refresh();
    stepRun.disabled = mode !== "standing" || !!forceEditor?.enabled;
    toggleButton.textContent = "Iniciar";
  } catch (error) {
    clearLoadedScenario();
    physicsStatus.textContent = "No se pudo cargar MuJoCo: " +
      (error instanceof Error ? error.message : String(error));
  } finally {
    loadButton.disabled = false;
    physicsModel.disabled = false;
  }
});

for (const control of [pushPeak, pushDuration, pushDirection]) {
  control.addEventListener("input", () => {
    pushRun.disabled = !!forceEditor?.enabled || !simulation || mode !== "standing" || !validPushInputs();
    if (simulation && mode === "standing" && !activePush) present(simulation.state());
  });
}

/** Keep the published balance/push humanoid unchanged. The corrected
 * knee-axis model is opt-in ONLY for the experimental gait test.
 * Recreate the MuJoCo model, rather than patching its compiled joint axes
 * in-place (which could invalidate precomputed contact structures).
 */
async function ensureStandingModel(requested: "nominal" | "gait"): Promise<void> {
  if (!simulation || mode !== "standing" || loadedStandingModel===requested) return;
  const previous=simulation;
  const file=requested==="gait" ? "models/humanoid_step_v2.xml" : "models/humanoid_v1.xml";
  physicsStatus.textContent = "Preparando modelo " +
    (requested==="gait" ? "de marcha con flexión anatómica" : "de equilibrio original") + "...";
  const response=await fetch(import.meta.env.BASE_URL + file);
  if (!response.ok) throw new Error("No se pudo cargar "+file+": HTTP "+response.status);
  const next=await MujocoSimulation.create(await response.text());
  if (simulation!==previous || mode!=="standing") {
    next.dispose();
    throw new Error("El escenario cambió durante la carga del modelo");
  }
  try {
    next.enableExperimentalStanding();
    viewer?.showArticulated(next.geometrySpecs());
  } catch(error) {
    next.dispose();
    throw error;
  }
  playing=false;
  simulation=next;
  previous.dispose();
  loadedStandingModel=requested;
  activePush=undefined;
  activeStepTrial=false;
  clock.reset();
  lastFrame=undefined;
  toggleButton.textContent="Iniciar";
  forceEditor?.reset();
}

pushRun.addEventListener("click", async () => {
  if (!simulation || mode !== "standing" || forceEditor?.enabled) return;
  try {
    const trial = selectedPushTrial();
    playing = false;
    // User may have just inspected a knee-corrected gait trial. Always
    // restore the unchanged standard model for push comparisons.
    await ensureStandingModel("nominal");
    if (!simulation || mode !== "standing") return;
    activeStepTrial = false;
    stepReadout.textContent = "El paso todavía no se ha ensayado.";
    simulation.reset();
    clock.reset();
    lastFrame = undefined;
    const b = simulation.humanoidBalanceObservation();
    initialPushCom = [b.comWorldM[0], b.comWorldM[1]];
    const feetIds = simulation.geometrySpecs().filter(spec =>
      spec.shape === "box" && Math.abs(spec.size[2] - 0.035) < 1e-6).map(spec => spec.id);
    initialPushFeet = new Map(simulation.geometryPoses().filter(p => feetIds.includes(p.id)).map(p => [p.id, p.position]));
    activePush = trial;
    minimumPushPelvisM = simulation.state().qpos[2]!;
    maximumPushTiltDeg = 0;
    settledAfterPush = true;
    playing = true;
    toggleButton.textContent = "Pausar";
    physicsStatus.textContent = "Empujón físico programado desde t=1,00 s; simulación hasta t=6,00 s.";
    present(simulation.state());
  } catch (error) {
    playing = false;
    physicsStatus.textContent = "No se pudo iniciar el ensayo: " +
      (error instanceof Error ? error.message : String(error));
  }
});

stepRun.addEventListener("click", async () => {
  if (!simulation || mode !== "standing" || forceEditor?.enabled) return;
  try {
    const side = stepFoot.value === "right" ? "right" : "left";
    const sign = side === "left" ? 1 : -1;
    const direction = stepDirection.value;
    const delta: readonly [number,number] = direction === "outward"
      ? [0, 0.12*sign] : direction === "forward" ? [0.12,0] :
        direction === "backward" ? [-0.10,0] : [0.12,0.08*sign];
    playing = false;
    await ensureStandingModel(Number(stepProfile.value)>0 ? "gait" : "nominal");
    if (!simulation || mode !== "standing") return;
    simulation.reset();
    simulation.enableExperimentalStepTrial(side, delta, 0.8, Number(stepCartesian.value), Number(stepProfile.value));
    activeStepFoot = side;
    clock.reset();
    lastFrame = undefined;
    activePush = undefined;
    initialPushCom = undefined;
    initialPushFeet = undefined;
    activeStepTrial = true;
    playing = true;
    toggleButton.textContent = "Pausar";
    physicsStatus.textContent = Number(stepProfile.value) > 0 ?
      "Ensayo de marcha coordinada: transferencia del COM, inclinación de tronco, elevación del muslo y flexión de rodilla. Resultados físicos sin garantías de estabilidad." :
      "Ensayo anterior de paso físico; los fallos se mostrarán como fallos.";
    present(simulation.state());
  } catch (error) {
    playing = false;
    activeStepTrial = false;
    physicsStatus.textContent = "No se pudo iniciar el ensayo de paso: " +
      (error instanceof Error ? error.message : String(error));
  }
});

toggleButton.addEventListener("click", () => {
  if (!simulation) return;
  playing = !playing;
  lastFrame = undefined;
  toggleButton.textContent = playing ? "Pausar" : "Iniciar";
  if (!playing) present(simulation.state());
  forceEditor?.refresh();
});

resetButton.addEventListener("click", () => {
  if (!simulation) return;
  playing = false;
  clock.reset();
  lastFrame = undefined;
  maxSampledForwardTiltDeg = 0;
  horizontalImpulseAppliedNs = 0;
  activePush = undefined;
  activeStepTrial = false;
  stepReadout.textContent = "El paso todavía no se ha ensayado.";
  initialPushCom = undefined;
  initialPushFeet = undefined;
  settledAfterPush = true;
  maximumPushTiltDeg = 0;
  minimumPushPelvisM = Infinity;
  simulation.reset();
  pushReadout.textContent = "El empujón comienza en t=1,00 s y dura 0,12 s.";
  pushVectorReadout.textContent = "Vector del empujón: selecciona una dirección y ejecuta el ensayo.";
  viewer?.clearForceArrow();
  viewer?.clearMultiForces();
  forceEditor?.reset();
  present(simulation.state());
  toggleButton.textContent = "Iniciar";
  physicsStatus.textContent = "Estado reiniciado. Los parámetros cargados siguen siendo los mismos.";
});

const forceEditor = installForceEditor(required<HTMLElement>("#multi-force-editor"), {
  getSimulation: () => mode === "standing" ? simulation : undefined,
  getViewer: () => viewer,
  canEdit: () => !playing,
  onToggle: () => {
    if (!simulation || mode !== "standing" || playing) return;
    if (forceEditor?.enabled) {
      activePush = undefined;
      viewer?.clearForceArrow();
    } else viewer?.clearMultiForces();
    updateCalculation();
    present(simulation.state());
  },
  onRun: () => {
    if (!simulation || mode !== "standing") return;
    playing = false;
    simulation.reset();
    clock.reset(); lastFrame = undefined;
    activePush = undefined;
    activeStepTrial = false;
    stepReadout.textContent = "El paso todavía no se ha ensayado.";
    playing = true;
    toggleButton.textContent = "Pausar";
    physicsStatus.textContent = "Secuencia de fuerzas externas 3D activa.";
    present(simulation.state());
    forceEditor?.refresh();
  },
});
updateCalculation();
frameHandle = window.requestAnimationFrame(animate);
window.addEventListener("pagehide", () => {
  playing = false;
  if (frameHandle !== undefined) window.cancelAnimationFrame(frameHandle);
  simulation?.dispose();
  simulation = undefined;
  viewer?.dispose();
}, { once: true });
