import "./styles.css";
import { impulseFromConstantForce } from "./physics/impulse";
import { GRAVITY_DEMO_MJCF } from "./physics/demoModel";
import { FixedClock } from "./physics/fixedClock";
import { MujocoSimulation, type PhysicsState } from "./physics/mujoco";
import { BallViewer } from "./renderer/ballViewer";
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
          <p class="muted">El impacto se aplica en el esternón del humanoide, mediante una fuerza externa en MuJoCo. No se fija la pelvis ni se mueve ningún pie artificialmente.</p>
          <label for="push-direction">Dirección</label>
          <select id="push-direction">
            <option value="forward">Hacia delante (+X)</option>
            <option value="backward" selected>Hacia atrás (-X)</option>
            <option value="left">Hacia la izquierda (+Y)</option>
            <option value="right">Hacia la derecha (-Y)</option>
          </select>
          <label for="push-peak">Fuerza máxima del empujón</label>
          <select id="push-peak">
            <option value="20">20 N — muy suave</option>
            <option value="60">60 N — suave</option>
            <option value="120">120 N — moderado</option>
            <option value="240" selected>240 N — puede derribar hacia atrás</option>
          </select>
          <button id="push-run" type="button" disabled>Reiniciar y probar empujón (6 s)</button>
          <p class="muted" id="push-readout">La fuerza se aplica entre t=1,00 y 1,12 s. La recuperación se observa durante el resto de la simulación.</p>
          <p class="muted">El diagnóstico visual es aproximado. Las pruebas automatizadas registran resultados cada 0,04 s.</p>
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
const physicsModel = required<HTMLSelectElement>("#physics-model");
const pushPanel = required<HTMLElement>("#standing-push-controls");
const rigidPanel = required<HTMLElement>("#rigid-force-controls");
const pushDirection = required<HTMLSelectElement>("#push-direction");
const pushPeak = required<HTMLSelectElement>("#push-peak");
const pushRun = required<HTMLButtonElement>("#push-run");
const pushReadout = required<HTMLElement>("#push-readout");

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

function updateCalculation(): void {
  const cylinderSelected = selectedScenario() === "cylinder";
  const standingSelected = selectedScenario() === "standing";
  pushPanel.hidden = !standingSelected;
  rigidPanel.hidden = standingSelected;
  pushRun.disabled = !simulation || mode !== "standing";
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
let mode: Scenario = "ball";
let activeTrial: CylinderTrial | undefined;
let activePush: PushTrial | undefined;
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

function present(state: PhysicsState): void {
  const z = state.qpos[2];
  balanceReadout.hidden = mode !== "humanoid" && mode !== "standing";
  if (mode === "cylinder") {
    maxSampledForwardTiltDeg = Math.max(maxSampledForwardTiltDeg, cylinderForwardTiltDeg(state.qpos));
    physicsReadout.textContent = "t = " + state.time.toFixed(3) +
      " s · CM z = " + (z?.toFixed(3) ?? "?") +
      " m · inclinación máxima muestreada = " + maxSampledForwardTiltDeg.toFixed(1) +
      "° · impulso aplicado = " + horizontalImpulseAppliedNs.toFixed(1) + " N·s";
    if (simulation) viewer?.setArticulatedPoses(simulation.geometryPoses());
    return;
  }
  physicsReadout.textContent = "t = " + state.time.toFixed(3) +
    " s · " + ((mode === "humanoid" || mode === "standing") ? "pelvis" : "altura") +
    " " + (z?.toFixed(3) ?? "?") + " m";
  if (mode === "humanoid" || mode === "standing") {
    if (simulation) {
      viewer?.setArticulatedPoses(simulation.geometryPoses());
      const b = simulation.humanoidBalanceObservation();
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
    } else if (mode === "standing" && activePush) {
      const trial = activePush;
      state = activeSimulation.step(steps, timeS => {
        const force = pushForceAtTime(timeS, trial);
        if (force.some(v => v !== 0)) activeSimulation.applyWorldForceAtSite("sternum_impact", force);
      });
    } else {
      state = activeSimulation.step(steps);
    }
    present(state);
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
    if (mode === "standing" && !activePush && state.time >= 10) {
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
  activeTrial = undefined;
  activePush = undefined;
  initialPushCom = undefined;
  initialPushFeet = undefined;
  minimumPushPelvisM = Infinity;
  maximumPushTiltDeg = 0;
  settledAfterPush = true;
  pushRun.disabled = true;
  pushReadout.textContent = "El empujón comienza en t=1,00 s y dura 0,12 s.";
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
    pushRun.disabled = mode !== "standing";
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

pushRun.addEventListener("click", () => {
  if (!simulation || mode !== "standing") return;
  try {
    const trial: PushTrial = {
      direction: pushDirection.value as PushDirection,
      peakForceN: Number(pushPeak.value),
      startS: 1,
      durationS: 0.12,
    };
    validatePushTrial(trial);
    playing = false;
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

toggleButton.addEventListener("click", () => {
  if (!simulation) return;
  playing = !playing;
  lastFrame = undefined;
  toggleButton.textContent = playing ? "Pausar" : "Iniciar";
});

resetButton.addEventListener("click", () => {
  if (!simulation) return;
  playing = false;
  clock.reset();
  lastFrame = undefined;
  maxSampledForwardTiltDeg = 0;
  horizontalImpulseAppliedNs = 0;
  activePush = undefined;
  initialPushCom = undefined;
  initialPushFeet = undefined;
  settledAfterPush = true;
  maximumPushTiltDeg = 0;
  minimumPushPelvisM = Infinity;
  simulation.reset();
  pushReadout.textContent = "El empujón comienza en t=1,00 s y dura 0,12 s.";
  present(simulation.state());
  toggleButton.textContent = "Iniciar";
  physicsStatus.textContent = "Estado reiniciado. Los parámetros cargados siguen siendo los mismos.";
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
