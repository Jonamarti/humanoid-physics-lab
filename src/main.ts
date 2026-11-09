import "./styles.css";
import { impulseFromConstantForce } from "./physics/impulse";
import { GRAVITY_DEMO_MJCF } from "./physics/demoModel";
import { FixedClock } from "./physics/fixedClock";
import { MujocoSimulation, type PhysicsState } from "./physics/mujoco";
import { BallViewer } from "./renderer/ballViewer";
import { CYLINDER, cylinderMJCF, cylinderForceAt, cylinderForwardTiltDeg, idealPivotPrediction, localGripPoint, validateCylinderTrial, type CylinderTrial } from "./physics/cylinderBenchmark";

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
    } else {
      state = activeSimulation.step(steps);
    }
    present(state);
    if (mode === "standing" && state.time >= 10) {
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
  simulation.reset();
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
