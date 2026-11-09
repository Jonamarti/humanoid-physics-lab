import "./styles.css";
import { impulseFromConstantForce } from "./physics/impulse";
import { GRAVITY_DEMO_MJCF } from "./physics/demoModel";
import { FixedClock } from "./physics/fixedClock";
import { MujocoSimulation, type PhysicsState } from "./physics/mujoco";
import { BallViewer } from "./renderer/ballViewer";

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
        <div id="physics-viewport" class="viewport" role="group" aria-label="Visor 3D: giro y zoom táctil de esfera física MuJoCo"></div>
        <div class="status"><span class="dot"></span> MuJoCo WASM + Three.js — esfera física, no humanoide</div>
        <div class="controls" aria-label="Control de simulación">
          <p id="physics-status" role="status" aria-live="polite">Motor sin cargar.</p>
          <p id="physics-readout" class="result">t = 0.000 s · altura 1.500 m</p>
          <div class="button-row">
            <button id="physics-load" type="button">Cargar simulación</button>
            <button id="physics-toggle" type="button" disabled>Iniciar</button>
            <button id="physics-reset" type="button" disabled>Reiniciar</button>
          </div>
        </div>
      </article>
      <article class="panel controls">
        <h2>Ensayo de impulso</h2>
        <p class="muted">Esta calculadora verifica unidades; todavía <strong>no</strong> simula el movimiento ni predice caídas.</p>
        <label for="force">Fuerza constante (N)</label>
        <input id="force" type="number" value="300" min="0" max="100000" step="25" />
        <label for="duration">Duración (ms)</label>
        <input id="duration" type="number" value="100" min="0" max="5000" step="10" />
        <p class="result" id="impulse" role="status" aria-live="polite">Impulso: 30.0 N·s</p>
        <p class="muted">Próximos hitos: publicación GitHub Pages (#6), humanoide articulado (#7), equilibrio 10 segundos (#8–#9).</p>
      </article>
    </section>
    <footer><a href="https://github.com/Jonamarti/humanoid-physics-lab" rel="noopener noreferrer">Código en GitHub</a> · <a href="https://github.com/Jonamarti/humanoid-physics-lab/issues/1" rel="noopener noreferrer">Hoja de ruta</a></footer>
  </main>
`;

const forceInput = document.querySelector<HTMLInputElement>("#force");
const durationInput = document.querySelector<HTMLInputElement>("#duration");
const impulseOutput = document.querySelector<HTMLElement>("#impulse");
if (!forceInput || !durationInput || !impulseOutput) throw new Error("Faltan controles del ensayo");

const update = (): void => {
  const force = forceInput.valueAsNumber;
  const durationMs = durationInput.valueAsNumber;
  if (!Number.isFinite(force) || !Number.isFinite(durationMs) || force < 0 || durationMs < 0) {
    impulseOutput.textContent = "Introduce valores válidos no negativos.";
    return;
  }
  impulseOutput.textContent = `Impulso: ${impulseFromConstantForce(force, durationMs / 1000).toFixed(1)} N·s`;
};
forceInput.addEventListener("input", update);
durationInput.addEventListener("input", update);

const loadButton = document.querySelector<HTMLButtonElement>("#physics-load");
const toggleButton = document.querySelector<HTMLButtonElement>("#physics-toggle");
const resetButton = document.querySelector<HTMLButtonElement>("#physics-reset");
const physicsStatus = document.querySelector<HTMLElement>("#physics-status");
const physicsReadout = document.querySelector<HTMLElement>("#physics-readout");
if (!loadButton || !toggleButton || !resetButton || !physicsStatus || !physicsReadout) {
  throw new Error("Faltan controles de MuJoCo");
}

let viewer: BallViewer | undefined;
const viewport = document.querySelector<HTMLElement>("#physics-viewport");
if (viewport) {
  try {
    viewer = new BallViewer(viewport);
  } catch (error) {
    physicsStatus.textContent = `No se pudo abrir WebGL: ${error instanceof Error ? error.message : String(error)}. La telemetría de física sigue disponible.`;
  }
}

let simulation: MujocoSimulation | undefined;
let playing = false;
let lastFrame: number | undefined;
const clock = new FixedClock(0.002, 50);
let frameHandle: number | undefined;

function present(state: PhysicsState): void {
  if (!physicsReadout) return;
  const z = state.qpos[2];
  physicsReadout.textContent = `t = ${state.time.toFixed(3)} s · altura ${z?.toFixed(3) ?? "?"} m`;
  if (state.qpos.length >= 3) {
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
    if (steps) present(simulation.step(steps));
  } catch (error) {
    playing = false;
    if (toggleButton) toggleButton.textContent = "Iniciar";
    if (physicsStatus) physicsStatus.textContent =
      `Error físico: ${error instanceof Error ? error.message : String(error)}`;
  }
}

loadButton.addEventListener("click", async () => {
  loadButton.disabled = true;
  physicsStatus.textContent = "Descargando e iniciando MuJoCo WASM...";
  try {
    simulation?.dispose();
    simulation = await MujocoSimulation.create(GRAVITY_DEMO_MJCF);
    playing = false;
    clock.reset();
    present(simulation.state());
    physicsStatus.textContent = "Motor físico preparado: esfera libre, gravedad y contacto real con el suelo.";
    toggleButton.disabled = false;
    resetButton.disabled = false;
    toggleButton.textContent = "Iniciar";
  } catch (error) {
    physicsStatus.textContent = `No se pudo cargar MuJoCo: ${error instanceof Error ? error.message : String(error)}`;
    simulation = undefined;
  } finally {
    loadButton.disabled = false;
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
  simulation.reset();
  present(simulation.state());
  toggleButton.textContent = "Iniciar";
});
frameHandle = window.requestAnimationFrame(animate);
window.addEventListener("pagehide", () => {
  playing = false;
  if (frameHandle !== undefined) window.cancelAnimationFrame(frameHandle);
  simulation?.dispose();
  simulation = undefined;
  viewer?.dispose();
}, { once: true });
