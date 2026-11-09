import "./styles.css";
import { impulseFromConstantForce } from "./physics/impulse";

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
        <div class="viewport" role="img" aria-label="Escena 3D pendiente de integrar con MuJoCo y Three.js">
          <div class="grid-floor"></div>
          <div class="silhouette" aria-hidden="true">♟</div>
          <p>Visor 3D — previsto en la issue #5</p>
        </div>
        <div class="status"><span class="dot"></span> Base web operativa · Simulación no integrada</div>
      </article>
      <article class="panel controls">
        <h2>Ensayo de impulso</h2>
        <p class="muted">Esta calculadora verifica unidades; todavía <strong>no</strong> simula el movimiento ni predice caídas.</p>
        <label for="force">Fuerza constante (N)</label>
        <input id="force" type="number" value="300" min="0" max="100000" step="25" />
        <label for="duration">Duración (ms)</label>
        <input id="duration" type="number" value="100" min="0" max="5000" step="10" />
        <p class="result" id="impulse" role="status" aria-live="polite">Impulso: 30.0 N·s</p>
        <p class="muted">Próximos hitos: motor físico WASM (#4), modelo articulado (#7), control postural y prueba de 10 s (#8–#9).</p>
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
