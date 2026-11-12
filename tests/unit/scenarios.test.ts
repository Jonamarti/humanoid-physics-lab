import { readdirSync, readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { HAND_DEFAULTS } from "../../src/effectors/hand";
import { idealPivotPrediction } from "../../src/physics/cylinderBenchmark";
import { runScenario } from "../../src/scenario/run";
import type { Scenario } from "../../src/scenario/schema";
import { validateScenario } from "../../src/scenario/validate";

/**
 * Prueba dirigida por datos: recorre `scenarios/*.json`, valida, ejecuta y exige `summary.expect === "pass"`.
 * Añadir una prueba es añadir un JSON.
 *
 * Margen contra umbrales frágiles (issue #74, regla de la PR 4): un caso «fall» debe estar al menos a 1,3 veces la
 * fuerza de umbral y uno «standing» por debajo de 0,6 veces. Se comprueba sin conocer el umbral: el caso «fall» con
 * todas sus fuerzas divididas por 1,3 debe seguir cayendo, y el «standing» con sus fuerzas divididas por 0,6
 * debe seguir de pie. Los umbrales hallados están en `docs/CYLINDER_BENCHMARK.md`.
 */
const FALL_MARGIN = 1.3;
const STAND_MARGIN = 0.6;

const dir = new URL("../../scenarios/", import.meta.url);
const files = readdirSync(dir).filter((f) => f.endsWith(".json")).sort();

function load(file: string): Scenario {
  return validateScenario(JSON.parse(readFileSync(new URL(file, dir), "utf8")) as unknown, { fileName: file });
}

/** Copia con todas las fuerzas de los efectores multiplicadas por `factor`. */
function scaled(scenario: Scenario, factor: number): Scenario {
  const copy = structuredClone(scenario);
  for (const e of copy.effectors) {
    // Mano virtual: lo que se escala es su tope de fuerza.
    if (e.kind === "hand") e.max_force_n = (e.max_force_n ?? HAND_DEFAULTS.max_force_n) * factor;
    else e.force_n = [e.force_n[0] * factor, e.force_n[1] * factor, e.force_n[2] * factor];
  }
  return copy;
}

describe("escenarios de scenarios/*.json (MuJoCo WASM real)", () => {
  it("existen los cinco escenarios del cilindro", () => {
    expect(files).toEqual(expect.arrayContaining([
      "cylinder_no_obstacle.json",
      "cylinder_obstacle_above_com.json",
      "cylinder_obstacle_high.json",
      "cylinder_obstacle_low.json",
      "cylinder_three_phase.json",
    ]));
  });

  for (const file of files) {
    describe(file, () => {
      const scenario = load(file);

      it("cumple su expect", async () => {
        expect(scenario.expect, "todo escenario de scenarios/ debe declarar expect").toBeDefined();
        const rec = await runScenario(scenario, { engineVersion: "test" });
        const { outcome, direction_deg, max_tilt_deg, final_tilt_deg } = rec.summary;
        console.log(`${scenario.id}: ${outcome}, dirección ${direction_deg === null ? "-" : direction_deg.toFixed(1)}°, inclinación máx ${max_tilt_deg.toFixed(1)}°, final ${final_tilt_deg.toFixed(1)}°`);
        expect(rec.summary.expect_diffs).toEqual([]);
        expect(rec.summary.expect).toBe("pass");
      });

      const pull = scenario.effectors.length === 1 ? scenario.effectors[0] : undefined;
      if (pull && pull.kind === "prescribed_force" && pull.waveform === "constant" && pull.force_n[0] > 0) {
        it("informa (sin afirmar) la predicción de pivote ideal frente a la simulación", async () => {
          const obstacle = scenario.props[0];
          const trial = {
            obstacleHeightM: obstacle ? obstacle.pos_m[2] + obstacle.size_m[2] / 2 : 0,
            horizontalForceN: pull.force_n[0],
            upwardRatio: pull.force_n[2] / pull.force_n[0],
            durationS: pull.duration_s,
          };
          const ideal = idealPivotPrediction(trial);
          const rec = await runScenario(scenario, { engineVersion: "test" });
          console.log(
            `${scenario.id}: pivote ideal ${ideal.tips ? "vuelca" : "no vuelca"} ` +
            `(E ${ideal.initialKineticEnergyJ.toFixed(1)} J, barrera ${ideal.potentialBarrierJ.toFixed(1)} J); simulado ${rec.summary.outcome}`,
          );
          expect(Number.isFinite(ideal.initialKineticEnergyJ)).toBe(true);
        });
      }

      const expected = scenario.expect?.outcome;
      if (expected === "fall" || expected === "standing") {
        const factor = expected === "fall" ? 1 / FALL_MARGIN : 1 / STAND_MARGIN;
        it(`margen: con las fuerzas ×${factor.toFixed(3)} sigue cumpliendo su expect (${expected === "fall" ? "umbral <= F/1,3" : "umbral >= F/0,6"})`, async () => {
          const rec = await runScenario(scaled(scenario, factor), { engineVersion: "test" });
          expect(rec.summary.expect_diffs).toEqual([]);
          expect(rec.summary.expect).toBe("pass");
        });
      }
    });
  }
});
