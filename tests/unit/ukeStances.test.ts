import { describe, expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";

/**
 * Cuatro tallas x cinco posturas, 10 s de pie en `tense`, con los criterios de la issue #75: tronco < 3°, pelvis > 90 % de
 * su altura inicial, asentamiento < 2 cm/s. Las combinaciones que HOY no los cumplen (medido, ver `docs/UKE_CONTROL.md`)
 * están marcadas con `it.fails` para que un cambio que las arregle (o las rompa) se note.
 */
const SIZES: ReadonlyArray<readonly [string, number, number]> = [
  ["55_155", 55, 1.55], ["75_175", 75, 1.75], ["100_195", 100, 1.95], ["150_200", 150, 2.0],
];
const STANCES = ["neutral", "natural_grip", "defensive", "staggered_left", "staggered_right"] as const;

/** Combinaciones medidas que incumplen algún criterio. */
const KNOWN_FAILING = new Set([
  "55_155 defensive", "75_175 defensive", "100_195 defensive", "150_200 defensive", // tronco 3,5-4,3° y asentamiento 4,7-6,6 cm/s
  "150_200 natural_grip", "150_200 staggered_left", "150_200 staggered_right", // asentamiento 2,02-2,04 cm/s (límite 2)
]);

describe("Uke tense: tallas x posturas (MuJoCo WASM real)", () => {
  for (const [label, mass, height] of SIZES) {
    for (const stance of STANCES) {
      const key = `${label} ${stance}`;
      const run = KNOWN_FAILING.has(key) ? it.fails : it;
      run(`${key}`, async () => {
        const scenario = validateScenario({
          schema: "hpl.scenario.v1", id: "uke_bench", duration_s: 10, record_hz: 50, ground: { friction: 1 },
          actors: [{ id: "uke", kind: "humanoid", mass_kg: mass, height_m: height, strength: 1, attitude: "tense", stance }],
          props: [], effectors: [],
        });
        const rec = await runScenario(scenario, { engineVersion: "test" });
        const u = rec.summary.uke!;
        console.log(`${key}: ${rec.summary.outcome}, tronco ${u.max_trunk_tilt_deg.toFixed(2)}°, pelvis mín ` +
          `${(100 * u.min_pelvis_height_m / u.initial_pelvis_height_m).toFixed(2)} %, asentamiento ${(100 * u.settle_com_speed_mps).toFixed(2)} cm/s`);
        expect(u.max_trunk_tilt_deg).toBeLessThan(3);
        expect(u.min_pelvis_height_m / u.initial_pelvis_height_m).toBeGreaterThan(0.9);
        expect(u.settle_com_speed_mps).toBeLessThan(0.02);
        expect(u.max_root_generalized_force).toBe(0);
      }, 60000);
    }
  }
});
