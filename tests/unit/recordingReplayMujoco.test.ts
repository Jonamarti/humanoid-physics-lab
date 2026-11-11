/**
 * Prueba con WASM real: reproducir una grabación con `mj_forward` (sin `mj_step`) da las mismas posiciones
 * de cuerpo (`xpos`) que la simulación que la produjo. Demuestra que la reproducción no necesita física.
 */
import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { RecordingPlayer } from "../../src/recording/player";
import { parseRecording } from "../../src/recording/read";
import { ReplayKinematics } from "../../src/recording/replayKinematics";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";

const FIXTURE = new URL("../fixtures/cylinder_obstacle_low.recording.json.gz", import.meta.url);
const SCENARIO = new URL("../../scenarios/cylinder_obstacle_low.json", import.meta.url);

function maxAbsDiff(a: ArrayLike<number>, b: ArrayLike<number>): number {
  let max = 0;
  for (let i = 0; i < a.length; i++) max = Math.max(max, Math.abs(a[i]! - b[i]!));
  return max;
}

describe("reproducción con mj_forward (WASM real)", () => {
  it("coincide con la simulación: xpos igual en cada muestra (error < 1e-6)", async () => {
    const mj = await getMujoco();
    const scenario = validateScenario(JSON.parse(readFileSync(SCENARIO, "utf8")));
    const fixture = await parseRecording(new Uint8Array(readFileSync(FIXTURE)));

    // Simulación de referencia: xpos real de MuJoCo en cada muestra grabada.
    const simXpos: Float64Array[] = [];
    const fresh = await runScenario(scenario, {
      onSample: (_model, data) => simXpos.push(Float64Array.from((data as { xpos: Float64Array }).xpos)),
    });
    expect(simXpos.length).toBe(fresh.samples.length);
    expect(fresh.samples.length).toBe(fixture.samples.length);
    expect(fresh.model.mjcf).toBe(fixture.model.mjcf);

    const kin = new ReplayKinematics(mj, fixture.model.mjcf);
    try {
      expect(kin.nq).toBe(fixture.samples[0]!.qpos.length);
      const player = new RecordingPlayer(fixture, kin.jointTypes());
      let worstFixture = 0;
      let worstFull = 0;
      let moved = 0;
      fixture.samples.forEach((s, i) => {
        // 1. Con la grabación del fichero (números redondeados a 6 cifras) a través del reproductor.
        kin.setQpos(player.sampleAt(s.t).qpos);
        worstFixture = Math.max(worstFixture, maxAbsDiff(kin.bodyPositions(), simXpos[i]!));
        // 2. Con la grabación sin redondear.
        kin.setQpos(fresh.samples[i]!.qpos);
        worstFull = Math.max(worstFull, maxAbsDiff(kin.bodyPositions(), simXpos[i]!));
        if (i > 0) moved = Math.max(moved, maxAbsDiff(simXpos[i]!, simXpos[0]!));
      });
      expect(moved).toBeGreaterThan(0.5); // el cilindro se mueve de verdad: la prueba no es trivial
      expect(worstFull).toBeLessThan(1e-9);
      expect(worstFixture).toBeLessThan(1e-6);
    } finally {
      kin.dispose();
    }
  }, 60_000);

  it("las poses interpoladas siguen siendo cinemática válida (geometrías finitas)", async () => {
    const mj = await getMujoco();
    const fixture = await parseRecording(new Uint8Array(readFileSync(FIXTURE)));
    const kin = new ReplayKinematics(mj, fixture.model.mjcf);
    try {
      const player = new RecordingPlayer(fixture, kin.jointTypes());
      expect(kin.geomSpecs().length).toBeGreaterThan(0);
      kin.setQpos(player.sampleAt(1.3037).qpos);
      for (const pose of kin.geomPoses()) {
        expect([...pose.position, ...pose.rotation].every(Number.isFinite)).toBe(true);
      }
    } finally {
      kin.dispose();
    }
  });
});
