import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import loadMujoco from "@mujoco/mujoco";
import { worldOptionTag } from "../../src/scenario/worldOptions";

type Mujoco = Awaited<ReturnType<typeof loadMujoco>>;

/** Opciones actuales de humanoid_step_v2.xml (cono piramidal por defecto). */
const V2_OPTION = `<option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>`;
/** Parámetros de contacto por geometría de humanoid_step_v2.xml, salvo el rozamiento. */
const V2_GEOM_CONTACT = `solref="0.015 1" solimp="0.9 0.95 0.001"`;

const SETTLE_S = 2;
const WINDOW_S = 1;

/**
 * Bloque de 75 kg sobre un plano inclinado `angleDeg` alrededor de Y.
 * El plano y el bloque comparten orientación; la bajada es hacia +X del plano.
 * Devuelve la velocidad media (mm/s) a lo largo de la bajada entre t=2 s y t=3 s.
 */
function rampSpeedMmPerS(mj: Mujoco, optionTag: string, mu: number, angleDeg: number): number {
  const a = angleDeg * Math.PI / 180;
  const half = 0.2;
  const px = Math.sin(a) * half;
  const pz = Math.cos(a) * half;
  const xml = `<mujoco model="ramp">
  ${optionTag}
  <worldbody>
    <geom name="ramp" type="plane" size="200 200 0.1" euler="0 ${angleDeg} 0"
      friction="${mu} 0.005 0.0001" ${V2_GEOM_CONTACT}/>
    <body name="block" pos="${px} 0 ${pz}" euler="0 ${angleDeg} 0">
      <freejoint/>
      <geom name="block_geom" type="box" size="${half} ${half} ${half}" mass="75"
        friction="${mu} 0.005 0.0001" ${V2_GEOM_CONTACT}/>
    </body>
  </worldbody>
</mujoco>`;
  const model = mj.MjModel.from_xml_string(xml);
  const data = new mj.MjData(model);
  try {
    const dt = model.opt.timestep;
    const dx = [Math.cos(a), 0, -Math.sin(a)] as const;
    const along = (): number => data.qpos[0]! * dx[0] + data.qpos[2]! * dx[2];
    const settleSteps = Math.round(SETTLE_S / dt);
    const windowSteps = Math.round(WINDOW_S / dt);
    for (let i = 0; i < settleSteps; i++) mj.mj_step(model, data);
    const s0 = along();
    for (let i = 0; i < windowSteps; i++) mj.mj_step(model, data);
    return (along() - s0) / WINDOW_S * 1000;
  } finally {
    data.delete();
    model.delete();
  }
}

const FRICTIONS = [0.5, 0.8, 1.0];
const deg = (mu: number): number => Math.atan(mu) * 180 / Math.PI;

describe("Prueba de la rampa: contacto con las opciones de escenarios (MuJoCo WASM real)", () => {
  const newOption = worldOptionTag();

  for (const mu of FRICTIONS) {
    it(`mu=${mu}: fluencia < 0,5 mm/s a atan(mu)-8 grados y desliza > 100 mm/s a atan(mu)+5 grados`, async () => {
      const mj = await loadMujoco();
      const below = rampSpeedMmPerS(mj, newOption, mu, deg(mu) - 8);
      const above = rampSpeedMmPerS(mj, newOption, mu, deg(mu) + 5);
      console.log(`rampa nueva mu=${mu}: ${(deg(mu) - 8).toFixed(2)} deg -> ${below.toFixed(4)} mm/s; ${(deg(mu) + 5).toFixed(2)} deg -> ${above.toFixed(1)} mm/s`);
      expect(Math.abs(below)).toBeLessThan(0.5);
      expect(above).toBeGreaterThan(100);
    });
  }

  it("informa (sin afirmar) el resultado con las opciones actuales de v2", async () => {
    const mj = await loadMujoco();
    for (const mu of FRICTIONS) {
      const below = rampSpeedMmPerS(mj, V2_OPTION, mu, deg(mu) - 8);
      const above = rampSpeedMmPerS(mj, V2_OPTION, mu, deg(mu) + 5);
      console.log(`rampa v2 mu=${mu}: ${(deg(mu) - 8).toFixed(2)} deg -> ${below.toFixed(4)} mm/s; ${(deg(mu) + 5).toFixed(2)} deg -> ${above.toFixed(1)} mm/s`);
      expect(Number.isFinite(below)).toBe(true);
      expect(Number.isFinite(above)).toBe(true);
    }
    // Mismos ángulos que la medición nativa de referencia de la issue #74 (mu=1).
    for (const angle of [30, 40]) {
      const oldSpeed = rampSpeedMmPerS(mj, V2_OPTION, 1, angle);
      const newSpeed = rampSpeedMmPerS(mj, newOption, 1, angle);
      console.log(`rampa mu=1 ${angle} deg: v2 ${oldSpeed.toFixed(4)} mm/s; nuevo ${newSpeed.toFixed(4)} mm/s`);
      expect(Number.isFinite(oldSpeed) && Number.isFinite(newSpeed)).toBe(true);
    }
  });

  it("informa (sin afirmar) el coste de 10 s del humanoide v2 con las opciones antiguas y nuevas", async () => {
    const mj = await loadMujoco();
    const xml = await readFile(new URL("../../public/models/humanoid_step_v2.xml", import.meta.url), "utf8");
    expect(xml).toContain(V2_OPTION);
    const run = (optionTag: string): { ms: number; steps: number } => {
      const model = mj.MjModel.from_xml_string(xml.replace(V2_OPTION, optionTag));
      const data = new mj.MjData(model);
      try {
        const steps = Math.round(10 / model.opt.timestep);
        const t0 = performance.now();
        for (let i = 0; i < steps; i++) mj.mj_step(model, data);
        return { ms: performance.now() - t0, steps };
      } finally {
        data.delete();
        model.delete();
      }
    };
    run(V2_OPTION); // calentamiento
    for (const [name, tag] of [["v2 antiguo", V2_OPTION], ["nuevo", worldOptionTag()], ["v2 antiguo", V2_OPTION], ["nuevo", worldOptionTag()]] as const) {
      const r = run(tag);
      console.log(`coste 10 s humanoide v2 (${name}): ${r.ms.toFixed(0)} ms para ${r.steps} pasos`);
      expect(r.ms).toBeGreaterThan(0);
    }
  });
});
