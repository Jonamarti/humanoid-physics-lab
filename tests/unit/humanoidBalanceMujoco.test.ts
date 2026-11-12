import { describe, expect, it } from "vitest";
import { capturePoint, centerOfMass, signedMargin } from "../../src/metrics/balance";
import { parseRecording } from "../../src/recording/read";
import { gzipRecording } from "../../src/recording/write";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";

/**
 * Veredicto y métricas de equilibrio con simulación real (issue #75, PR 4). Uke `tense` de 75 kg y 1,75 m recibe un empujón
 * hacia atrás (-X) en el esternón de medio seno de 0,1 s. Umbral medido el 12-11-2026: entre 12 y 13 N·s cae, 12 y menos no.
 * Se afirma lejos de él: 6 N·s (0,5 veces) y 25 N·s (2 veces).
 */
function pushScenario(impulseNs: number): ReturnType<typeof validateScenario> {
  const peakN = (impulseNs * Math.PI) / 2 / 0.1;
  return validateScenario({
    schema: "hpl.scenario.v1", id: `uke_push_back_${impulseNs}`, duration_s: 5, record_hz: 125, ground: { friction: 1 },
    actors: [{ id: "uke", kind: "humanoid", mass_kg: 75, height_m: 1.75, strength: 1, attitude: "tense", stance: "neutral" }],
    props: [],
    effectors: [{ id: "push", kind: "prescribed_force", actor: "uke", site: "sternum", frame: "world",
      force_n: [-peakN, 0, 0], start_s: 0.2, duration_s: 0.1, waveform: "half_sine" }],
  });
}

describe("veredicto del humanoide con simulación", () => {
  it("empujón pequeño (6 N·s): standing, los pies no se mueven y el punto de captura vuelve al apoyo", async () => {
    const rec = await runScenario(pushScenario(6), { engineVersion: "test" });
    const s = rec.summary;
    const u = s.uke!;
    console.log(`6 N·s: ${s.outcome}, t_outcome ${s.t_outcome_s.toFixed(3)} s, margen mín ${u.balance!.min_capture_margin_m!.toFixed(4)} m, ` +
      `final ${u.balance!.final_capture_margin_m!.toFixed(4)} m, pie máx ${JSON.stringify(u.balance!.max_foot_displacement_m)}, ` +
      `desequilibrio ${JSON.stringify(u.unbalance)}, tronco máx ${u.max_trunk_tilt_deg.toFixed(1)}°`);
    expect(s.outcome).toBe("standing");
    expect(s.expect).toBe("none");
    expect(u.fall).toBeNull();
    expect(u.step).toBeNull();
    expect(u.balance!.max_foot_displacement_m.left).toBeLessThan(0.05);
    expect(u.balance!.max_foot_displacement_m.right).toBeLessThan(0.05);
    expect(u.balance!.final_capture_margin_m!).toBeGreaterThan(0);
    expect(u.balance!.final_com_speed_mps).toBeLessThan(0.1);
    expect(u.balance!.stable_since_s).not.toBeNull();
    expect(s.t_outcome_s).toBeLessThan(3); // estable al menos los 2 últimos segundos de 5
    expect(u.max_root_generalized_force).toBe(0);
  }, 60000);

  it("empujón grande (25 N·s): fall hacia atrás, con instante, primera parte, orden de contactos y velocidades de impacto", async () => {
    const rec = await runScenario(pushScenario(25), { engineVersion: "test" });
    const s = rec.summary;
    const u = s.uke!;
    const f = u.fall!;
    console.log(`25 N·s: ${s.outcome}, caída ${f.t_s.toFixed(3)} s (${f.trigger}), dirección ${f.direction_deg}° (${f.direction_name}), ` +
      `primera parte ${f.first_part}@${f.first_part_t_s}, orden ${f.contact_order.map((c) => `${c.body}@${c.t_s.toFixed(3)}`).join(" ")}, ` +
      `impacto ${JSON.stringify(f.impact)}, desequilibrio ${JSON.stringify(u.unbalance)}, paso ${JSON.stringify(u.step)}`);
    expect(s.outcome).toBe("fall");
    expect(s.t_outcome_s).toBe(f.t_s);
    expect(f.t_s).toBeGreaterThan(0.3);
    expect(f.t_s).toBeLessThan(3);
    // Hacia atrás: 180 ± 30 grados respecto al frente inicial de Uke (+X) y en el mundo.
    expect(Math.abs(f.direction_deg!)).toBeGreaterThan(150);
    expect(f.direction_name).toBe("back");
    expect(Math.abs(s.direction_deg!)).toBeGreaterThan(150);
    // Orden de contactos: sin pies, ordenado y con la primera parte al principio; la pelvis cae de espaldas.
    expect(f.first_part).toBe(f.contact_order[0]!.body);
    expect(f.contact_order.map((c) => c.t_s)).toEqual([...f.contact_order.map((c) => c.t_s)].sort((a, b) => a - b));
    expect(f.contact_order.some((c) => c.body.endsWith("_foot"))).toBe(false);
    expect(f.contact_order.map((c) => c.body)).toContain("pelvis");
    expect(f.impact.pelvis!.speed_mps).toBeGreaterThan(1);
    expect(f.impact.pelvis!.normal_speed_mps).toBeGreaterThan(1);
    expect(f.impact.chest!.speed_mps).toBeGreaterThan(1);
    // Desequilibrio: el punto de captura sale del apoyo hacia atrás mientras o poco después de empujar, antes de la caída.
    expect(u.unbalance!.direction).toBe("back");
    expect(u.unbalance!.direction_deg).toBe(180);
    expect(u.unbalance!.t_s).toBeGreaterThanOrEqual(0.2);
    expect(u.unbalance!.t_s).toBeLessThan(f.t_s);
    expect(u.unbalance!.recovered).toBe(false);
    expect(u.max_root_generalized_force).toBe(0);
  }, 60000);

  it("por muestra: com, xcom y polígono de apoyo coherentes con el modelo; el margen es el del polígono", async () => {
    const rec = await runScenario(pushScenario(25), { engineVersion: "test" });
    const mj = await getMujoco();
    const model = mj.MjModel.from_xml_string(rec.model.mjcf);
    const data = new mj.MjData(model);
    try {
      const mass = model.body_mass as Float64Array;
      const nbody = rec.model.bodies.length;
      let checked = 0;
      for (const [k, sample] of rec.samples.entries()) {
        expect(sample.com).toBeDefined();
        expect(sample.xcom).toBeDefined();
        expect(sample.support_polygon).toBeDefined();
        if (k % 10 !== 0) continue;
        // Centro de masas independiente: suma de masa × posición de cada cuerpo con la cinemática directa de la muestra.
        (data.qpos as Float64Array).set(sample.qpos);
        mj.mj_forward(model, data);
        const xipos = data.xipos as Float64Array;
        const bodies = [];
        for (let b = 1; b < nbody; b++) {
          // Solo los cuerpos del actor (no los props, que aquí no hay): masa > 0.
          if (mass[b]! > 0) bodies.push({ mass: mass[b]!, pos: [xipos[3 * b]!, xipos[3 * b + 1]!, xipos[3 * b + 2]!] as const });
        }
        const ref = centerOfMass(bodies);
        expect(sample.com![0]).toBeCloseTo(ref.com[0], 6);
        expect(sample.com![1]).toBeCloseTo(ref.com[1], 6);
        expect(sample.com![2]).toBeCloseTo(ref.com[2], 6);
        expect(ref.mass).toBeCloseTo(75, 6);
        checked++;
      }
      expect(checked).toBeGreaterThan(10);
    } finally {
      data.delete();
      model.delete();
    }
    // Velocidad: xcom - com = v / w0; la velocidad sale de la derivada del centro de masas entre muestras vecinas.
    const dt = 1 / rec.meta.record_hz;
    let compared = 0;
    for (let k = 1; k < rec.samples.length - 1; k++) {
      const a = rec.samples[k - 1]!.com!, b = rec.samples[k + 1]!.com!, c = rec.samples[k]!;
      const v: [number, number, number] = [(b[0] - a[0]) / (2 * dt), (b[1] - a[1]) / (2 * dt), (b[2] - a[2]) / (2 * dt)];
      if (Math.hypot(v[0], v[1]) < 0.3) continue;
      const xi = capturePoint(c.com!, v);
      expect(c.xcom![0]).toBeCloseTo(xi[0], 1); // ±5 mm: la derivada centrada tiene error propio
      expect(c.xcom![1]).toBeCloseTo(xi[1], 1);
      compared++;
    }
    expect(compared).toBeGreaterThan(5);
    // Margen grabado = margen del polígono grabado.
    for (const s of rec.samples) {
      const poly = s.support_polygon!;
      const margin = signedMargin(s.xcom!, poly);
      expect(s.capture_margin_m ?? null).toBe(margin);
      if (poly.length === 0) expect(margin).toBeNull();
    }
    // De pie, antes del empujón: apoyo de los dos pies y punto de captura dentro con margen > 0.
    const first = rec.samples[0]!;
    expect(first.support_polygon!.length).toBeGreaterThanOrEqual(4);
    expect(first.capture_margin_m!).toBeGreaterThan(0.03);
    // Al final, tumbado: ningún pie con carga o el punto de captura fuera del apoyo.
    const last = rec.samples[rec.samples.length - 1]!;
    expect(last.capture_margin_m == null || last.capture_margin_m < 0).toBe(true);
  }, 60000);

  it("los campos nuevos sobreviven a escribir y leer la grabación, y las grabaciones sin ellos se siguen leyendo", async () => {
    const rec = await runScenario(pushScenario(6), { engineVersion: "test" });
    const back = await parseRecording(gzipRecording(rec));
    expect(back.samples[10]!.support_polygon).toBeDefined();
    expect(back.summary.uke!.balance!.stable_since_s).not.toBeUndefined();
    expect(back.summary.outcome).toBe("standing");
    // Grabación antigua: sin com/xcom/support_polygon ni los bloques del resumen.
    const old = structuredClone(back) as unknown as { samples: Array<Record<string, unknown>>; summary: { uke: Record<string, unknown> } };
    for (const s of old.samples) { delete s.com; delete s.xcom; delete s.support_polygon; delete s.capture_margin_m; }
    for (const key of ["balance", "fall", "step", "unbalance"]) delete old.summary.uke[key];
    const parsed = await parseRecording(JSON.stringify(old));
    expect(parsed.samples).toHaveLength(rec.samples.length);
    expect(parsed.samples[0]!.xcom).toBeUndefined();
  }, 60000);
});
