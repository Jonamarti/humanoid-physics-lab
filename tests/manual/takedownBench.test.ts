import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { gzipRecording } from "../../src/recording/write";
import type { HumanoidActor, Scenario } from "../../src/scenario/schema";

// Same matrix before and after; outcomes are observations, never forced expectations.
it.skipIf(process.env.HPL_TAKEDOWN_BENCH !== "1")("mide dos mecanismos con ablaciones y variaciones físicas", async () => {
  const out = resolve(process.env.HPL_RESULTS_DIR ?? "out/takedowns");
  mkdirSync(out, { recursive: true });
  const rows: unknown[] = [];
  for (const family of ["siega_exterior", "entrada_piernas"]) {
    const sourcePath = process.env.HPL_REF_SUITE === "1" ? `scenarios/physical_refs/${family === "siega_exterior" ? "outer_reap" : "leg_drive"}.json` : `scenarios/uke_75_175_tori_${family}.json`;
    const source = JSON.parse(readFileSync(sourcePath, "utf8")) as Scenario;
    const variants: Array<[string, (s: Scenario, a: HumanoidActor) => void]> = [
      ["nominal", () => {}],
      ["passive", (_s, a) => { a.attitude = "relaxed"; }],
      ["defensive", (_s, a) => { a.stance = "defensive"; }],
      ["heavy", (_s, a) => { a.mass_kg = 120; }],
      ["light", (_s, a) => { a.mass_kg = 55; }],
      ["tall", (_s, a) => { a.height_m = 1.95; }],
      ["slippery", (s) => { s.ground.friction = 0.2; }],
      ["weak_grip", (s) => { for (const g of Object.values(s.tori!.body!.grips!)) g.max_force_n = 40; }],
      ["no_grips", (s) => { s.tori!.body!.grips = {}; }],
      ["out_of_reach", (s) => { for (const k of s.tori!.body!.keys) { k.pelvis_m = [k.pelvis_m[0]+1, k.pelvis_m[1], k.pelvis_m[2]]; k.left_foot_m = [k.left_foot_m[0]+1, k.left_foot_m[1], k.left_foot_m[2]]; k.right_foot_m = [k.right_foot_m[0]+1, k.right_foot_m[1], k.right_foot_m[2]]; } }],
      ["no_attacker", (s) => { delete s.tori; }],
    ];
    if (process.env.HPL_REF_SUITE === "1") variants.push(
      ["protective", (_s,a) => { a.fall = "instinctive"; }],
      ["small_attacker", s => { s.tori!.mass_kg = 55; }],
      ["slow_action", s => { for (const k of s.tori!.body!.keys) k.t_s *= 1.4; for(const g of Object.values(s.tori!.body!.grips!)){ if(g.engage_s!==undefined)g.engage_s*=1.4;if(g.release_s!==undefined)g.release_s*=1.4; } }],
      ["oblique", s => { const angle=Math.PI/3,rotate=(v:readonly number[]):readonly[number,number,number]=>[Math.cos(angle)*v[0]!-Math.sin(angle)*v[1]!,Math.sin(angle)*v[0]!+Math.cos(angle)*v[1]!,v[2]!];for(const k of s.tori!.body!.keys){k.pelvis_m=rotate(k.pelvis_m);k.left_foot_m=rotate(k.left_foot_m);k.right_foot_m=rotate(k.right_foot_m);k.yaw_deg+=60;} }],
      ["proportions", (_s,a) => { a.segment_lengths_m = { thigh:.45,shin:.37,trunk:.55 }; }],
      ["passive_no_attacker", (s,a) => { delete s.tori; a.attitude="relaxed"; }],
    );
    for (const [variant, modify] of variants) {
      const s = structuredClone(source);
      s.id = `${family}_${variant}`;
      s.duration_s = 6;
      s.record_hz = 50;
      delete s.expect;
      modify(s, s.actors[0] as HumanoidActor);
      // The environment toggle changes only the grip model, preserving the identical matrix.
      if (process.env.HPL_COUPLED_GRIPS === "1" && s.tori?.body) Object.assign(s.tori.body, { grip_mode: "coupled" });
      const valid = validateScenario(s);
      const start = performance.now();
      const r = await runScenario(valid, { engineVersion: "takedown-bench" });
      const elapsed = performance.now() - start;
      expect(r.samples.every((x) => [...x.qpos, ...x.qvel].every(Number.isFinite))).toBe(true);
      expect(r.summary.uke?.max_root_generalized_force).toBe(0);
      writeFileSync(resolve(out, `${s.id}.json.gz`), gzipRecording(r));
      writeFileSync(resolve(out, `${s.id}.scenario.json`), JSON.stringify(valid, null, 2));
      const row = { id: s.id, outcome: r.summary.outcome, fall_s: r.summary.uke?.fall?.t_s,
        contact_peak_n: r.peaks.tori_body?.max_contact_force_on_actor_n ?? 0,
        hands: r.peaks.effectors, diagnostics: r.diagnostics, summary: r.summary,
        final_speed_mps: Math.hypot(...r.samples.at(-1)!.qvel.slice(0, 3)), elapsed_ms: elapsed };
      rows.push(row);
      writeFileSync(resolve(out, "results.json"), JSON.stringify(rows, null, 2));
      writeFileSync(resolve(out, "campaign.json"), JSON.stringify({node:process.version,platform:process.platform,mujoco:r.meta.mujoco_version,physics_hz:r.meta.physics_hz,record_hz:r.meta.record_hz,source:sourcePath,coupled:process.env.HPL_COUPLED_GRIPS === "1",prepared:process.env.HPL_REF_SUITE === "1",rows:rows.length},null,2));
      console.log(JSON.stringify({ id: s.id, outcome: row.outcome, contact_n: row.contact_peak_n,
        ground_pen_m: r.diagnostics?.ground_penetration.value, hand_n: Object.values(r.peaks.effectors).map(p => p.max_force_n),
        artifacts: r.summary.artifacts?.map(x => x.type), final_speed: row.final_speed_mps }));
    }
  }
}, 180_000);
