import { readFileSync, mkdirSync, writeFileSync } from "node:fs";
import { it } from "vitest";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { getMujoco } from "../../src/scenario/mujocoModule";
import { applyStance } from "../../src/uke/stances";
import { resolveToriBodyParams, toriHandPoints, toriWorldToHand } from "../../src/effectors/toriBody";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { gzipRecording } from "../../src/recording/write";
import type { HumanoidActor, Scenario, Vec3 } from "../../src/scenario/schema";
it.skipIf(process.env.HPL_PREPARE_REFS !== "1")("prepara agarres alcanzables, conserva los desplazamientos de la pista", async () => {
  mkdirSync("scenarios/physical_refs", { recursive: true });
  mkdirSync("out/takedowns/prepared", { recursive: true });
  const mj = await getMujoco();
  for (const [family, id] of [["siega_exterior", "outer_reap"], ["entrada_piernas", "leg_drive"]]) {
    const s = validateScenario(JSON.parse(readFileSync(`scenarios/uke_75_175_tori_${family}.json`, "utf8")));
    s.id = id!; s.duration_s = 6; s.record_hz = 125; delete s.expect;
    const a = s.actors[0] as HumanoidActor;
    a.joint_limit_timeconst_s = 0.004;
    if (family === "siega_exterior") a.stance = "natural_grip";
    const body = s.tori!.body!; body.grip_mode = "coupled";
    if (family === "siega_exterior") body.grips!.right!.site = "sternum";
    const params = resolveToriBodyParams({ mass_kg: s.tori!.mass_kg, ...body });
    const built = buildScenarioModel({ ...s, tori: undefined } as Scenario);
    const m = mj.MjModel.from_xml_string(built.mjcf); const d = new mj.MjData(m);
    try {
      applyStance(mj, m, d, a.stance); mj.mj_forward(m, d);
      for (const side of ["left", "right"] as const) {
        const g = body.grips![side]!;
        const site = m.site(g.site).id; const p = d.site_xpos as Float64Array;
        const target: Vec3 = [p[3*site]!, p[3*site+1]!, p[3*site+2]!];
        // The leg entry closes when the wrists arrive, rather than after they have already passed the knees.
        if (family === "entrada_piernas") g.engage_s = 0.85;
        const engage = g.engage_s ?? 0;
        const at = toriHandPoints(params, engage)[side];
        const delta: Vec3 = [target[0]-at[0], target[1]-at[1], target[2]-at[2]];
        const worlds = body.keys.map(k => toriHandPoints(params, k.t_s)[side]);
        for (let i=0; i<body.keys.length; i++) {
          const k = body.keys[i]!; const w = worlds[i]!;
          k[`${side}_hand_m`] = toriWorldToHand(params, k, [w[0]+delta[0], w[1]+delta[1], w[2]+delta[2]]);
        }
      }
    } finally { d.delete(); m.delete(); }
    if (family === "entrada_piernas") {
      // Symmetric entry: both hands follow the same sagittal approach to the two knees.
      for (const k of body.keys) {
        const left = toriHandPoints(params, k.t_s).left;
        k.right_hand_m = toriWorldToHand(params, k, [left[0], -left[1], left[2]]);
      }
    }
    for (const g of Object.values(body.grips!)) g.release_s = Math.min(g.release_s ?? 2.2, 2.2);
    writeFileSync(`scenarios/physical_refs/${id}.json`, JSON.stringify(s, null, 2));
    const r = await runScenario(validateScenario(s), { engineVersion: "prepared-refs" });
    writeFileSync(`out/takedowns/prepared/${id}.json.gz`, gzipRecording(r));
    console.log(id, JSON.stringify({ summary: r.summary, grips: r.peaks.effectors }));
  }
}, 60_000);
