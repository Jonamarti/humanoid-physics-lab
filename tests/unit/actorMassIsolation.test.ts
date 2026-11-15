import { readFileSync } from "node:fs";
import { expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import type { Scenario } from "../../src/scenario/schema";
it("Uke no sostiene la masa de Tori: un atacante distante no altera la respuesta al mismo empujón", async()=>{
  const source = JSON.parse(readFileSync("scenarios/uke_75_175_tori_siega_exterior.json","utf8")) as Scenario;
  const alone=structuredClone(source); delete alone.tori;delete alone.expect; alone.duration_s=3;
  alone.effectors=[{id:"push",kind:"prescribed_force",actor:"uke",body:"chest",local_point_m:[.12,0,.1],frame:"world",force_n:[100,0,0],start_s:.5,duration_s:.1,waveform:"half_sine"}];
  const far=structuredClone(alone);far.tori=structuredClone(source.tori!);far.tori.mass_kg=150;far.tori.body!.grips={};
  for(const k of far.tori.body!.keys){k.pelvis_m=[k.pelvis_m[0]+3,k.pelvis_m[1],k.pelvis_m[2]];k.left_foot_m=[k.left_foot_m[0]+3,k.left_foot_m[1],k.left_foot_m[2]];k.right_foot_m=[k.right_foot_m[0]+3,k.right_foot_m[1],k.right_foot_m[2]];}
  const a=await runScenario(validateScenario(alone));const b=await runScenario(validateScenario(far));
  let error=0;
  a.samples.forEach((sample,i)=>sample.qpos.forEach((q,j)=>{error=Math.max(error,Math.abs(q-b.samples[i]!.qpos[j]!));}));
  expect(b.peaks.tori_body!.max_contact_force_on_actor_n).toBe(0);
  expect(a.summary.outcome).toBe("standing");expect(b.summary.outcome).toBe("standing");
  expect(error).toBeLessThan(1e-7);
},30_000);
