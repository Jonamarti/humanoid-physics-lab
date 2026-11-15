import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { gzipRecording } from "../../src/recording/write";
import type { HumanoidActor, Scenario } from "../../src/scenario/schema";
function base(): Scenario { return validateScenario(JSON.parse(readFileSync("scenarios/uke_stand_75_175_neutral.json","utf8"))); }
describe("respuesta protectora experimental, con física real",()=>{
  it("no se activa en reposo ni en un empujón recuperable",async()=>{
    for(const impulse of [0,6,-6,25]) {
      const s=base();s.duration_s=5;delete s.expect;(s.actors[0] as HumanoidActor).fall="instinctive";
      if(impulse) s.effectors=[{id:"push",kind:"prescribed_force",actor:"uke",body:"chest",site:"sternum",local_point_m:[0,0,0],frame:"world",force_n:[impulse*Math.PI/.2,0,0],start_s:.5,duration_s:.1,waveform:"half_sine"}];
      const r=await runScenario(s);
      expect(r.summary.outcome).toBe("standing");
      expect(r.summary.uke?.controller.fall_response).toMatchObject({phase:"posture",activated_s:null});
      expect(r.summary.uke?.max_root_generalized_force).toBe(0);
    }
  },30_000);
  it("modifica pares internos tras una pérdida sostenida, registra los impactos y no garantiza éxito",async()=>{
    const rows=[];
    for(const direction of [-1,1]) for(const response of ["none","instinctive"] as const) {
      const s=base();s.id=`fall_${direction<0?"back":"front"}_${response}`;s.duration_s=5;delete s.expect;(s.actors[0] as HumanoidActor).fall=response;
      s.effectors=[{id:"push",kind:"prescribed_force",actor:"uke",body:"chest",site:"sternum",local_point_m:[0,0,0],frame:"world",force_n:[direction*(direction > 0 ? 55 : 25)*Math.PI/.2,0,0],start_s:.5,duration_s:.1,waveform:"half_sine"}];
      const r=await runScenario(s);
      expect(r.summary.uke?.max_root_generalized_force).toBe(0);
      expect(r.samples.every(x=>x.qpos.every(Number.isFinite))).toBe(true);
      if(response==="instinctive") {
        const st=r.summary.uke!.controller.fall_response as {activated_s:number;max_torque_ratio:number};
        expect(st.activated_s).toBeGreaterThan(.5);expect(st.max_torque_ratio).toBeLessThanOrEqual(1);
      }
      rows.push({id:s.id,summary:r.summary,diagnostics:r.diagnostics});
      if(process.env.HPL_FALL_RESULTS){mkdirSync(process.env.HPL_FALL_RESULTS,{recursive:true});writeFileSync(`${process.env.HPL_FALL_RESULTS}/${s.id}.json.gz`,gzipRecording(r));}
    }
    if(process.env.HPL_FALL_RESULTS) writeFileSync(`${process.env.HPL_FALL_RESULTS}/results.json`,JSON.stringify(rows,null,2));
    console.log("FALL_COMPARISON",JSON.stringify(rows.map(r=>({id:r.id,outcome:r.summary.outcome,impact:r.summary.uke?.fall?.impact,contact_peak:r.diagnostics?.contact_force,ground_pen:r.diagnostics?.ground_penetration,reflex:r.summary.uke?.controller.fall_response}))));
  },30_000);
});
