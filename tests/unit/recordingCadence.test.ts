import {readFileSync} from "node:fs";
import {expect,it} from "vitest";
import {runScenario} from "../../src/scenario/run";
import {validateScenario} from "../../src/scenario/validate";
it("la frecuencia de grabación no cambia las trayectorias de los dos mecanismos",async()=>{
  for(const id of ["outer_reap","leg_drive"]){
    const s=validateScenario(JSON.parse(readFileSync(`scenarios/physical_refs/${id}.json`,"utf8")));
    s.record_hz=50;const a=await runScenario(s);s.record_hz=125;const b=await runScenario(s);
    const byTime=new Map(b.samples.map(x=>[x.t.toFixed(6),x]));let common=0;
    for(const sample of a.samples){const other=byTime.get(sample.t.toFixed(6));if(other){expect(other.qpos).toEqual(sample.qpos);expect(other.qvel).toEqual(sample.qvel);common++;}}
    expect(common).toBeGreaterThan(100);expect(a.summary.outcome).toBe(b.summary.outcome);
    for(const id of Object.keys(a.peaks.effectors)){expect(a.peaks.effectors[id]).toEqual(b.peaks.effectors[id]);}
  }
},30_000);
