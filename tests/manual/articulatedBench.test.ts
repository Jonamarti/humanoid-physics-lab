import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {expect,it} from "vitest";
import {runScenario} from "../../src/scenario/run";
import {validateScenario} from "../../src/scenario/validate";
import {gzipRecording} from "../../src/recording/write";
import type {HumanoidActor,Scenario} from "../../src/scenario/schema";
it.skipIf(process.env.HPL_ARTICULATED_BENCH!=="1")("mide actores articulados con controles negativos y registra fallos",async()=>{
  const out=resolve("out/articulated");mkdirSync(out,{recursive:true});
  const source=(id:string)=>JSON.parse(readFileSync("scenarios/articulated_refs/"+id+".json","utf8")) as Scenario;
  const cases:Scenario[]=[source("two_standing"),source("wrist_load"),source("wrist_miss"),source("secondary_passive")];
  for(const [name,mutate] of [
    ["heavy_second",(s:Scenario)=>{(s.actors[1] as HumanoidActor).mass_kg=120;}],
    ["tall_second",(s:Scenario)=>{(s.actors[1] as HumanoidActor).height_m=1.95;}],
    ["rotated_second",(s:Scenario)=>{(s.actors[1] as HumanoidActor).placement={xy_m:[.8,0],heading_deg:90};}],
    ["slippery",(s:Scenario)=>{s.ground.friction=.2;}],
  ] as const){const s=source("two_standing");s.id=name;s.duration_s=4;mutate(s);cases.push(s);}
  const noGrip=source("wrist_load");noGrip.id="load_no_grip";noGrip.actor_grips=[];cases.push(noGrip);
  const noLoad=source("wrist_load");noLoad.id="grip_no_load";noLoad.effectors=[];cases.push(noLoad);
  const far=source("wrist_load");far.id="grip_out_of_reach";(far.actors[1] as HumanoidActor).placement={xy_m:[0,1.2],heading_deg:0};cases.push(far);
  const protection=source("secondary_passive");protection.id="secondary_protective";(protection.actors[1] as HumanoidActor).fall="instinctive";cases.push(protection);
  const rows=[];
  for(const input of cases){const s=validateScenario(input);const start=performance.now();const r=await runScenario(s);const elapsed=performance.now()-start;
    expect(r.samples.every(x=>[...x.qpos,...x.qvel].every(Number.isFinite))).toBe(true);
    for(const a of Object.values(r.summary.actors!))expect(a.max_root_generalized_force).toBe(0);
    writeFileSync(resolve(out,s.id+".json.gz"),gzipRecording(r));
    rows.push({id:s.id,duration_s:s.duration_s,elapsed_ms:elapsed,actors:r.summary.actors,grips:r.peaks.effectors});
    console.log(JSON.stringify({id:s.id,actors:Object.fromEntries(Object.entries(r.summary.actors!).map(([id,a])=>[id,a.outcome])),grip:r.peaks.effectors.wrist?.max_force_n??0}));
  }
  const metadata={physics_hz:500,node:process.version,mujoco:"3.17.0",cases:rows.length};
  writeFileSync(resolve(out,"metrics.json"),JSON.stringify({metadata,rows},null,2));
  const csv=["scenario,actor,outcome,max_tilt_deg,min_pelvis_m,root_force,ground_pen_m,interbody_pen_m,contact_n,joint_limit_rad,foot_slip_m,artifact_types"];
  for(const r of rows)for(const [id,a] of Object.entries(r.actors!))csv.push([r.id,id,a.outcome,a.max_trunk_tilt_deg,a.min_pelvis_height_m,a.max_root_generalized_force,a.diagnostics.ground_penetration.value,a.diagnostics.interbody_penetration_m?.value??0,a.diagnostics.contact_force.value,a.diagnostics.joint_limit_excursion_rad?.value??0,a.diagnostics.foot_slip?.value??0,a.artifacts?.map(x=>x.type).join("|")??""].join(","));
  writeFileSync(resolve(out,"metrics.csv"),csv.join("\n")+"\n");
},120_000);
