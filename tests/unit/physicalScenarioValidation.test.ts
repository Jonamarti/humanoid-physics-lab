import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { validateScenario } from "../../src/scenario/validate";
import { RecordingPlayer } from "../../src/recording/player";
import type { RecordingSample, EffectorSample } from "../../src/recording/schema";
import type { HumanoidActor, Scenario } from "../../src/scenario/schema";
const load=():Scenario=>JSON.parse(readFileSync("scenarios/physical_refs/outer_reap.json","utf8")) as Scenario;
describe("validación de controles físicos",()=>{
  it("acepta las referencias y rechaza límites no finitos, tiempos incompatibles y opciones ignoradas",()=>{
    expect(validateScenario(load()).tori?.body?.grip_mode).toBe("coupled");
    for(const value of [NaN,Infinity,-1,0.001,0.1]) {
      const s=load();(s.actors[0] as HumanoidActor).joint_limit_timeconst_s=value;
      expect(()=>validateScenario(s)).toThrow(/joint_limit_timeconst_s/);
    }
    const early=load();early.tori!.body!.grips!.left!.engage_s=2;early.tori!.body!.grips!.left!.release_s=1;
    expect(()=>validateScenario(early)).toThrow(/release_s/);
    const gap=load();gap.tori!.body!.grips!.left!.capture_radius_m=.3;
    expect(()=>validateScenario(gap)).toThrow(/break_distance/);
    const ignored=load();ignored.tori!.body!.grip_mode="target";ignored.tori!.body!.grips!.left!.max_power_w=100;
    expect(()=>validateScenario(ignored)).toThrow(/coupled/);
    const badFall=load();Object.assign(badFall.actors[0]!,{fall:"ukemi"});
    expect(()=>validateScenario(badFall)).toThrow(/fall/);
  });
});
it("el reproductor conserva el estado discreto del agarre sin inventar transiciones entre muestras",()=>{
  const e:EffectorSample={id:"grip",point_m:[0,0,0],force_n:[10,0,0],coupled_grip:{status:"held",hand_point_m:[.1,0,0],distance_m:.1,reaction_n:[-10,0,0],force_limited:false,power_limited:false}};
  const samples:RecordingSample[]=[{t:0,qpos:[0],qvel:[0],contacts:[],effectors:[e]},{t:1,qpos:[1],qvel:[1],contacts:[],effectors:[{...e,coupled_grip:{...e.coupled_grip!,status:"slipped"}}]}];
  const p=new RecordingPlayer({samples});
  expect(p.sampleAt(.5).effectors[0]!.coupled_grip!.status).toBe("held");
  expect(p.sampleAt(1).effectors[0]!.coupled_grip!.status).toBe("slipped");
  p.sampleAt(.5).effectors[0]!.coupled_grip!.status="released";
  expect(samples[0]!.effectors[0]!.coupled_grip!.status).toBe("held");
});
