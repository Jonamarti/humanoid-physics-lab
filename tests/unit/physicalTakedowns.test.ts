import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { scenarioToriBody } from "../../src/scenario/buildModel";
import { toriSegmentCommand, toriSkeleton } from "../../src/effectors/toriBody";
import type { HumanoidActor, Scenario, Vec3 } from "../../src/scenario/schema";
import { jointGroupOf, scaledGains } from "../../src/uke/gains";
function load(id:string):Scenario{return validateScenario(JSON.parse(readFileSync(`scenarios/physical_refs/${id}.json`,"utf8")));}
describe("referencias físicas: fuerzas explicables, adquisición y reproducibilidad",()=>{
  for(const id of ["outer_reap","leg_drive"]) {
    it(`${id}: endurecer el solver reduce la excursión sin ampliar los rangos`,async()=>{
      const tight=load(id);const old=structuredClone(tight);delete (old.actors[0] as HumanoidActor).joint_limit_timeconst_s;
      const a=await runScenario(old);const b=await runScenario(tight);
      expect(b.diagnostics!.joint_limit_excursion_rad!.value).toBeLessThan(a.diagnostics!.joint_limit_excursion_rad!.value);
      const ranges=(xml:string)=>Array.from(xml.matchAll(/ range="([^"]+)"/g),m=>m[1]);
      expect(ranges(b.model.mjcf)).toEqual(ranges(a.model.mjcf));
    },30_000);
    it(`${id}: ambos agarres se adquieren y cada reacción llega al antebrazo físico`,async()=>{
      const s=load(id); const params=scenarioToriBody(s)!;
      const snapshots:Array<{f:number[];pos:number[];vel:number[];rot:number[];inertia:number[];ids:number[]}> = [];
      const r=await runScenario(s,{onSample:(m,d)=>{
        const model=m as {body(n:string):{id:number};body_inertia:Float64Array};
        const data=d as {xfrc_applied:Float64Array;xipos:Float64Array;cvel:Float64Array;xmat:Float64Array};
        snapshots.push({f:Array.from(data.xfrc_applied),pos:Array.from(data.xipos),vel:Array.from(data.cvel),rot:Array.from(data.xmat),inertia:Array.from(model.body_inertia),ids:[model.body("tori_left_forearm").id,model.body("tori_right_forearm").id]});
      }});
      const triple=(a:number[],o:number):Vec3=>[a[o]!,a[o+1]!,a[o+2]!];
      let checked=0;
      r.samples.forEach((sample,i)=>{
        const snap=snapshots[i]!;
        for(const [j,side] of ["left","right"].entries()) {
          const e=sample.effectors.find(x=>x.id===`tori_${side}_hand`)!;
          if(Math.hypot(...e.force_n)<1)continue;
          const b=snap.ids[j]!;const segment=params.segments.find(x=>x.segment===`${side}_forearm`)!;
          const now=toriSkeleton(params,sample.t)[segment.segment], next=toriSkeleton(params,sample.t+.002)[segment.segment];
          const command=toriSegmentCommand(segment,{position_m:triple(snap.pos,3*b),velocity_mps:triple(snap.vel,6*b+3),angular_velocity_rps:triple(snap.vel,6*b),axis:[snap.rot[9*b+2]!,snap.rot[9*b+5]!,snap.rot[9*b+8]!],inertia_transverse:Math.max(snap.inertia[3*b]!,snap.inertia[3*b+1]!),inertia_axial:snap.inertia[3*b+2]!},now,next,.002);
          for(let k=0;k<3;k++)expect(snap.f[6*b+k]!-command.force_n[k]!+e.force_n[k]!).toBeCloseTo(0,7);
          const arm=e.point_m.map((x,k)=>x-snap.pos[3*b+k]!) as unknown as Vec3;
          const reaction=e.coupled_grip!.reaction_n;
          const torque=[arm[1]*reaction[2]-arm[2]*reaction[1],arm[2]*reaction[0]-arm[0]*reaction[2],arm[0]*reaction[1]-arm[1]*reaction[0]];
          for(let k=0;k<3;k++)expect(snap.f[6*b+3+k]!-command.torque_nm[k]!-torque[k]!).toBeCloseTo(0,7);
          checked++;
        }
      });
      expect(checked).toBeGreaterThan(10);
      for(const p of Object.values(r.peaks.effectors)){
        expect(p.grip_events?.some(x=>x.status==="held")).toBe(true);
        expect(p.max_force_n).toBeLessThanOrEqual(400);expect(p.max_power_w).toBeLessThanOrEqual(600);
      }
      expect(r.meta.tori_assist).toBe("independent_segment_tracking");
      expect(r.summary.uke?.max_root_generalized_force).toBe(0);
      const actor=s.actors[0] as HumanoidActor;const gains=scaledGains(actor.mass_kg,actor.height_m,actor.strength);
      for(const sample of r.samples)for(const [name,tau] of Object.entries(sample.joint_torques_nm??{}))expect(Math.abs(tau)).toBeLessThanOrEqual(gains.groups[jointGroupOf(name)].maxNm+1e-9);
      const again=await runScenario(s);expect(again.samples).toEqual(r.samples);
    },30_000);
    it(`${id}: fuera de alcance no agarra ni derriba`,async()=>{
      const s=load(id);
      for(const k of s.tori!.body!.keys){k.pelvis_m=[k.pelvis_m[0]+1,k.pelvis_m[1],k.pelvis_m[2]];k.left_foot_m=[k.left_foot_m[0]+1,k.left_foot_m[1],k.left_foot_m[2]];k.right_foot_m=[k.right_foot_m[0]+1,k.right_foot_m[1],k.right_foot_m[2]];}
      const r=await runScenario(s);
      expect(r.summary.outcome).toBe("standing");
      for(const p of Object.values(r.peaks.effectors)){expect(p.max_force_n).toBe(0);expect(p.grip_events?.some(x=>x.status==="held")).toBe(false);}
      expect(r.peaks.tori_body!.max_contact_force_on_actor_n).toBe(0);
    },30_000);
  }
});
