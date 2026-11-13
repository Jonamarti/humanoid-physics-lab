import { gzipSync } from "node:zlib";
import { describe, expect, it } from "vitest";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
// @ts-expect-error Browser-only public JS intentionally has no TS declarations.
import { buildFulcrumScenario, buildCircularKuzushiScenario, expectedYawMoment } from "../../public/fulcrum-scenarios.js";

/** Just the world coordinates of the main skeleton points, not a saved recording. */
const BODY_NAMES=[
  "pelvis","abdomen","chest","head",
  "left_upperarm","left_forearm","right_upperarm","right_forearm",
  "left_thigh","left_shin","left_foot","right_thigh","right_shin","right_foot",
] as const;
type Snapshot={t:number;poses:[number,number,number][]; yaw_deg:number};
const r=(x:number)=>Number(x.toFixed(3));

async function runOne(opts:Record<string,unknown>,emitFrames=false){
  const sc=validateScenario(buildFulcrumScenario(opts));
  const model=buildScenarioModel(sc);
  expect(model.bodies).toContain("right_foot");
  const snaps:Snapshot[]=[];
  const ids=BODY_NAMES.map(name=>model.bodies.indexOf(name));
  const rec=await runScenario(sc,{
    engineVersion:"fulcrum-comparison",
    onSample:(_m,raw,index)=>{
      const data=raw as {xpos:Float64Array;xmat:Float64Array};
      const row=ids.map(i=>([r(data.xpos[3*i]!),r(data.xpos[3*i+1]!),r(data.xpos[3*i+2]!)] as [number,number,number]));
      const mat=9*ids[0]!;
      const yaw=Math.atan2(data.xmat[mat+3]!,data.xmat[mat]!)*180/Math.PI;
      if(index%5===0)snaps.push({t:r(index/50),poses:row,yaw_deg:r(yaw)});
    },
  });
  const original=snaps[0]!;
  const maxZ=(name:"left_foot"|"right_foot")=>Math.max(...snaps.map(s=>s.poses[BODY_NAMES.indexOf(name)]![2]));
  const footXY=(name:"left_foot"|"right_foot")=>{
    const k=BODY_NAMES.indexOf(name);
    return r(Math.hypot(...([0,1].map(i=>snaps.at(-1)!.poses[k]![i]-original.poses[k]![i]) as [number,number])));
  };
  const peakYaw=r(Math.max(...snaps.map(s=>Math.abs(s.yaw_deg))));
  const stopContacts=rec.samples.flatMap(s=>s.contacts.filter(c=>c.body_a==="foot_stop"||c.body_b==="foot_stop")
    .map(c=>({t:r(s.t),part:c.body_a==="foot_stop"?c.body_b:c.body_a,force:r(Math.hypot(...c.force_n))})));
  const stopParts=Array.from(new Set(stopContacts.map(c=>c.part)));

  const summary={
    id:sc.id,outcome:rec.summary.outcome,dir:rec.summary.direction_deg,
    max_tilt:r(rec.summary.max_tilt_deg),fall:rec.summary.uke?.fall?.t_s??null,
    first:rec.summary.uke?.fall?.first_part??null,
    yaw_peak:peakYaw,yaw_end:snaps.at(-1)!.yaw_deg,
    foot_left_lift:r(maxZ("left_foot")-original.poses[BODY_NAMES.indexOf("left_foot")]![2]),
    foot_right_lift:r(maxZ("right_foot")-original.poses[BODY_NAMES.indexOf("right_foot")]![2]),
    foot_left_move:footXY("left_foot"),foot_right_move:footXY("right_foot"),
    head_xy:snaps.at(-1)!.poses[BODY_NAMES.indexOf("head")]!.slice(0,2),
    right_contact:stopContacts.length>0, stop_first_t:stopContacts[0]?.t??null,
    stop_parts:stopParts, stop_peak_n:r(Math.max(0,...stopContacts.map(c=>c.force))),
    frame_count:snaps.length,
  };
  expect(rec.samples.length).toBeGreaterThan(100);
  expect(rec.samples.at(-1)!.qpos.every(Number.isFinite)).toBe(true);
  if(emitFrames)console.log("FULCRUM_FRAMES|"+sc.id+"|"+Buffer.from(JSON.stringify({summary,snaps})).toString("base64"));
  else console.log("FULCRUM_METRICS|"+JSON.stringify(summary));
  return {summary,rec};
}

describe("fulcro unilateral y tirón simétrico de hombros (MuJoCo real)",()=>{
  it("la suma del par esperado es cero para apoyos simétricos y cambia de signo al elegir pie",()=>{
    expect(expectedYawMoment(500,"none")).toBe(0);
    expect(expectedYawMoment(500,"right")).toBeLessThan(0);
    expect(expectedYawMoment(500,"left")).toBeGreaterThan(0);
    expect(expectedYawMoment(500,"right")).toBeCloseTo(-expectedYawMoment(500,"left"),9);
  });
  it("valida variantes, fija solo un tope físico y mantiene tirones paralelos",()=>{
    for(const blocked of ["none","right","left"]){
      for(const height of ["ankle","shin","knee","thigh"]){
        const sc=validateScenario(buildFulcrumScenario({blocked,height,level:"medium"}));
        expect(sc.props.length).toBe(blocked==="none"?0:1);
        expect(sc.effectors).toHaveLength(2);
        expect(sc.effectors[0]!.kind).toBe("prescribed_force");
        expect(sc.effectors[1]!.kind).toBe("prescribed_force");
        if(sc.effectors[0]!.kind==="prescribed_force"&&sc.effectors[1]!.kind==="prescribed_force"){
          expect(sc.effectors[0]!.force_n).toEqual(sc.effectors[1]!.force_n);
          expect(sc.effectors[0]!.force_n[1]).toBe(0);
          expect(sc.effectors[0]!.force_n[2]).toBe(0);
        }
        expect(buildScenarioModel(sc).mjcf).toContain("<mujoco");
      }
    }
  });
  it.each([
    {blocked:"none",height:"ankle",level:"medium"},
    {blocked:"right",height:"ankle",level:"low"},
    {blocked:"right",height:"ankle",level:"medium"},
    {blocked:"right",height:"ankle",level:"high"},
    {blocked:"right",height:"shin",level:"medium"},
    {blocked:"right",height:"knee",level:"medium"},
    {blocked:"right",height:"thigh",level:"medium"},
    {blocked:"left",height:"ankle",level:"medium"},
    {blocked:"right",height:"ankle",level:"medium",site:"sleeve"},
    {blocked:"right",height:"ankle",level:"medium",stance:"staggered_left"},
  ])("ejecuta ensayo físico %j",(async opts=>{
    await runOne(opts,true);
  }),90_000);
});


describe("kuzushi por fases, manga derecha y contacto transitorio (MuJoCo real)",()=>{
  const cases=[
    {name:"recto_sin_tope",mode:"straight",blocked:"none",bias:"balanced"},
    {name:"circular_sin_tope",mode:"arc",blocked:"none",bias:"balanced"},
    {name:"circular_tobillo_bilateral",mode:"arc",blocked:"right",bias:"balanced"},
    {name:"circular_tobillo_derecho",mode:"arc",blocked:"right",bias:"right"},
    {name:"circular_tobillo_derecho_fuerte",mode:"arc",blocked:"right",bias:"strong"},
    {name:"circular_tobillo_izquierdo",mode:"arc",blocked:"left",bias:"right"},
    {name:"circular_tobillo_derecho_retirada",mode:"arc",blocked:"right",bias:"right",support:"withdraw"},
    {name:"circular_espinilla_derecha",mode:"arc",blocked:"right",bias:"right",height:"shin"},
    {name:"circular_espinilla_simetrico",mode:"arc",blocked:"right",bias:"balanced",height:"shin"},
    {name:"circular_espinilla_derecha_mas_fuerte",mode:"arc",blocked:"right",bias:"strong",height:"shin"},
    {name:"circular_espinilla_derecha_prolongada",mode:"arc",blocked:"right",bias:"right",height:"shin",tempo:"extended"},
    {name:"recto_espinilla_derecha",mode:"straight",blocked:"right",bias:"right",height:"shin"},
    {name:"circular_espinilla_hombro_derecho",mode:"arc",blocked:"right",bias:"right",height:"shin",rightSite:"shoulder"},
    {name:"circular_espinilla_retirada",mode:"arc",blocked:"right",bias:"right",height:"shin",support:"withdraw"},
    {name:"circular_tobillo_derecho_largo",mode:"arc",blocked:"right",bias:"right",tempo:"extended"},
    {name:"recto_tobillo_derecho",mode:"straight",blocked:"right",bias:"right"},
    {name:"circular_tobillo_suave",mode:"arc",blocked:"right",bias:"right",level:"low"},
    {name:"circular_tobillo_fuerte",mode:"arc",blocked:"right",bias:"right",level:"high"},
  ] as const;
  it("las curvas realmente contienen 4 fases y el brazo derecho sigue después del contacto estimado",()=>{
    const s=validateScenario(buildCircularKuzushiScenario({mode:"arc",blocked:"right",bias:"right"}));
    expect(s.effectors).toHaveLength(2);
    expect(s.effectors.every(e=>e.kind==="pull")).toBe(true);
    if(s.effectors.every(e=>e.kind==="pull")){
      const right=s.effectors[1]!.keys;
      expect(right[2]!.force_n[2]).toBeGreaterThan(0);  // arriba
      expect(right[5]!.force_n[2]).toBe(0);              // adelante
      expect(right[7]!.force_n[2]).toBeLessThan(0);       // adelante y abajo
      expect(right.at(-1)!.t_s).toBeGreaterThan(2.5);
      expect(right[7]!.force_n[0]).toBeGreaterThan(s.effectors[0]!.keys[7]!.force_n[0]);
    }
    const recto=validateScenario(buildCircularKuzushiScenario({mode:"straight",blocked:"right",bias:"right"}));
    expect(recto.effectors.every(e=>e.kind==="pull"&&e.keys.every(k=>k.force_n[2]===0))).toBe(true);
    expect(buildCircularKuzushiScenario({blocked:"none"}).props).toHaveLength(0);
    const withdrawn=validateScenario(buildCircularKuzushiScenario({support:"withdraw"}));
    expect(withdrawn.obstacles).toHaveLength(1);
    expect(withdrawn.obstacles![0]!.phases.length).toBeGreaterThan(0);
  });
  it.each(cases)("evaluación física por fases: $name",async opts=>{
    const {name,...config}=opts;
    const sc=validateScenario(buildCircularKuzushiScenario(config));
    const built=buildScenarioModel(sc);
    const ids=BODY_NAMES.map(x=>built.bodies.indexOf(x));
    const snap:Snapshot[]=[];
    const chestFacing:Array<{t:number,frontUp:number}>=[];
    const rec=await runScenario(sc,{engineVersion:"circular-kuzushi-test",onSample:(_m,raw,index)=>{
      if(index%10!==0)return;
      const d=raw as {xpos:Float64Array;xmat:Float64Array};
      const pelvisId=ids[0]!;
      const chestId=ids[2]!;
      chestFacing.push({t:r(index/sc.record_hz),frontUp:r(d.xmat[9*chestId+6]!) });
      snap.push({t:r(index/sc.record_hz),
        poses:ids.map(id=>[r(d.xpos[3*id]!),r(d.xpos[3*id+1]!),r(d.xpos[3*id+2]!)]),
        yaw_deg:r(Math.atan2(d.xmat[9*pelvisId+3]!,d.xmat[9*pelvisId]!)*180/Math.PI)});
    }});
    const contact=rec.samples.flatMap(q=>q.contacts.filter(c=>c.body_a==="foot_stop"||c.body_b==="foot_stop")
      .map(c=>({t:r(q.t),body:c.body_a==="foot_stop"?c.body_b:c.body_a,
        fn:r(Math.hypot(...c.force_n))})));
    const contactFirst=contact[0]?.t??null, contactLast=contact.at(-1)?.t??null;
    const chestTime=rec.summary.uke?.fall?.contact_order?.find(c=>c.body==="chest")?.t_s;
    const chestFacingAtImpact=chestTime===undefined?null:chestFacing.reduce((p,c)=>
      Math.abs(c.t-chestTime)<Math.abs(p.t-chestTime)?c:p,chestFacing[0]!).frontUp;
    const minZ=(p:string)=>Math.min(...snap.map(x=>x.poses[BODY_NAMES.indexOf(p as typeof BODY_NAMES[number])]![2]));
    const maxZ=(p:string)=>Math.max(...snap.map(x=>x.poses[BODY_NAMES.indexOf(p as typeof BODY_NAMES[number])]![2]));
    const summary={
      name,id:sc.id,outcome:rec.summary.outcome,fall_t_s:rec.summary.uke?.fall?.t_s??null,
      direction_deg:rec.summary.direction_deg,
      first_part:rec.summary.uke?.fall?.first_part??null,
      head_impact_mps:rec.summary.uke?.fall?.impact.head?.speed_mps??null,
      body_contact_parts:Array.from(new Set(contact.map(x=>x.body))),
      first_contact_s:contactFirst,last_contact_s:contactLast,
      separated_before_end:contactLast!==null&&contactLast<sc.duration_s-.2,
      contact_before_fall:contactFirst!==null&&(rec.summary.uke?.fall?.t_s??Infinity)>contactFirst,
      torso_impact_mps:rec.summary.uke?.fall?.impact.chest?.speed_mps??null,
      chest_front_up_z_at_impact:chestFacingAtImpact,
      contact_order:rec.summary.uke?.fall?.contact_order?.slice(0,8)??[],
      peak_contact_n:r(Math.max(0,...contact.map(x=>x.fn))),
      left_foot_lift_m:r(maxZ("left_foot")-snap[0]!.poses[10]![2]),
      right_foot_lift_m:r(maxZ("right_foot")-snap[0]!.poses[13]![2]),
      min_head_z_m:r(minZ("head")),
      max_trunk_tilt_deg:r(rec.summary.max_tilt_deg),
      final_yaw_deg:snap.at(-1)!.yaw_deg,
      artifacts:rec.summary.artifacts?.map(a=>a.type)??[],
    };
    expect(rec.samples.length).toBeGreaterThan(150);
    expect(rec.samples.at(-1)!.qpos.every(Number.isFinite)).toBe(true);
    if(name==="circular_tobillo_derecho_retirada")expect(built.bodies).toContain("foot_stop");
    console.log("ARC_METRICS|"+JSON.stringify(summary));
    if(["recto_sin_tope","circular_sin_tope","circular_tobillo_derecho","circular_tobillo_derecho_retirada","circular_tobillo_derecho_fuerte","circular_espinilla_derecha","circular_espinilla_derecha_mas_fuerte","recto_espinilla_derecha","circular_espinilla_retirada"].includes(name)){
      console.log("ARC_FRAMES|"+name+"|"+Buffer.from(JSON.stringify({summary,snaps:snap})).toString("base64"));
    }
  },90_000);
});

/**
 * Readable slow-motion viewer source: actual MuJoCo body positions and ACTUAL
 * applied force vectors (recording.samples.effectors), never scripted arrows.
 * The diagnostic gzip is emitted only to test logs; not checked into recordings.
 */
describe("equilibrio previo, Uke firme y fuerzas reales",()=>{
  const series=[
    {name:"sin_fuerza",posture:"firm",begin:1.2,noForce:true},
    {name:"sin_fuerza_con_tope",posture:"firm",begin:1.2,noForce:true},
    {name:"sin_fuerza_tope_fijo",posture:"firm",begin:1.2,noForce:true},
    {name:"tense_08",posture:"tense",begin:.8,noForce:false},
    {name:"firm_08",posture:"firm",begin:.8,noForce:false},
    {name:"firm_12",posture:"firm",begin:1.2,noForce:false},
  ] as const;
  it.each(series)("mide retroceso sin traccion: $name",async ({name,posture,begin,noForce})=>{
    const scenario=buildCircularKuzushiScenario({
      blocked:"right",height:"shin",support:name==="sin_fuerza_tope_fijo"?"fixed":"withdraw",bias:"right",
      posture,begin,
    });
    if(noForce){
      scenario.effectors=[];
      scenario.duration_s=1.24;
      scenario.id="arc_no_force_firm_"+name;
      if(name==="sin_fuerza"){
        scenario.obstacles=[];
        scenario.props=[];
      }
    }
    const spec=validateScenario(scenario);
    const built=buildScenarioModel(spec);
    const ids=BODY_NAMES.map(x=>built.bodies.indexOf(x));
    const samples:Snapshot[]=[];
    const rate=spec.record_hz;
    const rec=await runScenario(spec,{engineVersion:"slow-motion-diagnostics",onSample:(_m,raw,index)=>{
      if(index%10!==0 && index!==0)return;
      const d=raw as {xpos:Float64Array;xmat:Float64Array};
      const pelvisId=ids[0]!;
      samples.push({t:r(index/rate),
        poses:ids.map(id=>[r(d.xpos[3*id]!),r(d.xpos[3*id+1]!),r(d.xpos[3*id+2]!)]),
        yaw_deg:r(Math.atan2(d.xmat[9*pelvisId+3]!,d.xmat[9*pelvisId]!)*180/Math.PI),
      });
    }});
    // Compare same time before the first nonzero force, and record the
    // unforced control trajectory to separate spontaneous motion.
    const pelvis=(s:Snapshot)=>s.poses[0]!;
    const origin=pelvis(samples[0]!);
    const pre=samples.filter(q=>q.t<Math.min(begin,1.15));
    const maxDrift=Math.max(...pre.map(q=>Math.hypot(
      pelvis(q)[0]-origin[0],pelvis(q)[1]-origin[1])));
    const postureAt0=samples[0]!.poses;
    const head=postureAt0[3]!, feet=[postureAt0[10]!,postureAt0[13]!];
    const forceData=samples.map(s=>{
      const closest=rec.samples.reduce((a,b)=>Math.abs(b.t-s.t)<Math.abs(a.t-s.t)?b:a,rec.samples[0]!);
      return closest.effectors.filter(e=>["kuzushi_left","kuzushi_right"].includes(e.id))
        .map(e=>({id:e.id,point:e.point_m.map(r),force:e.force_n.map(r)}));
    });
    const firstApplied=rec.samples.find(q=>q.effectors.some(e=>Math.hypot(...e.force_n)>1))?.t??null;
    const earlyStopContact=rec.samples.some(q=>q.t<begin&&q.contacts.some(c=>c.body_a==="foot_stop"||c.body_b==="foot_stop"));
    const metrics={name,posture,begin,stance:"neutral",early_stop_contact:earlyStopContact,
      pre_force_pelvis_drift_m:r(maxDrift),initial_pelvis:origin,
      initial_head_height_m:head[2],
      feet_center_z_m:r((feet[0][2]+feet[1][2])/2),
      first_applied_force_s:firstApplied,
      outcome:rec.summary.outcome,
      first_ground_part:rec.summary.uke?.fall?.first_part??null,
      torso_tilt_deg:r(rec.summary.max_tilt_deg),
    };
    expect(rec.samples.at(-1)!.qpos.every(Number.isFinite)).toBe(true);
    expect(forceData.length).toBe(samples.length);
    if(noForce)expect(firstApplied).toBe(null);
    else expect(firstApplied).not.toBe(null);
    console.log("SLOW_METRICS|"+JSON.stringify(metrics));
    const bundle={metrics,samples,forceData};
    console.log("SLOW_MUJOCO|"+name+"|"+gzipSync(Buffer.from(JSON.stringify(bundle))).toString("base64"));
    // Compact independent rendering stream: 0.16 s frames, world positions
    // in signed centimetres, actual applied forces in signed 5-N units.
    if(name==="firm_08"||name==="tense_08"||name==="sin_fuerza_tope_fijo") {
      const packed:number[]=[];
      const put=(v:number)=>{const n=Math.round(v);packed.push(n&255,(n>>8)&255);};
      for(let i=0;i<samples.length;i+=2){
        for(const p of samples[i]!.poses)for(const c of p)put(c*100);
        for(const effectorId of ["kuzushi_left","kuzushi_right"]){
          const eff=forceData[i]!.find(e=>e.id===effectorId);
          for(const val of (eff?.point??[0,0,0]))put(val*100);
          for(const val of (eff?.force??[0,0,0]))put(val/5);
        }
      }
      console.log("SLOW_PACK|"+name+"|"+samples.length+"|"+
        Buffer.from(packed).toString("base64"));
    }
  },120_000);
});
