import { describe, expect, it } from "vitest";
import { buildScenarioModel } from "../../src/scenario/buildModel";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
// @ts-expect-error Browser-only public JS intentionally has no TS declarations.
import { buildFulcrumScenario, expectedYawMoment } from "../../public/fulcrum-scenarios.js";

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
