import {describe,it,expect} from "vitest";
import {buildScenarioModel} from "../../src/scenario/buildModel";
import {validateScenario} from "../../src/scenario/validate";
import {runScenario} from "../../src/scenario/run";
// @ts-expect-error Public browser bridge intentionally plain JS.
import {buildOneSleeveScenario} from "../../public/one-sleeve-lab.js";

const scenarios=[
  {name:"right_brief", mode:"brief",bar:"right"},
  {name:"no_bar_brief", mode:"brief",bar:"none"},
  {name:"right_sustained", mode:"sustained",bar:"right"},
  {name:"right_redirected", mode:"redirect",bar:"right"},
  {name:"both_brief", mode:"brief",bar:"both"},
] as const;

describe("manga derecha aislada: barra diagonal y torque sobre Uke",()=>{
  it("genera barra diagonal real y solo una tracción",()=>{
    const s=validateScenario(buildOneSleeveScenario({mode:"brief"}));
    const built=buildScenarioModel(s);
    expect(s.actors[0]!.kind).toBe("humanoid");
    if (s.actors[0]!.kind === "humanoid") expect(s.actors[0]!.stance).toBe("neutral");
    expect(s.effectors).toHaveLength(1);
    expect(s.effectors[0]!.site).toBe("right_sleeve_elbow");
    expect(s.props).toHaveLength(1);
    expect(s.props[0]!.kind).toBe("fixed_bar");
    expect(built.mjcf).toContain('type="capsule"');
    expect(built.mjcf).toContain('name="right_diagonal_bar_geom"');
    expect(built.mjcf).toContain('fromto="0.3 -0.3 0.13 0.34 -0.055 0.32"');
    expect(validateScenario(buildOneSleeveScenario({bar:"none"})).props).toHaveLength(0);
    expect(validateScenario(buildOneSleeveScenario({bar:"both"})).props).toHaveLength(2);
    expect(()=>validateScenario({...s,props:[{id:"bad",kind:"fixed_bar",from_m:[0,0,0],to_m:[0,0,0],radius_m:.03}]})).toThrow();
  });
  it.each(scenarios)("ejecuta MuJoCo real y mide rotación, contacto y caída: $name",async o=>{
    const s=validateScenario(buildOneSleeveScenario(o));
    const built=buildScenarioModel(s);
    const chestId=built.bodies.indexOf("chest");
    const pelvisId=built.bodies.indexOf("pelvis");
    const rightFootId=built.bodies.indexOf("right_foot");
    const headId=built.bodies.indexOf("head");
    const origin:Array<number>=[];
    const motion:Array<{t:number,angle:number,faceX:number,faceY:number,headZ:number,pelvisX:number,rightFootZ:number}>=[];
    const rec=await runScenario(s,{engineVersion:"one-sleeve-diagnostic",onSample:(_m,raw,index)=>{
      if(index%10!==0)return;
      const d=raw as {xpos:Float64Array;xmat:Float64Array};
      const m=9*chestId,p=3*pelvisId;
      const row={
        t:Number((index/s.record_hz).toFixed(3)),
        angle:Number((Math.atan2(d.xmat[m+3]!,d.xmat[m]!)*180/Math.PI).toFixed(2)),
        faceX:Number(d.xmat[m]!.toFixed(3)),
        faceY:Number(d.xmat[m+3]!.toFixed(3)),
        headZ:Number(d.xpos[3*headId+2]!.toFixed(3)),
        pelvisX:Number(d.xpos[p]!.toFixed(3)),
        rightFootZ:Number(d.xpos[3*rightFootId+2]!.toFixed(3)),
      };
      if(!origin.length)origin.push(row.pelvisX,row.rightFootZ);
      motion.push(row);
    }});
    const barContacts=rec.samples.flatMap(x=>x.contacts.filter(c=>/diagonal_bar/.test(c.body_a+c.body_b))
      .map(c=>({t:Number(x.t.toFixed(3)),other:/diagonal_bar/.test(c.body_a)?c.body_b:c.body_a,
        n:Number(Math.hypot(...c.force_n).toFixed(1))})));
    const pre=motion.filter(f=>f.t<.65);
    const drift=Math.max(...pre.map(f=>Math.abs(f.pelvisX-origin[0]!)));
    const nearImpact=motion.filter(f=>f.t<=1.65);
    const peakYaw=Math.max(...nearImpact.map(f=>Math.abs(f.angle)));
    const fall=rec.summary.uke?.fall;
    const out={
      name:o.name,kind:"one_right_sleeve_only",peakYawDeg:Number(peakYaw.toFixed(2)),
      faceYAt1s:motion.reduce((a,b)=>Math.abs(b.t-1)<Math.abs(a.t-1)?b:a).faceY,
      first_contact:barContacts[0]??null,
      contact_parts:[...new Set(barContacts.map(c=>c.other))],
      last_contact:barContacts.at(-1)?.t??null,
      pre_pull_drift_m:Number(drift.toFixed(3)),
      fall_time_s:fall?.t_s??null,first_ground_part:fall?.first_part??null,
      head_ground_speed_mps:fall?.impact.head?.speed_mps??null,
      status:rec.summary.outcome,
      endAngle:motion.at(-1)!.angle,
    };
    expect(rec.samples.at(-1)!.qpos.every(Number.isFinite)).toBe(true);
    expect(pre).not.toHaveLength(0);
    console.log("SINGLE_SLEEVE_METRICS|"+JSON.stringify(out));
    console.log("SINGLE_SLEEVE_FRAMES|"+o.name+"|"+Buffer.from(JSON.stringify({out,motion})).toString("base64"));
  },90000);
  it("Uke queda vertical sin manga ni barra en el control inicial",async()=>{
    const s=buildOneSleeveScenario({bar:"none"});
    s.effectors=[];
    const rec=await runScenario(validateScenario(s),{engineVersion:"one-sleeve-stability-control"});
    expect(rec.summary.outcome).not.toBe("fall");
  },90000);
});