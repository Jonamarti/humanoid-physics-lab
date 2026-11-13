import {gzipSync} from "node:zlib";
import {describe,it,expect} from "vitest";
import {buildScenarioModel} from "../../src/scenario/buildModel";
import {runScenario} from "../../src/scenario/run";
import {validateScenario} from "../../src/scenario/validate";
import {ACTOR_BODIES} from "../../src/scenario/schema";
// @ts-expect-error Public bridge is intentionally JavaScript.
import {buildOneSleeveScenario} from "../../public/one-sleeve-lab.js";

/**
 * Temporary read-only visualization export for a video (no recording writes).
 * Positions + chest direction + actually applied vector, not a motion script.
 */
const cases=[
 {tag:"barra_breve_420",mode:"brief",bar:"right",force:"medium"},
 {tag:"sin_barra_breve_420",mode:"brief",bar:"none",force:"medium"},
 {tag:"barra_prolongada_230",mode:"sustained",bar:"right",force:"soft"},
] as const;
const round=(x:number)=>Number(x.toFixed(3));
const names=ACTOR_BODIES.humanoid;
describe("visualizacion reproducible manga unica",()=>{
 it.each(cases)("exporta centros de cuerpos y fuerza realmente aplicada: $tag",async ({tag,...opts})=>{
   const sc=validateScenario(buildOneSleeveScenario(opts));
   const built=buildScenarioModel(sc);
   const bodyIds=names.map(name=>built.bodies.indexOf(name));
   const chestId=built.bodies.indexOf("chest");
   const samples:Array<{t:number;p:number[][];face:number[]}>=[];
   const rec=await runScenario(sc,{engineVersion:"single-sleeve-video-ci",
    onSample:(_model,raw,index)=>{
      if(index%5!==0)return;
      const d=raw as {xpos:Float64Array;xmat:Float64Array};
      samples.push({
        t:round(index/sc.record_hz),
        p:bodyIds.map(id=>[round(d.xpos[id*3]!),round(d.xpos[id*3+1]!),round(d.xpos[id*3+2]!)]),
        face:[round(d.xmat[9*chestId]!),round(d.xmat[9*chestId+3]!),round(d.xmat[9*chestId+6]!)],
      });
    }
   });
   const frames=samples.map((s)=>{
     const at=rec.samples.reduce((a,b)=>Math.abs(b.t-s.t)<Math.abs(a.t-s.t)?b:a,rec.samples[0]!);
     const applied=at.effectors.find(f=>f.id==="single_right_sleeve");
     const contacts=at.contacts.filter(c=>c.body_a.includes("diagonal_bar")||c.body_b.includes("diagonal_bar"));
     return [...s.p.flatMap(p=>p),...s.face,
        ...(applied?.point_m??[0,0,0]).map(round),
        ...(applied?.force_n??[0,0,0]).map(round),
        round(contacts.reduce((m,c)=>Math.max(m,Math.hypot(...c.force_n)),0))];
   });
   const first=rec.samples.find(s=>s.contacts.some(c=>c.body_a.includes("diagonal_bar")||c.body_b.includes("diagonal_bar")));
   const metadata={
     case:tag, names,dt:.04,frameCount:frames.length,
     props:sc.props,forceOnset:.65,
     firstContact:first?.t??null,
     fall:rec.summary.uke?.fall?.t_s??null,
     firstGround:rec.summary.uke?.fall?.first_part??null,
   };
   // Float arrays JSON -> gzip -> base64 in CI logs; no binary upload is required.
   console.log("VIDEO_SINGLE_RIGHT|"+tag+"|"+gzipSync(Buffer.from(JSON.stringify({metadata,frames}))).toString("base64"));
   // A second lossily quantized 8-bit stream to reconstruct a readable video
   // without depending on massive CI log attachments. Each frame contains
   // the 14 body centres, actual sleeve point and actual 3D force vector.
   const quantizedFrames=frames.filter((_,i)=>i%2===0);
   const min=[-1.0,-2.0,-.5], span=[5.0,4.0,3.0];
   const bytes:number[]=[];
   for(const row of quantizedFrames){
     for(let j=0;j<45;j++){
       const coord=j%3;
       bytes.push(Math.max(0,Math.min(255,Math.round((row[j]!-min[coord]!)/span[coord]!*255))));
     }
     for(let j=45;j<48;j++){
       const coord=(j-45)%3;
       bytes.push(Math.max(0,Math.min(255,Math.round((row[j]!-min[coord]!)/span[coord]!*255))));
     }
     // One force is on the manga, actual force from MuJoCo; max display +-1000 N.
     for(let j=48;j<51;j++)bytes.push(Math.max(0,Math.min(255,Math.round((row[j]!+1000)/2000*255))));
     bytes.push(Math.max(0,Math.min(255,Math.round(row[51]!/20))));
   }
   console.log("VIDEO_COMPACT|"+tag+"|"+JSON.stringify({
     metadata:{...metadata,dt:.08,frameCount:quantizedFrames.length},
     data:Buffer.from(bytes).toString("base64"),
   }));

   expect(frames.length).toBeGreaterThan(70);
   expect(frames.every(f=>f.every(Number.isFinite))).toBe(true);
   expect(built.bodies.indexOf("pelvis")).toBe(1);
 },90000);
});
