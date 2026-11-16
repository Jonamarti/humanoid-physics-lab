/** Export existing public Tori targets; no new attack choreography. */
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { resolve, dirname } from "node:path";
import { resolveToriBodyParams, toriSkeleton, buildToriBodyMjcf, TORI_BODY_DEFAULTS } from "../../src/effectors/toriBody";
const input=resolve(process.argv[2]!);const output=resolve(process.argv[3]!);
const source=JSON.parse(readFileSync(input,"utf8").replace(/^\uFEFF/,""));
const body=source.tori.body;
const params=resolveToriBodyParams({mass_kg:source.tori.mass_kg,height_m:body.height_m,keys:body.keys,interp:body.interp,friction:body.friction});
const hz=960;const count=Math.ceil(source.duration_s*hz)+2;
const targets=Array.from({length:count},(_,k)=> {
 const pose=toriSkeleton(params,k/hz);
 return params.segments.map(s=>[pose[s.segment].a,pose[s.segment].b]);
});
let interpolationError=0;
for (let k=0;k<Math.round(source.duration_s*960);k++) {
 const x=k*hz/960;const i=Math.floor(x);const a=x-i;const direct=toriSkeleton(params,k/960);
 for (let j=0;j<params.segments.length;j++) {
  const ends=direct[params.segments[j]!.segment];
  for (let end=0;end<2;end++) {
   const target=end===0?ends.a:ends.b;
   const error=Math.hypot(...target.map((v,c)=>v-((1-a)*targets[i]![j]![end]![c]!+a*targets[i+1]![j]![end]![c]!)));
   interpolationError=Math.max(interpolationError,error);
  }
 }
}
mkdirSync(dirname(output),{recursive:true});
writeFileSync(output,JSON.stringify({source,params,hz,targets,mjcf:buildToriBodyMjcf(params),defaults:TORI_BODY_DEFAULTS,target_interpolation_max_error_m:interpolationError}));
console.log("EXPORTED",source.id,params.segments.length,"finite-mass segments",count,"target frames; max interpolation error",interpolationError);
