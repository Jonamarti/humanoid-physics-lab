/* global TextEncoder, btoa */
/**
 * Isolated one-sleeve Tai-Otoshi experiment.
 * Uke begins upright facing +X. Right = -Y, left = +Y.
 * One pull on right_sleeve_elbow; a static diagonal capsule in front of
 * the right shin/foot, optionally mirrored to the left leg for a control.
 * Zero modifications to the existing physics engine or recorded animations.
 */
const rounded=(n)=>Number(n.toFixed(4));
const v=(x,y,z)=>[rounded(x),rounded(y),rounded(z)];
const INTENSITY={soft:230,medium:420,strong:650};
const PHASES={
  brief:[[0,0,0],[.12,1,0],[.34,1,0],[.55,0,0]],
  sustained:[[0,0,0],[.18,.65,.18],[.48,1,.18],[1.1,1,0],[1.65,.83,0],[2.05,0,0]],
  redirect:[[0,0,0],[.18,.65,.4],[.48,1,.38],[.85,1,0],[1.15,.94,-.35],[1.5,.85,-.75],[1.85,.5,-.88],[2.2,0,0]],
};
export const ONE_SLEEVE_VARIANTS=Object.keys(PHASES);
export function buildOneSleeveScenario({mode="brief",bar="right",force="medium",posture="firm",begin=.65,lean="none",angle="diagonal",barX=.55}={}) {
  if(!(mode in PHASES))throw Error("Perfil desconocido.");
  if(!["right","both","none"].includes(bar))throw Error("Barra desconocida.");
  if(!(force in INTENSITY))throw Error("Fuerza desconocida.");
  if(!["firm","tense"].includes(posture))throw Error("Tono desconocido.");
  if(!["none","left"].includes(lean))throw Error("Sesgo lateral desconocido.");
  if(!["diagonal","straight"].includes(angle))throw Error("Orientación de barra desconocida.");
  if(!Number.isFinite(begin)||begin<.4||begin>1.3)throw Error("Inicio fuera de intervalo.");
  if(!Number.isFinite(barX)||barX<.04||barX>3)throw Error("Posición anterior de barra fuera de 4-300 cm.");
  const f=INTENSITY[force];
  const keys=PHASES[mode].map(([t,x,z])=>({
    t_s:rounded(begin+t),force_n:v(f*x,lean==="left"?f*x*.22:0,f*z),
  }));
  // The bar's two ends are 3D points, so this is a TRUE diagonal capsule,
  // not an axis-aligned box drawn obliquely. The right foot can rise/pivot
  // over it; contact is naturally lost when it clears the bar.
  // +X anterior to shin: no initial interpenetration. Compare to no-bar
  // and no-force controls to catch a bad settling contact.
  const barAt=side=>{
    const sign=side==="right"?-1:1;
    const from=angle==="diagonal"?v(barX,sign*.30,.13):v(barX+.02,sign*.29,.17);
    const to=angle==="diagonal"?v(barX+.04,sign*.055,.32):v(barX+.02,sign*.055,.17);
    return {id:side+"_diagonal_bar",kind:"fixed_bar",from_m:from,to_m:to,radius_m:.040};
  };
  return {
    schema:"hpl.scenario.v1",
    id:`one_sleeve_${mode}_${bar}_${force}_${posture}_${lean}_${angle}_${String(begin).replace(".","p")}_x${String(barX).replace(".","p")}`,
    duration_s:rounded(Math.max(3.4,keys.at(-1).t_s+1)),
    record_hz:125,ground:{friction:1.0},
    actors:[{id:"uke",kind:"humanoid",mass_kg:75,height_m:1.75,
      strength:posture==="firm"?1.5:1,waist_stiffness:1,attitude:"tense",stance:"neutral"}],
    props:bar==="none"?[]:bar==="both"?[barAt("right"),barAt("left")]:[barAt("right")],
    effectors:[{id:"single_right_sleeve",kind:"pull",actor:"uke",
      site:"right_sleeve_elbow",frame:"actor_initial",
      keys,max_force_n:900,max_power_w:1250}],
  };
}
export function oneSleeveHash(s){
  let bin="";
  for(const b of new TextEncoder().encode(JSON.stringify(s)))bin+=String.fromCharCode(b);
  return "#scn="+btoa(bin).replace(/\+/g,"-").replace(/\//g,"_").replace(/=+$/,"");
}
