/* global TextEncoder, btoa */
/**
 * A/B foot-fulcrum physical trial. The existing HPL runScenario() remains untouched.
 * Symmetric forward shoulder/sleeve pulls, optional unilateral front stop.
 * Coordinates: Uke faces +X; +Y left; right foot is at -Y.
 */
const R=(n,d=4)=>Number(n.toFixed(d));
const vec=(x,y,z)=>[R(x),R(y),R(z)];
const FORCE_LEVELS=Object.freeze({low:260,medium:650,high:1200});
const BLOCK_LEVELS=Object.freeze({ankle:0.13,shin:0.33,knee:0.55,thigh:0.78});
export const FULCRUM_LEVELS=FORCE_LEVELS;
export const FULCRUM_HEIGHTS=BLOCK_LEVELS;
export const FULCRUM_SIDES=Object.freeze(["none","left","right"]);

/** Generates a new, disposable scenario; no recorded pose is driven by the source. */
export function buildFulcrumScenario({
  level="medium", height="ankle", blocked="right", site="shoulder", stance="neutral",
  forceN, heightM,
}={}) {
  if(!Object.hasOwn(FORCE_LEVELS,level))throw new Error("Intensidad desconocida.");
  if(!Object.hasOwn(BLOCK_LEVELS,height))throw new Error("Altura desconocida.");
  if(!FULCRUM_SIDES.includes(blocked))throw new Error("Lado desconocido.");
  if(!["shoulder","sleeve"].includes(site))throw new Error("Punto de tracción desconocido.");
  if(!["neutral","staggered_left","staggered_right"].includes(stance))throw new Error("Postura desconocida.");
  const force=forceN??FORCE_LEVELS[level], h=heightM??BLOCK_LEVELS[height];
  if(!Number.isFinite(force)||force<=0||force>4000)throw new Error("Fuerza fuera de 0–4000 N por brazo.");
  if(!Number.isFinite(h)||h<.08||h>1.0)throw new Error("Altura fuera de 8–100 cm.");
  // Limb-specific pad in front of the selected foot, so only that foot/shin
  // can contact it; Uke's opposite leg can pass freely at the other Y.
  // X coordinate is approximate for neutral stance. In the staggered stance
  // the rear foot is farther back and this fixed block must be repositioned.
  const sign=blocked==="left"?1:-1;
  // A narrow, *raised* strike zone at the selected height; a knee-level stop
  // must not also block the foot just because it occupies the whole column below.
  // X positions leave a clearance at t=0 for the 1.75m neutral Uke geometry.
  const barrier={ankle:{x:.242,depth:.065,band:.12},shin:{x:.14,depth:.07,band:.12},
    knee:{x:.15,depth:.07,band:.12},thigh:{x:.16,depth:.09,band:.12}}[height];
  const shift=stance==="staggered_left"&&blocked==="right" ? -.13 :
    stance==="staggered_right"&&blocked==="left" ? -.13 : 0;
  const block=blocked==="none"?[]:[{
    id:"foot_stop",kind:"fixed_box",
    size_m:vec(barrier.depth,.13,barrier.band),
    pos_m:vec(barrier.x+shift,sign*.105,h),
  }];
  const siteName=(s)=>site==="shoulder"?s+"_shoulder_impact":s+"_sleeve_elbow";
  const eff=["left","right"].map((s)=>({
    id:"forward_"+s,kind:"prescribed_force",actor:"uke",
    site:siteName(s),frame:"actor_initial",force_n:vec(force,0,0),
    start_s:.30,duration_s:.42,waveform:"half_sine",
  }));
  const key=`fulcrum_${blocked}_${height}_${level}_${site}_${stance}`;
  return {
    schema:"hpl.scenario.v1",id:key,
    duration_s:3.0,record_hz:50,ground:{friction:1.1},
    actors:[{id:"uke",kind:"humanoid",mass_kg:75,height_m:1.75,strength:1,
      waist_stiffness:1,attitude:"tense",stance}],
    props:block,effectors:eff,
  };
}

export function fulcrumScenarioHash(scenario) {
  let bits="";
  for(const b of new TextEncoder().encode(JSON.stringify(scenario)))bits+=String.fromCharCode(b);
  return "#scn="+btoa(bits).replace(/\+/g,"-").replace(/\//g,"_").replace(/=+$/,"");
}
/** Expected external torque about a right-foot fulcrum, before any reaction/contact correction. */
export function expectedYawMoment(forcePerShoulderN,side="right",halfShoulderWidthM=.225,footHalfWidthM=.105) {
  if(side==="none")return 0;
  const yp=side==="right"?-footHalfWidthM:footHalfWidthM;
  // sum of (r_y * -F_x) of two parallel symmetric pulls, in N*m.
  return -((halfShoulderWidthM-yp)+(-halfShoulderWidthM-yp))*forcePerShoulderN;
}
