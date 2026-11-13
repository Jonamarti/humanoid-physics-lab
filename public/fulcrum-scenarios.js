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
    knee:{x:.31,depth:.07,band:.12},thigh:{x:.16,depth:.09,band:.12}}[height];
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


/**
 * Segunda generación: kuzushi por fases + contacto unilateral TEMPORAL.
 *
 * El apoyo NO es una junta fija del pie. Es una superficie de colisión: Uke
 * puede deslizarse, despegar, pasar por encima o dejar de tocarla en cualquier
 * instante. En 'withdraw' la superficie (pierna idealizada de Tori) se retira
 * siguiendo una trayectoria temporal; no espera a detectar el impacto.
 *
 * El tirón es una fuerza real sobre dos puntos del muñeco, no una animación
 * prescrita de Uke. La geometría inicial queda sin alterar.
 */
export const CIRCULAR_TEMPOS=Object.freeze({long:1,extended:1.34});
export const CIRCULAR_BIASES=Object.freeze({balanced:1,right:1.45,strong:1.85});
export const CIRCULAR_MODES=Object.freeze(["arc","straight"]);
export const CIRCULAR_SUPPORTS=Object.freeze(["fixed","withdraw"]);
const CIRCULAR_SCALE=Object.freeze({low:.65,medium:1,high:1.4});
const ARC_KNOTS=[
  // [tiempo desde 0.30 s, delante X, vertical Z] relativos a Fbase;
  // up -> forward -> forward/down -> down/deceleration.
  [0,0,0],
  [.14,.34,.46],
  [.34,.72,.70],
  [.52,.92,.52],
  [.72,1.02,.25],
  [.95,1.04,0],
  [1.25,1.10,-.35],
  [1.52,1.08,-.65],
  [1.80,.85,-.95],
  [2.04,.51,-.72],
  [2.30,0,0],
];
const lerpClamp=(v,a,b)=>Math.min(1,Math.max(0,(v-a)/(b-a)));
const smoothStep=u=>u*u*(3-2*u);
function arcKeys(level,tempo,mode,bias,side,begin=.30) {
  const base=410*CIRCULAR_SCALE[level];
  const durationScale=CIRCULAR_TEMPOS[tempo];
  const holdLeft=bias!=="balanced";
  return ARC_KNOTS.map(([offset,x,z])=>{
    const t=begin+offset*durationScale;
    const lateralRamp=smoothStep(lerpClamp(offset,.72,1.60));
    let scale=1;
    if(side==="right"&&bias!=="balanced")scale+=((CIRCULAR_BIASES[bias]-1)*lateralRamp);
    if(side==="left"&&holdLeft)scale*=1-(bias==="strong"?.65:.38)*lateralRamp;
    // Straight control uses exactly the same duration and Fx progression
    // as arc, but zero vertical component (controls force direction).
    return {t_s:R(t),force_n:vec(base*x*scale,0,
      mode==="arc"?base*z*scale:0)};
  });
}
function stationaryStop(blocked,height,stance) {
  const sign=blocked==="left"?1:-1;
  const h=BLOCK_LEVELS[height];
  const barrier={ankle:{x:.242,depth:.065,band:.12},
    shin:{x:.32,depth:.07,band:.12},
    knee:{x:.15,depth:.07,band:.12},
    thigh:{x:.33,depth:.09,band:.12}}[height];
  const shift=stance==="staggered_left"&&blocked==="right"?-.13:
    stance==="staggered_right"&&blocked==="left"?-.13:0;
  return {
    x:R(barrier.x+shift),y:R(sign*.105),z:R(h),
    sx:R(barrier.depth),sy:.13,sz:R(barrier.band),
  };
}
export function buildCircularKuzushiScenario({
  level="medium",height="ankle",blocked="right",stance="neutral",
  tempo="long",bias="right",mode="arc",support="withdraw",
  rightSite="sleeve",posture="tense",begin=.30,
}={}) {
  if(!Object.hasOwn(CIRCULAR_SCALE,level))throw new Error("Intensidad desconocida.");
  if(!Object.hasOwn(BLOCK_LEVELS,height))throw new Error("Altura desconocida.");
  if(!FULCRUM_SIDES.includes(blocked))throw new Error("Pierna desconocida.");
  if(!["neutral","staggered_left","staggered_right"].includes(stance))throw new Error("Postura desconocida.");
  if(!Object.hasOwn(CIRCULAR_TEMPOS,tempo))throw new Error("Duración desconocida.");
  if(!Object.hasOwn(CIRCULAR_BIASES,bias))throw new Error("Sesgo de manga desconocido.");
  if(!CIRCULAR_MODES.includes(mode))throw new Error("Dirección de tracción desconocida.");
  if(!CIRCULAR_SUPPORTS.includes(support))throw new Error("Apoyo desconocido.");
  if(!["sleeve","shoulder"].includes(rightSite))throw new Error("Sitio de manga desconocido.");
  if(!["tense","firm"].includes(posture))throw new Error("Tono desconocido.");
  if(!Number.isFinite(begin)||begin<.3||begin>1.5)throw new Error("Inicio fuera de 0,30-1,50 s.");
  const tEnd=R(begin+2.30*CIRCULAR_TEMPOS[tempo]);
  const scenario={
    schema:"hpl.scenario.v1",
    id:`arc_${blocked}_${height}_${level}_${tempo}_${bias}_${mode}_${support}_${rightSite}_${stance}_${posture}_${String(R(begin)).replace(".","p")}`,
    duration_s:R(Math.max(3.8,tEnd+1.0)),
    record_hz:125,
    ground:{friction:1.1},
    actors:[{id:"uke",kind:"humanoid",mass_kg:75,height_m:1.75,
      strength:posture==="firm"?1.5:1,waist_stiffness:1,attitude:"tense",stance}],
    props:[],effectors:[
      {id:"kuzushi_left",kind:"pull",actor:"uke",site:"left_shoulder_impact",
        frame:"actor_initial",keys:arcKeys(level,tempo,mode,bias,"left",begin),
        max_force_n:1800,max_power_w:1600},
      {id:"kuzushi_right",kind:"pull",actor:"uke",
        site:rightSite==="sleeve"?"right_sleeve_elbow":"right_shoulder_impact",
        frame:"actor_initial",keys:arcKeys(level,tempo,mode,bias,"right",begin),
        max_force_n:1800,max_power_w:1600},
    ],
  };
  if(blocked!=="none"){
    const p=stationaryStop(blocked,height,stance);
    if(support==="fixed"){
      scenario.props=[{id:"foot_stop",kind:"fixed_box",
        size_m:vec(p.sx,2*p.sy,2*p.sz),pos_m:vec(p.x,p.y,p.z)}];
    }else{
      // Spring-controlled pad: mimics Tori withdrawing the blocking leg.
      // Disengagement is deliberately time-keyed and NOT contingent on contact.
      const start=begin+1.16*CIRCULAR_TEMPOS[tempo];
      const dy=blocked==="right"?-.55:.55;
      scenario.obstacles=[{
        id:"foot_stop",shape:"pad",tori_mass_kg:75,
        // Narrow lateral pad, deliberately moved 6 cm toward the selected leg:
        // avoid sweeping both feet while keeping Uke free to rotate over it.
        start_m:vec(p.x,p.y+(blocked==="right"?-.06:.06),p.z),
        size_m:vec(p.sx,.15,2*p.sz),
        friction:1.2,surface:"cloth",mass_kg:4,
        max_force_n:4500,max_vertical_force_n:4500,
        kp_n_per_m:15000,
        phases:[{start_s:R(start),duration_s:.5,offset_m:vec(0,dy,0)}],
      }];
    }
  }
  return scenario;
}
