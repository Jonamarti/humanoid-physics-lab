/* global TextEncoder, btoa */
/**
 * Read-only bridge from AnimSpec's 23-point trajectories to EXISTING HPL MuJoCo scenarios.
 * Does not change Uke's recorded qpos or force any angular motion. The HPL solver computes it.
 * The adapter is approximate: HPL's parametrized humanoid != AnimSpec's Mixamo retargeting.
 * Pure JS served from public/, so it works both in Vite dev and GitHub Pages.
 */
const DEG = 180 / Math.PI;
const round = (v, digits = 3) => Number(v.toFixed(digits));
const add = (a, b) => a.map((x, i) => x + b[i]);
const sub = (a, b) => a.map((x, i) => x - b[i]);
const mul = (a, f) => a.map(x => x * f);
const dot = (a, b) => a.reduce((v, x, i) => v + x * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (a, fallback) => { const d = Math.hypot(...a); return d > 1e-7 ? mul(a, 1 / d) : fallback; };
const avg = (a, b) => mul(add(a, b), .5);
const vround = a => a.map(x => round(x));

function inputFrames(frames) {
  if (!Array.isArray(frames) || frames.length < 2) throw new Error("Se necesitan dos o más fotogramas.");
  if (frames.length > 12000) throw new Error("Demasiados fotogramas.");
  let prev = -Infinity;
  for (const f of frames) {
    if (!Number.isFinite(f.t) || f.t <= prev || !Array.isArray(f.tori) || f.tori.length !== 23 ||
        f.tori.some(p => !Array.isArray(p) || p.length !== 3 || p.some(n => !Number.isFinite(n)))) {
      throw new Error("Fotogramas de Tori incompletos o tiempos incorrectos.");
    }
    prev = f.t;
  }
  const duration = frames.at(-1).t - frames[0].t;
  if (duration <= 0 || duration > 30) throw new Error("La duración física debe estar entre 0 y 30 s.");
}

function sampleIndices(frames, interval = .17) {
  const indices = [0];
  let lastT = frames[0].t;
  for (let i = 1; i < frames.length - 1; i++) {
    if (frames[i].t - lastT >= interval - 1e-8) {
      indices.push(i);
      lastT = frames[i].t;
    }
  }
  indices.push(frames.length - 1);
  return indices;
}

function poseFromTori(points, t, previousYaw) {
  const pel = avg(points[8], points[9]);
  const hipsLeft = unit(sub(points[8], points[9]), [0, -1, 0]);
  let yaw = Math.atan2(-hipsLeft[0], hipsLeft[1]);
  if (previousYaw !== undefined) {
    while (yaw - previousYaw > Math.PI) yaw -= 2 * Math.PI;
    while (yaw - previousYaw < -Math.PI) yaw += 2 * Math.PI;
  }
  const fwd = [Math.cos(yaw), Math.sin(yaw), 0];
  const left = [-Math.sin(yaw), Math.cos(yaw), 0];
  const up = unit(sub(points[21], pel), [0, 0, 1]);
  const lean = Math.atan2(dot(up, fwd), up[2]) * DEG;
  const sideLean = Math.atan2(dot(up, left), up[2]) * DEG;
  // HPL's shoulder midpoint is 0.9 * 0.6m above its pelvis for a 1.75m human.
  const shoulderMid = add(pel, mul(up, .54 * 1.75 / 1.76));
  const chestFwd = unit(cross(left, up), fwd);
  const chestLeft = unit(cross(up, chestFwd), left);
  const handLocal = wrist => {
    const delta = sub(wrist, shoulderMid);
    return vround([dot(delta, chestFwd), dot(delta, chestLeft), dot(delta, up)]);
  };
  const foot = ankle => vround([ankle[0], ankle[1], Math.max(0, ankle[2] - .09)]);
  return {
    t_s: round(t, 4), pelvis_m: vround(pel), yaw_deg: round(yaw * DEG, 2),
    lean_deg: round(lean, 2), side_lean_deg: round(sideLean, 2),
    left_foot_m: foot(points[4]), right_foot_m: foot(points[5]),
    left_hand_m: handLocal(points[14]), right_hand_m: handLocal(points[15]),
  };
}

function toriKeys(frames) {
  let yaw;
  const t0 = frames[0].t;
  return sampleIndices(frames).map(i => {
    const key = poseFromTori(frames[i].tori, frames[i].t - t0, yaw);
    yaw = key.yaw_deg / DEG;
    return key;
  });
}

function segmentCenter(points, support) {
  if (support === "leg") return avg(points[7], points[5]); // right shin, not an exact sweep surface
  if (support === "back") return avg(points[20], points[21]); // approximate torso
  return avg(points[8], points[9]); // hips
}

function supportObstacle(frames, support) {
  const idx = sampleIndices(frames, .21);
  const centers = idx.map(i => segmentCenter(frames[i].tori, support));
  const start = centers[0];
  let end = 0;
  const t0 = frames[0].t;
  const phases = [];
  for (let j = 1; j < idx.length; j++) {
    const t = frames[idx[j]].t - t0;
    const dt = t - end;
    if (dt <= 1e-8) continue;
    phases.push({start_s: round(end, 4), duration_s: round(dt, 4), offset_m: vround(sub(centers[j], start))});
    end = t;
  }
  const config = {
    leg: {shape:"bar", size_m:undefined},
    back: {shape:"pad", size_m:[.20,.36,.42]},
    hip: {shape:"pad", size_m:[.26,.30,.18]},
  }[support];
  const obstacle = {
    id:"tori_contact", shape:config.shape, tori_mass_kg:75,
    start_m:vround(start), phases,
    surface:"cloth", friction:.8,
  };
  if (config.size_m) obstacle.size_m = config.size_m;
  return obstacle;
}

/** Hand forces prescribed in the initial Uke frame. Not a force replay from AnimSpec. */
function pullProfile(technique) {
  const all = technique === "tai" ? [
    {name:"manga",site:"right_sleeve_elbow",f:[[0,[0,0,0]],[.30,[180,0,140]],[.60,[280,0,160]],[1.35,[280,0,-60]],[1.57,[335,-35,-180]],[1.90,[230,-35,-180]],[2.15,[0,0,0]]]},
    {name:"solapa",site:"sternum",f:[[0,[0,0,0]],[.30,[130,0,110]],[.60,[210,0,100]],[1.35,[200,0,-80]],[1.57,[265,0,-125]],[1.85,[90,0,-80]],[2.10,[0,0,0]]]},
  ] : [
    {name:"manga",site:"right_sleeve_elbow",f:[[0,[0,0,0]],[.30,[-100,-40,50]],[.70,[-195,-105,70]],[1.10,[-245,-120,10]],[1.40,[-300,-130,-95]],[1.75,[-70,-30,-25]],[2,[0,0,0]]]},
    {name:"solapa",site:"sternum",f:[[0,[0,0,0]],[.30,[-125,-60,30]],[.70,[-240,-115,40]],[1.10,[-310,-140,-65]],[1.40,[-325,-150,-80]],[1.75,[-75,-35,-25]],[2,[0,0,0]]]},
  ];
  return all.map(e => ({
    id:"experimental_"+e.name,kind:"pull",actor:"uke",site:e.site,frame:"actor_initial",
    max_force_n:500,max_power_w:600,
    keys:e.f.map(([t_s,force_n])=>({t_s,force_n})),
  }));
}

export function buildThrowPhysicsScenario(frames, { technique, model = "support", support = "leg", withoutSupport = false } = {}) {
  inputFrames(frames);
  if (!["tai","osoto"].includes(technique)) throw new Error("Técnica desconocida.");
  if (!["full","support"].includes(model)) throw new Error("Modo físico desconocido.");
  if (!["leg","back","hip"].includes(support)) throw new Error("Apoyo desconocido.");
  if (model === "full" && withoutSupport) throw new Error("No hay control 'sin contacto' con agarres equivalentes en modo Tori completo.");
  const duration = round(frames.at(-1).t - frames[0].t,4);
  if (duration < 2.3) throw new Error("Necesitamos un guion de al menos 2,3 s para el perfil de fuerzas de prueba.");
  const scenario = {
    schema:"hpl.scenario.v1",id:`throw_axis_${technique}_${model}_${withoutSupport?"control":support}`,
    duration_s:duration,record_hz:50,
    ground:{friction:1},
    actors:[{id:"uke",kind:"humanoid",mass_kg:75,height_m:1.75,strength:1,attitude:"tense",stance:"neutral"}],
    props:[],effectors:[],
  };
  if (model === "full") {
    scenario.tori = {
      mass_kg:75,
      body:{
        height_m:1.75, friction:.8,
        grips:{left:{site:"right_sleeve_elbow",max_force_n:300},
               right:{site:"sternum",max_force_n:300,release_s:technique==="tai"?1.85:1.58}},
        keys:toriKeys(frames),
      },
    };
  } else {
    scenario.effectors = pullProfile(technique); // exactly the same with / without the support
    if (!withoutSupport) scenario.obstacles = [supportObstacle(frames,support)];
  }
  return scenario;
}

export function physicsScenarioHash(scenario) {
  const bytes = new TextEncoder().encode(JSON.stringify(scenario));
  let binary = "";
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return "#scn=" + btoa(binary).replace(/\+/g,"-").replace(/\//g,"_").replace(/=+$/,"");
}
