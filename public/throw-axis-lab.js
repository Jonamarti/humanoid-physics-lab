// Self-contained kinematic diagnostic. No imports from HPL/AnimSpec, no persistence or network requests.
// World coordinates: X forward for Uke, Y to Uke's left, Z up. GrappleMap: (x,z,-y).
const $ = id => document.getElementById(id);
const NAMES = ["LeftToe","RightToe","LeftHeel","RightHeel","LeftAnkle","RightAnkle","LeftKnee","RightKnee","LeftHip","RightHip","LeftShoulder","RightShoulder","LeftElbow","RightElbow","LeftWrist","RightWrist","LeftHand","RightHand","LeftFingers","RightFingers","Core","Neck","Head"];
const BONES = [[0,4],[2,4],[4,6],[6,8],[1,5],[3,5],[5,7],[7,9],[8,20],[9,20],[20,21],[21,22],[21,10],[21,11],[10,12],[12,14],[14,16],[11,13],[13,15],[15,17]];
const COLORS={uke:"#68c7ef",tori:"#f6bf73",head:"#f178a0",support:"#7ae2b3"};
const V=(a,b)=>a.map((v,i)=>v+b[i]);const sub=(a,b)=>a.map((v,i)=>v-b[i]);const mul=(a,s)=>a.map(v=>v*s);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);const len=a=>Math.hypot(...a);const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const mix=(a,b,u)=>a.map((v,i)=>v+(b[i]-v)*u);
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const deg=x=>x*Math.PI/180;
function interp(k,t){if(t<=k[0][0])return k[0][1];for(let i=1;i<k.length;i++)if(t<=k[i][0]){let u=(t-k[i-1][0])/(k[i][0]-k[i-1][0]);return Array.isArray(k[i][1])?mix(k[i-1][1],k[i][1],u):k[i-1][1]+(k[i][1]-k[i-1][1])*u;}return k[k.length-1][1];}
function rx(v,a){const c=Math.cos(a),s=Math.sin(a);return [v[0],v[1]*c-v[2]*s,v[1]*s+v[2]*c];}
function ry(v,a){const c=Math.cos(a),s=Math.sin(a);return [v[0]*c+v[2]*s,v[1],-v[0]*s+v[2]*c];}
function rz(v,a){const c=Math.cos(a),s=Math.sin(a);return [v[0]*c-v[1]*s,v[0]*s+v[1]*c,v[2]];}
function skeleton(pel,yaw=0,pitch=0,roll=0){const locals=[
[.18,.09,-.92],[.18,-.09,-.92],[-.05,.09,-.93],[-.05,-.09,-.93],[0,.09,-.88],[0,-.09,-.88],
[.05,.09,-.45],[.05,-.09,-.45],[0,.09,0],[0,-.09,0],
[0,.19,.40],[0,-.19,.40],[.12,.35,.26],[.12,-.35,.26],[.26,.38,.14],[.26,-.38,.14],
[.29,.39,.12],[.29,-.39,.12],[.36,.40,.12],[.36,-.40,.12],[0,0,.18],[0,0,.58],[.05,0,.70]];
return locals.map(v=>V(pel,rz(ry(rx(v,roll),pitch),yaw)));}
const CONFIG={
tai:{duration:3.2,ukePel:[[0,[0,0,.97]],[.6,[.06,0,1]],[1.05,[.14,0,1]],[1.57,[.28,0,.95]],[1.72,[.46,0,.93]],[1.88,[.70,0,.92]],[2.05,[.94,0,.56]],[2.2,[1.04,0,.13]],[3.2,[1.04,0,.13]]],
pitch:[[0,0],[.6,6],[1.05,16],[1.57,50],[1.72,108],[1.88,196],[2.05,252],[2.2,270],[3.2,270]],
yaw:[[0,0],[1.05,0],[1.57,16],[2.2,22],[3.2,22]],
toriPel:[[0,[.64,0,.93]],[.6,[.63,.03,.92]],[1.05,[.52,.23,.86]],[1.57,[.26,.36,.69]],[2.2,[.34,.33,.62]],[3.2,[.34,.33,.62]]],
toriYaw:[[0,180],[.6,180],[1.05,270],[1.57,360],[3.2,367]],toriPitch:[[0,3],[1.05,6],[1.57,26],[2.2,66],[3.2,66]]},
osoto:{duration:3.2,ukePel:[[0,[0,0,.97]],[.7,[-.03,-.08,.97]],[1.03,[-.07,-.12,.97]],[1.18,[-.15,-.15,.96]],[1.3,[-.26,-.20,.86]],[1.42,[-.38,-.32,.68]],[1.58,[-.5,-.52,.40]],[1.75,[-.55,-.62,.13]],[3.2,[-.55,-.62,.13]]],
pitch:[[0,0],[.7,-7],[1.03,-13],[1.18,-32],[1.3,-62],[1.42,-86],[1.58,-90],[3.2,-90]],
yaw:[[0,0],[1.22,0],[1.34,-18],[1.46,-50],[1.6,-84],[1.75,-98],[3.2,-98]],roll:[[0,0],[.7,7],[1.03,10],[1.3,6],[1.5,0],[3.2,0]],
toriPel:[[0,[.64,-.05,.93]],[.7,[.32,-.30,.9]],[1.03,[.11,-.37,.93]],[1.33,[.06,-.39,.94]],[2.7,[.06,-.39,.93]],[3.2,[.14,-.34,.84]]],
toriYaw:[[0,180],[3.2,180]],toriPitch:[[0,3],[1.03,8],[1.33,63],[1.75,69],[3.2,54]]}
};
const synthetic=tech=>{const c=CONFIG[tech];const n=Math.round(c.duration*30);const frames=[];for(let i=0;i<=n;i++){const t=i/30;let u=skeleton(interp(c.ukePel,t),deg(interp(c.yaw,t)),deg(interp(c.pitch,t)),deg(interp(c.roll||[[0,0],[c.duration,0]],t)));
let to=skeleton(interp(c.toriPel,t),deg(interp(c.toriYaw,t)),deg(interp(c.toriPitch,t)));
if(tech==="tai"){const footR=interp([[0,[.64,.09,.09]],[1.2,[.44,.18,.09]],[1.57,[.10,-.33,.09]],[3.2,[.10,-.33,.09]]],t);to[5]=footR;to[1]=V(footR,[0,-.15,-.02]);to[3]=V(footR,[-.05,.04,-.04]);to[7]=mix(to[9],footR,.51);}
if(tech==="osoto"){const footR=interp([[0,[.64,.09,.09]],[1.03,[-.40,-.33,.42]],[1.33,[.87,-.30,1]],[2.7,[.87,-.30,1]],[3.2,[.52,-.16,.09]]],t);to[5]=footR;to[1]=V(footR,[.15,0,-.02]);to[3]=V(footR,[-.05,0,-.02]);to[7]=mix(to[9],footR,.51);}
frames.push({t,tori:to,uke:u});}return frames;};
const state={tech:"tai",frames:synthetic("tai"),imported:false,time:0,playing:false,last:null,angle:null};
function parse(raw){if(!raw||!Array.isArray(raw.t)||!Array.isArray(raw.tori)||!Array.isArray(raw.uke))throw Error("Se requieren t[], tori[] y uke[].");
if(raw.t.length<2||raw.t.length!==raw.tori.length||raw.t.length!==raw.uke.length||raw.t.length>12000)throw Error("Número de fotogramas incompatible (2–12000).");
const frames=raw.t.map((t,i)=>{if(!Number.isFinite(t)||(i&&t<=raw.t[i-1]))throw Error("Tiempos no crecientes.");const convert=(a)=>{if(!Array.isArray(a)||a.length!==23)throw Error("Cada personaje debe tener 23 puntos.");return a.map(p=>{if(!Array.isArray(p)||p.length!==3||p.some(v=>!Number.isFinite(v)||Math.abs(v)>100))throw Error("Coordenada inválida.");return [p[0],-p[2],p[1]];});};return {t,tori:convert(raw.tori[i]),uke:convert(raw.uke[i])};});return frames;}
function pelvis(p){return mul(V(p[8],p[9]),.5);}
function yawFromShoulders(u){const left=sub(u[10],u[11]);return Math.atan2(-left[0],left[1]);}
function unwrappedYaws(frames){let prev=yawFromShoulders(frames[0].uke),value=0;return frames.map((f,i)=>{let a=yawFromShoulders(f.uke);if(i){let d=a-prev;while(d>Math.PI)d-=2*Math.PI;while(d< -Math.PI)d+=2*Math.PI;value+=d;}prev=a;return value;});}
function rebuildAngle(){state.angle=unwrappedYaws(state.frames);state.time=0;state.last=null;state.playing=false;$("play").textContent="▶ Reproducir";$("time").value="0";}
function alternative(points,at){const gain=Number($("gain").value)/100, extra=$("correction").value==="reverse"?-2*at*gain:0,p=pelvis(points);return points.map(v=>V(p,rz(sub(v,p),extra)));}
function current(){const frames=state.frames;const end=frames[frames.length-1].t;const t=state.time;let i=0;while(i+1<frames.length&&frames[i+1].t<t)i++;const j=Math.min(i+1,frames.length-1);const dt=frames[j].t-frames[i].t;const u=dt>0?clamp((t-frames[i].t)/dt,0,1):0;const f={t,tori:frames[i].tori.map((p,k)=>mix(p,frames[j].tori[k],u)),uke:frames[i].uke.map((p,k)=>mix(p,frames[j].uke[k],u))};const angle=state.angle[i]+(state.angle[j]-state.angle[i])*u;return {f,angle,end};}
function project(v,view){const scale=260,center=[360,395];let xx,yy;if(view==="side"){xx=v[0];yy=v[2];}else if(view==="front"){xx=v[1];yy=v[2];}else if(view==="top"){xx=v[0];yy=v[1];}else{xx=.78*v[0]-.64*v[1];yy=.83*v[2]-.32*v[0]-.24*v[1];}return [center[0]+xx*scale,center[1]-yy*scale];}
function line(ctx,a,b,color,width=3){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
function circle(ctx,p,color,r=5){ctx.beginPath();ctx.arc(p[0],p[1],r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
function segmentDistance(p1,q1,p2,q2){const u=sub(q1,p1),v=sub(q2,p2),w=sub(p1,p2),a=dot(u,u),b=dot(u,v),c=dot(v,v),d=dot(u,w),e=dot(v,w),D=a*c-b*b;let sN=D,tN=D,sD=D,tD=D;
if(D<1e-12){sN=0;sD=1;tN=e;tD=c;}else{sN=b*e-c*d;tN=a*e-b*d;if(sN<0){sN=0;tN=e;tD=c;}else if(sN>sD){sN=sD;tN=e+b;tD=c;}}
if(tN<0){tN=0;if(-d<0)sN=0;else if(-d>a)sN=sD;else{sN=-d;sD=a;}}else if(tN>tD){tN=tD;if((-d+b)<0)sN=0;else if((-d+b)>a)sN=sD;else{sN=-d+b;sD=a;}}
const sc=Math.abs(sN)<1e-12?0:sN/sD,tc=Math.abs(tN)<1e-12?0:tN/tD;return len(sub(V(w,mul(u,sc)),mul(v,tc)));}
function supports(tori){return {leg:[tori[9],tori[7],tori[5]],back:[pelvis(tori),tori[20],tori[21]],hip:[tori[8],tori[9]]};}
function selectedSupports(s){const key=$("support").value;return key==="both"?Object.values(s):[s[key]];}
function proximity(tori,uke){const parts=selectedSupports(supports(tori));const body=[[pelvis(uke),uke[20]],[uke[20],uke[21]],[uke[8],uke[6]],[uke[9],uke[7]]];let d=Infinity;for(const p of parts)for(let i=1;i<p.length;i++)for(const b of body)d=Math.min(d,segmentDistance(p[i-1],p[i],b[0],b[1]));return d;}
function drawCharacter(ctx,p,color,view,muted){ctx.globalAlpha=muted?.48:1;for(const [a,b] of BONES)line(ctx,project(p[a],view),project(p[b],view),color,a===20||b===20?4:2.7);for(const i of [4,5,6,7,8,9,10,11,20,21])circle(ctx,project(p[i],view),color,3);circle(ctx,project(p[22],view),COLORS.head,10);ctx.globalAlpha=1;}
function drawSupport(ctx,tori,view){for(const parts of selectedSupports(supports(tori))){for(let i=1;i<parts.length;i++)line(ctx,project(parts[i-1],view),project(parts[i],view),COLORS.support,12);for(const p of parts)circle(ctx,project(p,view),COLORS.support,6);}}
function drawTrace(ctx,variant,view,frames,now){const points=[];for(let i=0;i<frames.length;i+=Math.max(1,Math.floor(frames.length/80))){const f=frames[i];if(f.t>now)break;const uke=variant?alternative(f.uke,state.angle[i]):f.uke;points.push(project(uke[22],view));}for(let i=1;i<points.length;i++)line(ctx,points[i-1],points[i],COLORS.head,1.6);}
function paint(canvas,f,uke,variant,end){const ctx=canvas.getContext("2d");if(!ctx)return;const view=$("view").value;ctx.clearRect(0,0,720,660);
ctx.strokeStyle="#39536a";ctx.lineWidth=1;for(let m=-2;m<=3;m+=.5){line(ctx,project([m,-2,0],view),project([m,2,0],view),"#294056",1);line(ctx,project([-2,m,0],view),project([3,m,0],view),"#294056",1);}
if($("trace").checked)drawTrace(ctx,variant,view,state.frames,f.t);
if($("mode").value==="full")drawCharacter(ctx,f.tori,COLORS.tori,view,true);else drawSupport(ctx,f.tori,view);
drawCharacter(ctx,uke,COLORS.uke,view,false);circle(ctx,project(pelvis(uke),view),"#fff",5);
const d=proximity(f.tori,uke);ctx.font="18px ui-monospace,monospace";ctx.fillStyle="#e5f2fc";ctx.fillText("t="+f.t.toFixed(2)+" s",20,30);ctx.fillText("Distancia esqueleto: "+Math.round(d*100)+" cm",20,56);
const h=project(uke[22],view),p=project(pelvis(uke),view);line(ctx,p,h,COLORS.head,1);ctx.font="15px system-ui";ctx.fillStyle="#b8cada";ctx.fillText(variant?"Hipótesis: yaw invertido":"Grabación / entrada intacta",20,635);}
function refresh(){const {f,angle,end}=current();const newUke=alternative(f.uke,angle);paint($("original"),f,f.uke,false,end);paint($("variant"),f,newUke,true,end);
const da=proximity(f.tori,f.uke),db=proximity(f.tori,newUke),a=pelvis(f.uke),h=f.uke[22],bh=newUke[22];const hand= len(sub(f.tori[16],newUke[17]));const fmt=n=>n.toFixed(2).replace(".",",");
$("clock").textContent=fmt(state.time)+" s";
$("time").value=String(Math.round(state.time/end*1000));
$("gain-label").textContent=$("gain").value+" %";
$("report").textContent="Fuente: "+(state.imported?"JSON AnimSpec importado localmente":"esquema sintético aproximado")+
"\nGiro horizontal medido (sin corregir): "+(angle*180/Math.PI).toFixed(1)+"°"+
"\nCabeza - pelvis, lateral original / variante: "+fmt(h[1]-a[1])+" / "+fmt(bh[1]-a[1])+" m"+
"\nDistancia mínima a apoyos, original / variante: "+fmt(da)+" / "+fmt(db)+" m"+
"\nSeparación mano izquierda Tori - mano derecha Uke (variante): "+fmt(hand)+" m"+
"\nNo se calculan fuerzas, torque, penetración ni contacto físico real.";
}
function reset(tech){state.tech=tech;state.frames=synthetic(tech);state.imported=false;rebuildAngle();$("source").textContent="Datos: ejemplo esquemático (no fiel al guion exacto).";refresh();}
$("tech").addEventListener("change",e=>reset(e.target.value));$("reset").addEventListener("click",()=>{$("file").value="";reset($("tech").value);});
$("file").addEventListener("change",async e=>{const file=e.target.files?.[0];if(!file)return;try{if(file.size>25_000_000)throw Error("El archivo supera 25 MB.");const frames=parse(JSON.parse(await file.text()));state.frames=frames;state.imported=true;rebuildAngle();$("source").textContent="Cargado localmente: "+file.name+" ("+frames.length+" fotogramas). No se ha enviado ni guardado.";refresh();}catch(err){$("source").textContent="Error: "+(err instanceof Error?err.message:String(err));}});
for(const id of ["view","mode","support","correction","gain","trace","contacts"])$(id).addEventListener("input",refresh);
$("time").addEventListener("input",e=>{state.time=Number(e.target.value)/1000*state.frames.at(-1).t;state.playing=false;$("play").textContent="▶ Reproducir";refresh();});
$("play").addEventListener("click",()=>{state.playing=!state.playing;$("play").textContent=state.playing?"❚❚ Pausar":"▶ Reproducir";if(state.playing&&state.time>=state.frames.at(-1).t)state.time=0;state.last=null;});
function step(s){state.playing=false;$("play").textContent="▶ Reproducir";state.time=clamp(state.time+s/30,0,state.frames.at(-1).t);refresh();}
$("step-back").addEventListener("click",()=>step(-1));$("step-forward").addEventListener("click",()=>step(1));
function tick(ts){if(state.playing){if(state.last!==null)state.time=Math.min(state.frames.at(-1).t,state.time+(ts-state.last)/1000);state.last=ts;if(state.time>=state.frames.at(-1).t){state.playing=false;$("play").textContent="▶ Reproducir";}refresh();}else state.last=null;requestAnimationFrame(tick);}
rebuildAngle();refresh();requestAnimationFrame(tick);