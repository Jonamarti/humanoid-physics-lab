// Run against Vite dev server. Playwright is an optional local recording tool.
import { chromium } from "playwright";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";

const out=path.resolve(process.env.HPL_CAPTURE_OUT??"media/load-transfer");
const base=process.env.HPL_BASE_URL??"http://127.0.0.1:4173/humanoid-physics-lab/";
await mkdir(out,{recursive:true});
const browser=await chromium.launch({headless:true,
  ...(process.env.CHROME_EXECUTABLE?{executablePath:process.env.CHROME_EXECUTABLE}:{}),
  args:["--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
const results=[];
try {
  // Exercise the real application before recording the instrumented bench.
  const smoke=await browser.newPage({viewport:{width:1400,height:900}});
  const errors=[];smoke.on("pageerror",e=>errors.push(e.message));
  await smoke.goto(base);
  await smoke.locator("#physics-model").selectOption("standing");
  await smoke.locator("#physics-load").click();
  await smoke.waitForFunction(()=>!document.querySelector("#v3-push-run").disabled);
  const editor=smoke.locator("#multi-force-editor");
  await editor.locator('input[type="checkbox"]').check();
  await editor.getByLabel("Fx (N)",{exact:true}).fill("120");
  await editor.getByLabel("Fy (N)",{exact:true}).fill("30");
  await editor.getByLabel("Fz (N)",{exact:true}).fill("10");
  await editor.getByRole("button",{name:"Ejecutar secuencia",exact:true}).click();
  await smoke.waitForFunction(()=>Number((document.querySelector("#physics-readout").textContent.match(/t\s*=\s*([\d.]+)/)||[])[1])>=5.96,null,{timeout:120000});
  const editorStatus=await editor.getByRole("status").innerText();
  await writeFile(path.join(out,"force-editor.html"),await editor.innerHTML());
  await editor.screenshot({path:path.join(out,"force-editor.png")});
  await editor.locator('input[type="checkbox"]').uncheck();
  await smoke.locator("#push-direction").selectOption("forward");
  await smoke.locator("#push-peak").fill("120");
  await smoke.locator("#push-duration").fill("160");
  await smoke.locator("#v3-balance-mode").selectOption("whole-body-step");
  await smoke.locator("#v3-push-run").click();
  await smoke.waitForFunction(()=>Number((document.querySelector("#physics-readout").textContent.match(/t\s*=\s*([\d.]+)/)||[])[1])>=5.96,null,{timeout:120000});
  await smoke.screenshot({path:path.join(out,"application.png"),fullPage:true});
  results.push({app:true,errors,editorStatus,status:await smoke.locator("#physics-status").innerText(),
    step:await smoke.locator("#step-readout").innerText()});
  if(errors.length)throw new Error(errors.join("\n"));
  await smoke.close();
  const edit=await browser.newPage({viewport:{width:1000,height:800}});
  await edit.goto(base);
  const points=await edit.evaluate(async base=>{
    const {MujocoSimulation}=await import(base+"src/physics/mujoco.ts");
    const {BallViewer}=await import(base+"src/renderer/ballViewer.ts");
    document.body.innerHTML='<div id="edit-view" style="width:900px;height:720px"></div>';
    const sim=await MujocoSimulation.create(await(await fetch(base+"models/humanoid_gait_v3.xml")).text());
    const viewer=new BallViewer(document.querySelector("#edit-view"));
    viewer.showArticulated(sim.geometrySpecs());viewer.setArticulatedPoses(sim.geometryPoses());
    globalThis.editorChecks={dragged:[],picked:[]};
    viewer.configureForceEditing("f",(_id,geom,p)=>{
      const a=sim.bodyAttachmentAtGeomPoint(geom,p);
      const reconstructed=sim.bodyAttachmentWorldPosition(a.bodyIndex,a.localComOffsetM);
      globalThis.editorChecks.picked.push({geom,error:Math.hypot(...p.map((x,i)=>x-reconstructed[i]))});
    },(_id,d)=>globalThis.editorChecks.dragged.push(d));
    const point=sim.siteWorldPosition("sternum_impact");
    viewer.setMultiForces([{id:"f",point,force:[120,30,10],active:false}]);
    await new Promise(requestAnimationFrame);
    const head=viewer.editableHeads.get("f");
    const rect=viewer.renderer.domElement.getBoundingClientRect();
    const project=p=>{const v=p.clone().project(viewer.camera);return {x:rect.x+(v.x+1)*rect.width/2,y:rect.y+(1-v.y)*rect.height/2};};
    globalThis.armPick=()=>viewer.armForcePointPick("f");
    const body=head.position.clone().set(point[0],point[2],-point[1]);
    return {head:project(head.position),body:project(body)};
  },base);
  await edit.mouse.move(points.head.x,points.head.y);await edit.mouse.down();
  await edit.mouse.move(points.head.x+50,points.head.y+35,{steps:8});await edit.mouse.up();
  await edit.evaluate(()=>globalThis.armPick());
  await edit.mouse.click(points.body.x,points.body.y);
  const checks=await edit.evaluate(()=>globalThis.editorChecks);
  if(!checks.dragged.length || !checks.picked.length || checks.picked.some(p=>p.error>1e-7))
    throw new Error("3D force drag/pick regression: "+JSON.stringify(checks));
  results.push({editor3d:checks});
  await edit.screenshot({path:path.join(out,"force-editor-3d.png")});await edit.close();
  for(const trial of [
    {name:"baseline-left300",force:[0,300,0],options:{}},
    {name:"early-left300",force:[0,300,0],options:{captureLookaheadS:.2,planningDelayS:.04}},
    {name:"baseline-forward200",force:[200,0,0],options:{}},
    {name:"baseline-forward120-no-step",force:[120,0,0],options:{}},
  ])for(const view of ["front","side"]){
    const name=trial.name+"-"+view;
    const context=await browser.newContext({viewport:{width:1280,height:800},
      recordVideo:{dir:out,size:{width:1280,height:800}}});
    const page=await context.newPage();
    await page.goto(base);
    const row=await page.evaluate(async ({trial,view,base})=>{
      const {MujocoSimulation}=await import(base+"src/physics/mujoco.ts");
      const {BallViewer}=await import(base+"src/renderer/ballViewer.ts");
      const {editedForceAtTime}=await import(base+"src/physics/forceSequence.ts");
      const {rootTiltDegrees}=await import(base+"src/physics/standingPush.ts");
      document.body.innerHTML=`<style>body{margin:0;background:#102032;color:#eaf4ff;font:16px system-ui}h2,p{margin:12px}#view{position:absolute;left:0;top:90px;width:760px;height:710px}#graphs{position:absolute;left:770px;top:90px}#stats{font:15px monospace;white-space:pre}</style><h2>${trial.name} · ${view} · MuJoCo físico real</h2><p>75 kg · Δt 2 ms · pulso 160 ms · cargas resueltas a 50 Hz · sin recuperación inventada</p><div id="view"></div><canvas id="graphs" width="500" height="680"></canvas><p id="stats"></p>`;
      const xml=await(await fetch(base+"models/humanoid_gait_v3.xml")).text();
      const sim=await MujocoSimulation.create(xml);
      sim.enableExperimentalStanding();sim.enableExperimentalReactiveRecovery(trial.options);
      const viewer=new BallViewer(document.querySelector("#view"));
      viewer.showArticulated(sim.geometrySpecs());
      // TS private fields are used only by this optional diagnostic recorder.
      viewer.camera.position.set(...(view==="front"?[5,1.8,-.45]:[.45,1.8,-5]));
      viewer.controls.target.set(.45,.8,-.45);viewer.controls.update();
      const ctx=document.querySelector("#graphs").getContext("2d");
      const trace=[];
      const pulse={id:"push",site:"sternum_impact",forceN:trial.force,startS:1,durationS:.16,waveform:"half-sine"};
      for(let n=0;n<300;n++){
        await new Promise(requestAnimationFrame);
        sim.step(10,t=>{const f=editedForceAtTime(t,pulse);if(Math.hypot(...f)>0)sim.applyWorldForceAtSite(pulse.site,f);});
        const st=sim.state(),g=sim.humanoidGroundSupportObservation(),b=sim.humanoidBalanceObservation();
        const step=sim.experimentalStepStatus(),r=sim.experimentalReactiveRecoveryStatus();
        const tilt=rootTiltDegrees(st.qpos),v=b.comVelocityWorldMps;
        const cp=b.comWorldM.map((x,i)=>x+v[i]*Math.sqrt(b.comWorldM[2]/9.81));
        trace.push([st.time,g.left.verticalLoadN,g.right.verticalLoadN,b.comWorldM[1],cp[1],tilt]);
        viewer.setArticulatedPoses(sim.geometryPoses());
        viewer.setBalanceProjection(b.comWorldM,g.loadedSupportPolygonXY,true);
        const f=editedForceAtTime(st.time,pulse);
        viewer.setForceArrow(sim.siteWorldPosition("sternum_impact"),f,Math.hypot(...f)>0);
        ctx.fillStyle="#102032";ctx.fillRect(0,0,500,680);
        const chart=(top,title,series,min,max)=>{
          ctx.fillStyle="#fff";ctx.font="15px system-ui";ctx.fillText(title,12,top);
          ctx.strokeStyle="#345";ctx.strokeRect(40,top+16,440,150);
          for(const [index,color] of series){ctx.strokeStyle=color;ctx.beginPath();
            trace.forEach((p,j)=>{const x=40+p[0]/6*440,y=top+166-(Math.max(min,Math.min(max,p[index]))-min)/(max-min)*150;
              if(j===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();}
        };
        chart(20,"Normal: izquierda azul / derecha naranja (0–1200 N)",[[1,"#50d9f9"],[2,"#ffbd59"]],0,1200);
        chart(220,"COM Y azul / captura Y naranja (−0,2 a 1 m)",[[3,"#50d9f9"],[4,"#ffbd59"]],-.2,1);
        chart(420,"Inclinación de raíz (0–100°)",[[5,"#ff6880"]],0,100);
        ctx.fillStyle="#fff";ctx.fillText(`t=${st.time.toFixed(2)} s · ${step?.phase??r.phase}`,12,625);
        ctx.fillText(`L=${g.left.verticalLoadN.toFixed(0)} N · R=${g.right.verticalLoadN.toFixed(0)} N`,12,651);
      }
      const row={name:trial.name,view,step:sim.experimentalStepStatus(),coordinator:sim.experimentalReactiveRecoveryStatus(),trace};
      sim.dispose();return row;
    },{trial,view,base});
    await page.screenshot({path:path.join(out,name+".png")});
    const video=page.video();await page.close();await context.close();
    await video.saveAs(path.join(out,name+".webm"));
    await video.delete();
    await writeFile(path.join(out,name+".json"),JSON.stringify(row));
    results.push({name,phase:row.step?.phase??row.coordinator.phase});
    console.log("RECORDED",name,results.at(-1).phase);
  }
}finally{await writeFile(path.join(out,"results.json"),JSON.stringify(results,null,2));await browser.close();}
