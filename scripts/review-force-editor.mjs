import {chromium} from "playwright";
import {mkdir,rename,writeFile} from "node:fs/promises";
import path from "node:path";

const out=path.resolve("media-force-editor");
await mkdir(out,{recursive:true});
const browser=await chromium.launch({headless:true,
  executablePath:"/usr/bin/google-chrome",
  args:["--no-sandbox","--disable-dev-shm-usage",
    "--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
const context=await browser.newContext({
  viewport:{width:1440,height:900},deviceScaleFactor:1,
  recordVideo:{dir:out,size:{width:1440,height:900}},
});
const page=await context.newPage();
const errors=[];
page.on("pageerror",e=>errors.push(e.message));
const result={status:"started",errors,steps:[]};
const snapshot=async(label)=>{
  await page.locator("#physics-viewport").screenshot({path:path.join(out,label+"-visor.png")});
  await page.screenshot({path:path.join(out,label+"-panel.png"),fullPage:true});
};
const physicsTime=async(t)=>await page.waitForFunction(min=>{
  const text=document.querySelector("#physics-readout")?.textContent||"";
  const m=text.match(/t\s*=\s*([0-9.]+)\s*s/);
  return !!m && Number(m[1])>=min;
},t,{timeout:90000});
try {
  await page.goto("http://127.0.0.1:4173/humanoid-physics-lab/",{waitUntil:"domcontentloaded"});
  await page.locator("#physics-model").selectOption("standing");
  await page.locator("#physics-load").click();
  await page.locator("#physics-toggle").waitFor({state:"visible"});
  await page.waitForFunction(()=>!document.querySelector("#physics-toggle")?.disabled,
    undefined,{timeout:45000});
  if(!await page.locator("#physics-viewport canvas").count()) {
    throw new Error("No hay canvas 3D, estado: "+
      await page.locator("#physics-status").innerText());
  }
  result.steps.push("model-loaded");
  const editorCheckbox=page.locator(".force-editor-toggle input[type=checkbox]");
  await editorCheckbox.check();
  await page.locator(".advanced-forces").waitFor({state:"visible"});
  const rows=page.locator(".edited-force-row");
  if(await rows.count()!==1)throw new Error("Editor no inicia con una fuerza");
  await snapshot("01-editor");
  await page.getByRole("button",{name:"+ Añadir fuerza"}).click();
  if(await rows.count()!==2)throw new Error("No se añadió segunda fuerza");
  const second=rows.nth(1);
  const numeric=second.locator("input[type=number]");
  // Initial f2 is 0,+120,0 at t=1.2. Move to a simultaneous oblique
  // sustained load so the 3D glyph remains visible long enough to capture.
  await numeric.nth(1).fill("-180");
  await numeric.nth(1).press("Tab");
  await numeric.nth(2).fill("90");
  await numeric.nth(2).press("Tab");
  await numeric.nth(3).fill("1");
  await numeric.nth(3).press("Tab");
  await numeric.nth(4).fill("0.8");
  await numeric.nth(4).press("Tab");
  await second.locator("select").last().selectOption("constant");
  result.steps.push("two-3d-forces-configured");
  await snapshot("02-two-vectors");
  await page.getByRole("button",{name:"Ejecutar secuencia"}).click();
  if(!await page.getByRole("button",{name:"+ Añadir fuerza"}).isDisabled())
    throw new Error("La edición no está bloqueada durante la simulación");
  await physicsTime(1.30);
  await snapshot("03-multiple-active");
  await physicsTime(6.01);
  await snapshot("04-final");
  const buttonsUnlocked=!(await page.getByRole("button",{name:"+ Añadir fuerza"}).isDisabled());
  if(!buttonsUnlocked)throw new Error("Editor bloqueado tras terminar");
  result.steps.push("simulation-ended-editor-unlocked");
  result.physicsStatus=await page.locator("#physics-status").innerText();
  result.editorStatus=await page.locator(".advanced-forces [role=status]").first().innerText().catch(()=>null);
  result.status=errors.length?"browser-errors":"passed";
  console.log("FORCE-EDITOR-RESULT "+JSON.stringify(result));
} catch(e) {
  result.status="failed";result.error=String(e);
  result.physicsStatus=await page.locator("#physics-status").innerText().catch(()=>"");
  console.error("FORCE-EDITOR-FAILED "+JSON.stringify(result));
} finally {
  const video=page.video();
  await page.close();
  await context.close();
  if(video) await rename(await video.path(),path.join(out,"force-editor-real.webm"));
  await browser.close();
  await writeFile(path.join(out,"results.json"),JSON.stringify(result,null,2));
}
if(result.status!=="passed")process.exitCode=1;
