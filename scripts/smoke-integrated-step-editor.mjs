import { chromium } from "playwright";
import { existsSync } from "node:fs";
import { mkdir, rename, writeFile } from "node:fs/promises";
import path from "node:path";

const output=path.resolve("integration-media");
await mkdir(output,{recursive:true});
const executablePath=["/usr/bin/google-chrome","/usr/bin/chromium"].find(existsSync);
const browser=await chromium.launch({headless:true,
  ...(executablePath?{executablePath}:{}),
  args:["--no-sandbox","--disable-dev-shm-usage",
    "--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
const context=await browser.newContext({viewport:{width:1280,height:760},
  deviceScaleFactor:1,recordVideo:{dir:output,size:{width:1280,height:760}}});
const page=await context.newPage();
const issues=[];
page.on("pageerror",error=>issues.push(error.message));
const result={};
async function atTime(t){
  await page.waitForFunction(target=>{
    const text=document.querySelector("#physics-readout")?.textContent||"";
    const match=text.match(/t\s*=\s*([\d.]+)/);
    return !!match && Number(match[1])>=target;
  },t,{timeout:120000});
}
async function shot(name){
  await page.locator("#physics-viewport").screenshot({
    path:path.join(output,name+"-visor.png")});
  await page.screenshot({path:path.join(output,name+"-full.png"),fullPage:true});
}
try{
  await page.goto("http://127.0.0.1:4173/humanoid-physics-lab/",
    {waitUntil:"domcontentloaded",timeout:30000});
  await page.locator("#physics-model").selectOption("standing");
  await page.locator("#physics-load").click();
  await page.waitForFunction(()=>
    document.querySelector("#physics-toggle")?.disabled===false,
    undefined,{timeout:60000});
  const step=page.locator("#step-run");
  const editorCheck=page.locator(".force-editor-toggle input[type=checkbox]");
  if(!await step.isEnabled())throw Error("Step button disabled in nominal standing");
  await shot("00-inicio-integrado");
  await page.locator("#step-foot").selectOption("left");
  await page.locator("#step-direction").selectOption("diagonal");
  await page.locator("#step-cartesian").selectOption("0.2");
  await step.click();
  await atTime(1.65);
  await shot("01-paso-despegue");
  await atTime(5.18);
  await shot("02-paso-resultado");
  const stepStatus=await page.locator("#physics-status").innerText();
  const stepDetails=await page.locator("#step-readout").innerText();
  if(!stepDetails.includes("velocidad horizontal precontacto"))
    throw Error("Missing actual foot velocity telemetry");
  if(!(/Paso experimental no completado|caída física/.test(stepStatus)))
    throw Error("Unproven physical step incorrectly accepted: "+stepStatus);
  await editorCheck.check();
  if(await step.isEnabled())throw Error("Step button enabled while force editor active");
  await page.getByRole("button",{name:"+ Añadir fuerza"}).click();
  if((await page.locator(".edited-force-row").count())!==2)
    throw Error("Expected 2 independent forces");
  await shot("03-editor-dos-fuerzas");
  await page.getByRole("button",{name:"Ejecutar secuencia"}).click();
  await atTime(1.24);
  await shot("04-editor-fuerzas-activas");
  await atTime(6.03);
  await shot("05-editor-resultado");
  const finalStatus=await page.locator("#physics-status").innerText();
  if(!finalStatus.includes("Secuencia múltiple completada"))
    throw Error("Editor did not complete: "+finalStatus);
  if(issues.length)throw Error("Browser errors: "+issues.join(";"));
  Object.assign(result,{verdict:"ok",stepStatus,stepDetails,finalStatus,errors:issues});
  console.log("INTEGRATION-BROWSER-SMOKE "+JSON.stringify(result));
}catch(error){
  Object.assign(result,{verdict:"failed",error:String(error),errors:issues});
  console.error("INTEGRATION-BROWSER-SMOKE "+JSON.stringify(result));
  process.exitCode=1;
}finally{
  await writeFile(path.join(output,"results.json"),JSON.stringify(result,null,2));
  const video=page.video();
  await page.close();await context.close();
  if(video)await rename(await video.path(),path.join(output,"integrated-step-and-force-editor.webm"));
  await browser.close();
}
