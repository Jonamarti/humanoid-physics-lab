import { chromium } from "playwright";
import { existsSync } from "node:fs";
import { mkdir, rename, writeFile } from "node:fs/promises";
import path from "node:path";

const out=path.resolve("editor-smoke-media");
await mkdir(out,{recursive:true});
const executablePath=["/usr/bin/google-chrome","/usr/bin/chromium"].find(existsSync);
const browser=await chromium.launch({headless:true,
  ...(executablePath?{executablePath}:{}),
  args:["--no-sandbox","--disable-dev-shm-usage",
    "--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
const ctx=await browser.newContext({
  viewport:{width:1280,height:760},deviceScaleFactor:1,
  recordVideo:{dir:out,size:{width:1280,height:760}},
});
const page=await ctx.newPage();
const failures=[];
page.on("pageerror",e=>failures.push(e.message));
const capture=async name=>{
  await page.locator("#physics-viewport").screenshot({
    path:path.join(out,name+"-visor.png"),
  });
  await page.screenshot({path:path.join(out,name+"-full.png"),fullPage:true});
};
const waitTime=async target=>{
  await page.waitForFunction(t=>{
    const s=document.querySelector("#physics-readout")?.textContent||"";
    const match=s.match(/t\s*=\s*([\d.]+)/);
    return !!match && Number(match[1])>=t;
  },target,{timeout:90000});
};
let report={};
try{
  await page.goto("http://127.0.0.1:4173/humanoid-physics-lab/",{
    waitUntil:"domcontentloaded",timeout:30000});
  await page.locator("#physics-model").selectOption("standing");
  await page.locator("#physics-load").click();
  await page.waitForFunction(()=>document.querySelector("#physics-toggle")?.disabled===false,
    undefined,{timeout:60000});
  if(!(await page.locator("#physics-viewport canvas").count()))throw Error("No canvas");
  await page.locator(".force-editor-toggle input[type=checkbox]").check();
  await page.locator(".advanced-forces").waitFor({state:"visible"});
  await page.getByRole("button",{name:"+ Añadir fuerza"}).click();
  const rows=page.locator(".edited-force-row");
  if(await rows.count()!==2)throw Error("No se añadieron ambas fuerzas");
  await rows.nth(1).locator("input[type=number]").nth(2).fill("160");
  await rows.nth(1).locator("input[type=number]").nth(2).dispatchEvent("change");
  await rows.nth(1).locator("select").nth(1).selectOption("constant");
  await rows.nth(1).locator("input[type=number]").nth(4).fill("0.8");
  await rows.nth(1).locator("input[type=number]").nth(4).dispatchEvent("change");
  await capture("01-editor-dos-fuerzas");
  await page.getByRole("button",{name:"Ejecutar secuencia"}).click();
  await waitTime(1.24);
  await capture("02-fuerzas-activas");
  await waitTime(6.0);
  await capture("03-resultado");
  report={
    verdict:"ok",forceRows:await rows.count(),
    status:await page.locator("#physics-status").innerText(),
    message:await page.locator(".advanced-forces [role=status]").last().innerText().catch(()=>null),
    errors:failures,
  };
  if(failures.length) throw new Error("Errores JS: "+failures.join(" | "));
  if(!(await page.getByRole("button",{name:"+ Añadir fuerza"}).isEnabled()))
    throw new Error("El editor no recuperó edición al finalizar");
}catch(e){
  report={...report,verdict:"failed",error:String(e),errors:failures};
  console.log("EDITOR-SMOKE-ERROR "+JSON.stringify(report));
  process.exitCode=1;
}finally{
  await writeFile(path.join(out,"results.json"),JSON.stringify(report,null,2));
  const video=page.video();
  await page.close();await ctx.close();
  if(video)await rename(await video.path(),path.join(out,"editor-multifuerza.webm"));
  await browser.close();
}
console.log("EDITOR-SMOKE-RESULT "+JSON.stringify(report));
