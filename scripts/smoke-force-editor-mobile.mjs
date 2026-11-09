import { chromium } from "playwright";
import { existsSync } from "node:fs";
import { writeFile } from "node:fs/promises";

const executablePath=["/usr/bin/google-chrome","/usr/bin/chromium"].find(existsSync);
const browser=await chromium.launch({headless:true,
  ...(executablePath?{executablePath}:{}),
  args:["--no-sandbox","--disable-dev-shm-usage",
    "--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"]});
const ctx=await browser.newContext({
  viewport:{width:390,height:844},isMobile:true,hasTouch:true,
  deviceScaleFactor:1,
});
const page=await ctx.newPage();
const errors=[];
page.on("pageerror",e=>errors.push(e.message));
let result={verdict:"unknown",errors};
try{
  await page.goto("http://127.0.0.1:4173/humanoid-physics-lab/",
    {waitUntil:"domcontentloaded"});
  await page.locator("#physics-model").selectOption("standing");
  await page.locator("#physics-load").click();
  await page.waitForFunction(()=>document.querySelector("#physics-toggle")?.disabled===false,
    undefined,{timeout:60000});
  await page.locator(".force-editor-toggle input").check();
  await page.getByRole("button",{name:"+ Añadir fuerza"}).tap();
  await page.locator(".edited-force-row").nth(1).getByText("Editar esta flecha").tap();
  await page.screenshot({path:"editor-smoke-media/04-movil-390px.png",fullPage:true});
  const sizes=await page.evaluate(()=>({
    viewport:window.innerWidth,
    document:document.documentElement.scrollWidth,
    body:document.body.scrollWidth,
  }));
  result={verdict:"ok",...sizes,forceRows:await page.locator(".edited-force-row").count(),errors};
  if(sizes.document>sizes.viewport+4)throw Error("Contenido desborda anchura móvil");
  if(result.forceRows!==2||errors.length)throw Error("Fallo al editar fuerzas en móvil");
}catch(e){
  result={...result,verdict:"failed",error:String(e),errors};
  process.exitCode=1;
}finally{
  await writeFile("editor-smoke-media/mobile-results.json",JSON.stringify(result,null,2));
  await ctx.close();await browser.close();
}
console.log("EDITOR-MOBILE-SMOKE "+JSON.stringify(result));
