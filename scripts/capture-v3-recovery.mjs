import { chromium } from "playwright";
import { existsSync } from "node:fs";
import { mkdir, rename, writeFile } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import path from "node:path";

const out = path.resolve("media/v3-push-recovery");
await mkdir(out,{recursive:true});
const executablePath = [
  process.env.CHROME_EXECUTABLE || "",
  "/usr/bin/google-chrome",
  "/usr/bin/google-chrome-stable",
  "/usr/bin/chromium",
].find(p => p && existsSync(p));
const browser=await chromium.launch({
  headless:true,...(executablePath ? {executablePath} : {}),
  args:["--no-sandbox","--disable-dev-shm-usage",
    "--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"],
});
const url="http://127.0.0.1:4173/humanoid-physics-lab/";
const cases=[
  {name:"v3-frontal-120N-estable",direction:"forward",peakN:120},
  {name:"v3-frontal-200N-caida",direction:"forward",peakN:200},
  {name:"v3-lateral-300N-caida",direction:"left",peakN:300},
];
const results=[];
async function waitPhysicalTime(page,minTimeS){
  await page.waitForFunction(t=>{
    const s=document.querySelector("#physics-readout")?.textContent||"";
    const m=s.match(/t\\s*=\\s*([\\d.]+)\\s*s/);
    return m && Number(m[1])>=t;
  },minTimeS,{timeout:120000,polling:"raf"});
}
async function record(c){
  const dir=path.join(out,c.name);
  await mkdir(dir,{recursive:true});
  const context=await browser.newContext({
    viewport:{width:1400,height:820},deviceScaleFactor:1,
    recordVideo:{dir,size:{width:1400,height:820}},
  });
  const page=await context.newPage();
  const errors=[];
  page.on("pageerror",e=>errors.push(e.message));
  let outcome={name:c.name,peakN:c.peakN,direction:c.direction,durationS:0.16};
  try{
    await page.goto(url,{waitUntil:"domcontentloaded",timeout:30000});
    await page.locator("#physics-model").selectOption("standing");
    await page.locator("#physics-load").click();
    await page.waitForFunction(()=>
      document.querySelector("#v3-push-run")?.disabled===false,
      undefined,{timeout:60000});
    if(await page.locator("#physics-viewport canvas").count()===0){
      throw new Error("Sin canvas WebGL: "+await page.locator("#physics-status").innerText());
    }
    await page.locator("#push-direction").selectOption(c.direction);
    await page.locator("#push-peak").fill(String(c.peakN));
    await page.locator("#push-duration").fill("160");
    await page.locator("#v3-balance-mode").selectOption("whole-body-step");
    await page.evaluate((label)=>{
      const mark=document.createElement("div");
      mark.textContent=label;
      mark.style.cssText=[
        "position:fixed","top:12px","left:12px","z-index:99999",
        "background:rgba(0,0,0,.8)","color:#fff","padding:12px 20px",
        "font:700 21px Arial,sans-serif","border-radius:9px",
        "pointer-events:none",
      ].join(";");
      document.body.appendChild(mark);
    }, `MuJoCo real · v3 · ${c.peakN} N · ${c.direction}`);
    await page.locator("#v3-push-run").click();
    await page.locator("#physics-viewport").scrollIntoViewIfNeeded();
    await waitPhysicalTime(page,5.96);
    await page.locator("#physics-viewport").screenshot({path:path.join(dir,"resultado-visor.png")});
    await page.screenshot({path:path.join(dir,"resultado-pantalla.png")});
    outcome={
      ...outcome,
      finalStatus:await page.locator("#physics-status").innerText(),
      pushReadout:await page.locator("#push-readout").innerText(),
      stepReadout:await page.locator("#step-readout").innerText(),
      errors,
    };
    console.log("RECORDED-V3-PHYSICS",JSON.stringify(outcome));
  }catch(e){
    outcome={...outcome,error:String(e),errors};
    console.error("RECORDED-V3-ERROR",JSON.stringify(outcome));
    try{await page.screenshot({path:path.join(dir,"error.png")});}catch{}
  }finally{
    const video=page.video();
    await page.close();
    await context.close();
    if(video){
      const old=await video.path();
      const webm=path.join(dir,c.name+".webm");
      await rename(old,webm);
      try {
        execFileSync("ffmpeg",[
          "-hide_banner","-loglevel","error","-y","-i",webm,
          "-c:v","libx264","-preset","veryfast","-crf","28",
          "-pix_fmt","yuv420p","-movflags","+faststart",
          path.join(dir,c.name+".mp4"),
        ],{timeout:90000});
      }catch(error){console.warn("MP4 NOT AVAILABLE",String(error));}
    }
    results.push(outcome);
  }
}
try {
  for(const c of cases)await record(c);
}finally{
  await writeFile(path.join(out,"results.json"),JSON.stringify(results,null,2));
  await browser.close();
}
if(results.some(x=>x.error))process.exitCode=1;
