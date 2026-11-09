import { chromium } from "playwright";
import { existsSync } from "node:fs";
import { mkdir, rename, writeFile } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import path from "node:path";

const out = path.resolve("media");
await mkdir(out, { recursive: true });
const executablePath = [process.env.CHROME_EXECUTABLE || "",
  "/usr/bin/google-chrome", "/usr/bin/google-chrome-stable", "/usr/bin/chromium"]
  .find(p => p && existsSync(p));
const browser = await chromium.launch({
  headless: true, ...(executablePath ? {executablePath} : {}),
  args: ["--no-sandbox","--disable-dev-shm-usage",
    "--enable-webgl","--use-angle=swiftshader","--enable-unsafe-swiftshader"],
});
const url = "http://127.0.0.1:4173/humanoid-physics-lab/";
const results = [];

async function waitTime(page, minTime) {
  await page.waitForFunction(t => {
    const s = document.querySelector("#physics-readout")?.textContent || "";
    const m = s.match(/t\s*=\s*([\d.]+)\s*s/);
    return m && Number(m[1]) >= t;
  }, minTime, { timeout:120000, polling:"raf" });
}

async function scenario({name,type,direction,peak,duration}) {
  const folder = path.join(out,name);
  await mkdir(folder,{recursive:true});
  const context = await browser.newContext({
    viewport:{width:1280,height:760},deviceScaleFactor:1,
    recordVideo:{dir:folder,size:{width:1280,height:760}},
  });
  const page = await context.newPage();
  const failures = [];
  page.on("pageerror",e=>failures.push(e.message));
  try {
    await page.goto(url,{waitUntil:"domcontentloaded",timeout:30000});
    await page.locator("#physics-model").selectOption("standing");
    await page.locator("#physics-load").click();
    await page.waitForFunction(()=>document.querySelector("#physics-toggle")?.disabled===false,
      undefined,{timeout:60000});
    if (!await page.locator("#physics-viewport canvas").count())
      throw new Error("No se ha renderizado canvas WebGL: "+await page.locator("#physics-status").innerText());
    await page.screenshot({path:path.join(folder,"01-inicio.png")});
    if (type==="push") {
      await page.locator("#push-direction").selectOption(direction);
      await page.locator("#push-peak").fill(String(peak));
      await page.locator("#push-duration").fill(String(duration));
      await page.locator("#push-run").click();
      await waitTime(page,duration>=500 ? 1.2 : 1.06);
      await page.screenshot({path:path.join(folder,"02-pulso.png")});
      await waitTime(page,5.95);
      await page.screenshot({path:path.join(folder,"03-resultado.png")});
    } else {
      await page.locator("#step-foot").selectOption("left");
      await page.locator("#step-direction").selectOption("diagonal");
      await page.locator("#step-run").click();
      await waitTime(page,1.66);
      await page.screenshot({path:path.join(folder,"02-paso.png")});
      await waitTime(page,5.17);
      await page.screenshot({path:path.join(folder,"03-resultado.png")});
    }
    results.push({
      name,status:await page.locator("#physics-status").innerText(),
      step:type==="step" ? await page.locator("#step-readout").innerText():null,
      projection:await page.locator("#ground-projection-readout").innerText(),
      errors:failures
    });
    console.log("CAPTURED",name,JSON.stringify(results.at(-1)));
  } catch(e) {
    results.push({name,error:String(e),
      status:await page.locator("#physics-status").innerText().catch(()=>""),errors:failures});
    console.log("FAILED",name,String(e),JSON.stringify(failures));
  } finally {
    const video = page.video();
    await page.close();
    await context.close();
    if (video) {
      const source = await video.path();
      const dest = path.join(folder,name+".webm");
      await rename(source,dest);
      try {
        execFileSync("ffmpeg",["-hide_banner","-loglevel","error","-y",
          "-i",dest,"-c:v","libx264","-preset","veryfast","-crf","26",
          "-pix_fmt","yuv420p","-movflags","+faststart",path.join(folder,name+".mp4")],
          {timeout:90000});
      } catch(e) { console.warn("MP4 unavailable",String(e)); }
    }
  }
}

try {
  await scenario({name:"empujon-frontal-240N",type:"push",direction:"forward",peak:240,duration:120});
  await scenario({name:"empujon-posterior-240N",type:"push",direction:"backward",peak:240,duration:120});
  await scenario({name:"fuerza-prolongada-120N",type:"push",direction:"forward",peak:120,duration:800});
  await scenario({name:"paso-diagonal-experimental",type:"step"});
} finally {
  await writeFile(path.join(out,"results.json"),JSON.stringify(results,null,2));
  await browser.close();
}
if (results.some(r=>r.error)) process.exitCode=1;
