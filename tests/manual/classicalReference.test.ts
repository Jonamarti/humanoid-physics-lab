/**
 * C0 reference reproduction, separate from the HPL controller (#114).
 * Runs the pinned upstream SIMBICON browser demo on its original Ammo/Bullet
 * physics and 0.00025 s dt. No gains, model, solver, initial pose or control
 * callbacks are changed. Only wall-clock rendering/scheduling is replaced
 * with a fixed number of original World.step() calls and read-only probes.
 *
 * External reference diagnostic; not a new dependency or production asset.
 * Runs on the designated draft PR only, or explicitly via HPL_REFERENCE=1.
 */
import { execFileSync } from "node:child_process";
import { existsSync, mkdtempSync, readFileSync, writeFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { pathToFileURL } from "node:url";
import { gzipSync } from "node:zlib";
import { createHash } from "node:crypto";
import { describe,expect,it } from "vitest";

const SHA="ddcb94844d2722e117aa2f07569d0fa356a72776";
const UPSTREAM="https://github.com/mfirmin/SimbiconJS.git";
const enabled=process.env.HPL_REFERENCE==="1" ||
  (process.env.GITHUB_ACTIONS==="true"&&process.env.GITHUB_HEAD_REF==="experiment/classical-c0-simbicon");
const pinnedFiles:Record<string,string>={
  "index.html":"e6a92aac79a78194b4a40823eb0ae677f22cc46e",
  "LICENSE":"afb8d45b7937ce40dfa70c878fd52f2818f1aa4b",
  "static/simbicon.js":"c3de189862c8f630366e20b83f1d450180cb2307",
  "static/human.js":"224a3faa70c59967de53a357ec7d3e42e62fbe82",
  "static/coach.js":"e2d10a97bbc71e3c5fbb91fbca4a25e991773588",
};
const browserProbe=String.raw`
(function(){
  function report(o){
    var p=document.getElementById("hpl-reference-output");
    if(!p){p=document.createElement("pre");p.id="hpl-reference-output";(document.body||document.documentElement).appendChild(p);}
    p.textContent=encodeURIComponent(JSON.stringify(o));
  }
  window.addEventListener("error",function(e){report({status:"error",message:String(e.message),stack:String(e.error&&e.error.stack||"")});});
  if(typeof Coach==="undefined") {report({status:"error",message:"Pinned Coach bundle did not expose Coach"});return;}
  Coach.World.prototype.go=function(options){
    try{
      var world=this;
      if(world.dt!==.00025||world.is2D!==true)throw new Error("Unexpected upstream timestep or plane");
      if(typeof world.step!=="function"||typeof world.simulator.setCallback!=="function")throw new Error("Unexpected upstream scheduler API");
      world.simulator.setCallback(options.simulationCallback);
      var names=Object.keys(human.parts).sort();
      function read(name){
        // Read the rigid-body transform, not Coach's cached entity pose.
        // Before the first step its orientation cache can still be zero.
        var id="human."+name;
        var entity=world.simulator.entities[id];
        if(!entity||!entity.body)throw new Error("Missing Bullet body: "+id);
        var transform=entity.body.getWorldTransform();
        var p=transform.getOrigin(),q=transform.getRotation();
        var pose=[p.x(),p.y(),p.z(),q.w(),q.x(),q.y(),q.z()];
        if(!pose.every(Number.isFinite)||Math.hypot.apply(null,pose.slice(3))<1e-8)
          throw new Error("Invalid Bullet pose: "+id+" "+JSON.stringify(pose));
        return pose;
      }
      function clearance(name){
        var a=read(name),q=a.slice(3),dims=human.parts[name].sides;
        var norm=Math.hypot.apply(null,q);
        if(!(norm>0))throw new Error("Invalid foot quaternion");
        q=q.map(function(x){return x/norm;});
        var w=q[0],x=q[1],y=q[2],z=q[3];
        // Lowest point of actual upstream box, Y-up, WXYZ.
        return a[1]-.5*(Math.abs(2*(x*y+w*z))*dims[0]+Math.abs(1-2*(x*x+z*z))*dims[1]+Math.abs(2*(y*z-w*x))*dims[2]);
      }
      var initial=read("pelvis"),frames=[],transitions=[],phase=window.hplReadPhase();
      var cached=world.entities["human.lFoot"];
      var initialization={cachedFootOrientation:cached.orientation,bulletFootPose:read("lFoot"),bulletPelvisPose:initial};
      var qualified=0,flight=false,fallS=null,maxTilt=0,steps=48000,nonfinite=null;
      function sample(i){
        var pose=names.map(read);
        frames.push({t:i*world.dt,phase:window.hplReadPhase(),pose:pose,clearance:[clearance("lFoot"),clearance("rFoot")]});
      }
      sample(0);
      var i;
      for(i=1;i<=steps;i++){
        try{
          world.step();
          var pelvis=read("pelvis"),torso=read("uTorso");
          var q=torso.slice(3),qn=Math.hypot.apply(null,q);if(!(qn>0))throw new Error("Invalid torso quaternion");
          var tilt=Math.acos(Math.max(-1,Math.min(1,1-2*(q[1]*q[1]+q[3]*q[3])/(qn*qn))))*180/Math.PI;
          maxTilt=Math.max(maxTilt,tilt);
          if(fallS===null&&(pelvis[1]<.45||tilt>60))fallS=i*world.dt;
          var foot=phase<2?"rFoot":"lFoot";
          var clear=clearance(foot);
          if(clear>.01)flight=true;
          var next=window.hplReadPhase();
          if(next!==phase){
            var landing=(phase===1&&next===2)||(phase===3&&next===0);
            // Upstream controller transitions only after contactPairTest.
            // Independent geometry check adds observed flight and proximity;
            // it is NOT a normal-force or sustained-load validation.
            var accepted=landing&&flight&&clear<.02&&fallS===null;
            transitions.push({t:i*world.dt,from:phase,to:next,foot:foot,landing:landing,flight:flight,clearanceM:clear,qualified:accepted});
            if(accepted)qualified++;
            if(landing)flight=false;
            phase=next;
          }
          if(i%160===0||i===steps)sample(i);
          if(fallS!==null&&i*world.dt>=fallS+.8)break;
        }catch(e){nonfinite=String(e);break;}
      }
      var last=frames[frames.length-1],lastPelvis=last.pose[names.indexOf("pelvis")];
      report({status:"ok",dt:world.dt,planar:world.is2D,partNames:names,frames:frames,transitions:transitions,initialization:initialization,
        metrics:{executedSteps:Math.min(i,steps),simulatedS:Math.min(i,steps)*world.dt,qualifiedLandings:qualified,
          fallS:fallS,nonfinite:nonfinite,maxTiltDeg:maxTilt,forwardM:lastPelvis[0]-initial[0],
          massKg:names.reduce(function(s,n){return s+human.parts[n].mass;},0),
          twentyLandings:qualified>=20&&fallS===null&&nonfinite===null}});
    }catch(e){report({status:"error",message:String(e),stack:String(e.stack||"")});}
  };
})();
`;

interface ReferenceRun {
  status:string; message?:string; stack?:string; dt?:number; planar?:boolean;
  partNames?:string[]; frames?:Array<{t:number;phase:number;pose:number[][];clearance:number[]}>;
  transitions?:unknown[];metrics?:{executedSteps:number;simulatedS:number;qualifiedLandings:number;fallS:number|null;nonfinite:string|null;maxTiltDeg:number;forwardM:number;massKg:number;twentyLandings:boolean};
}
describe("Pinned upstream SIMBICON planar reference (not HPL 3D validation)",()=>{
  it.skipIf(!enabled)("executes the original demo's model and controller without real-time rendering",()=>{
    const root=mkdtempSync(join(tmpdir(),"hpl-simbicon-reference-"));
    const checkout=join(root,"upstream");
    const run=(cmd:string,args:string[],cwd=root,timeout=60000)=>execFileSync(cmd,args,{cwd,encoding:"utf8",timeout,maxBuffer:32*1024*1024});
    try{
      run("git",["init","--quiet",checkout]);
      run("git",["fetch","--quiet","--depth=1",UPSTREAM,SHA],checkout);
      run("git",["checkout","--quiet","--detach","FETCH_HEAD"],checkout);
      expect(run("git",["rev-parse","HEAD"],checkout).trim()).toBe(SHA);
      for(const [path,blob] of Object.entries(pinnedFiles)){
        expect(run("git",["hash-object",path],checkout).trim(),path).toBe(blob);
      }
      const chrome=[process.env.CHROME_BIN,"/usr/bin/google-chrome","/usr/bin/google-chrome-stable","/usr/bin/chromium"].find((x):x is string=>!!x&&existsSync(x));
      if(!chrome)throw new Error("No Chrome/Chromium installed; use CHROME_BIN");
      console.log("CLASSICAL_REFERENCE_ENV "+JSON.stringify({sha:SHA,browser:run(chrome,["--version"]).trim(),blobs:pinnedFiles}));
      const original=readFileSync(join(checkout,"static/simbicon.js"),"utf8");
      const phaseDecl="var phase = 0;";
      if(original.split(phaseDecl).length!==2)throw new Error("Unexpected source phase declaration");
      // Copy, never edit the pinned source; expose a read-only phase probe.
      writeFileSync(join(checkout,"static/hpl-reference-simbicon.js"),original.replace(phaseDecl,phaseDecl+"\nwindow.hplReadPhase = function(){return phase;};"));
      writeFileSync(join(checkout,"static/hpl-reference-probe.js"),browserProbe);
      const html=readFileSync(join(checkout,"index.html"),"utf8").replace(/^---\r?\n[\s\S]*?\r?\n---\r?\n/,"");
      const script='<script src="static/simbicon.js"></script>';
      if(html.split(script).length!==2)throw new Error("Unexpected demo entry point");
      // Old local bundle is loaded in an isolated temporary browser profile;
      // forbid remote document resource loads. No repository credentials exposed.
      const page=html.replace("<head>",'<head><meta http-equiv="Content-Security-Policy" content="default-src file: data: blob:; script-src file: \'unsafe-inline\' \'unsafe-eval\'; style-src file: \'unsafe-inline\'; connect-src \'none\'">')
        .replace(script,'<script src="static/hpl-reference-probe.js"></script><script src="static/hpl-reference-simbicon.js"></script>');
      const pagePath=join(checkout,"hpl-reference.html");writeFileSync(pagePath,page);
      const out=run(chrome,["--headless=new","--no-sandbox","--disable-dev-shm-usage","--disable-background-networking",
        "--no-first-run","--no-default-browser-check","--allow-file-access-from-files","--use-gl=angle","--use-angle=swiftshader",
        "--enable-unsafe-swiftshader","--user-data-dir="+join(root,"profile"),"--dump-dom","--virtual-time-budget=15000",
        pathToFileURL(pagePath).href],checkout,240000);
      const m=out.match(/<pre id="hpl-reference-output">([^<]+)<\/pre>/);
      if(!m)throw new Error("Reference browser did not emit a result. DOM tail: "+out.slice(-2000));
      const result=JSON.parse(decodeURIComponent(m[1]!)) as ReferenceRun;
      console.log("CLASSICAL_REFERENCE "+JSON.stringify({...result,frames:undefined}));
      expect(result.status,result.message+" "+result.stack).toBe("ok");
      expect(result.dt).toBe(.00025);expect(result.planar).toBe(true);
      const frames=result.frames!;
      expect(frames.length).toBeGreaterThan(0);
      if(result.metrics?.nonfinite===null) expect(frames.length).toBeGreaterThan(1);
      expect(frames.every(f=>f.pose.every(p=>Math.abs(Math.hypot(...p.slice(3))-1)<.01))).toBe(true);
      expect(frames.every(f=>f.pose.every(p=>p.length===7&&p.every(Number.isFinite)))).toBe(true);
      const bytes=Buffer.from(JSON.stringify({partNames:result.partNames,frames}),"utf8");
      const packed=gzipSync(bytes).toString("base64");
      // Chunk for CI log transport. Reconstruction verifies the raw hash.
      const chunkSize=3000;
      const count=Math.ceil(packed.length/chunkSize);
      const hash=createHash("sha256").update(bytes).digest("hex");
      for(let i=0;i<count;i++)console.log("CLASSICAL_REFERENCE_REPLAY "+JSON.stringify({index:i,count,sha256:hash,data:packed.slice(i*chunkSize,(i+1)*chunkSize)}));
      // Physical failures are preserved as diagnostic evidence, not made
      // green by changing the model or thresholds.
    }finally{rmSync(root,{recursive:true,force:true});}
  },300000);
});
