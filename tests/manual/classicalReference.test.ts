/**
 * C0 reference reproduction, separate from the HPL controller (#114).
 * Runs the pinned upstream SIMBICON browser demo on its original Ammo/Bullet
 * physics and 0.00025 s dt. No gains, model, solver, initial pose or control
 * callbacks are changed in the .00025 s baseline; the paired .002 s case
 * changes only the physics/control timestep. Wall-clock rendering/scheduling is replaced
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
      if(world.dt!==window.hplReferenceDt||world.is2D!==true)throw new Error("Unexpected reference timestep or plane");
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
      var initialization={cachedFootOrientation:Array.from(cached.orientation),bulletFootPose:read("lFoot"),bulletPelvisPose:initial};
      // Read the compiled physical parameters, not just the authored JSON.
      // Older Ammo builds omit some accessors; report null, never invent a value.
      function get(obj,method){
        try{return typeof obj[method]==="function"?obj[method]():null;}catch(e){return null;}
      }
      function vector(obj){
        return obj&&typeof obj.x==="function"?[obj.x(),obj.y(),obj.z()]:null;
      }
      var bodies=names.map(function(name){
        var b=world.simulator.entities["human."+name].body;
        var s=get(b,"getCollisionShape"),inv=get(b,"getInvMass"),iv=vector(get(b,"getInvInertiaDiagLocal"));
        return {name:name,authored:human.parts[name],pose:read(name),massKg:inv>0?1/inv:null,
          inertiaKgM2:iv?iv.map(function(x){return x>0?1/x:null;}):null,
          friction:get(b,"getFriction"),restitution:get(b,"getRestitution"),
          linearDamping:get(b,"getLinearDamping"),angularDamping:get(b,"getAngularDamping"),
          linearFactor:vector(get(b,"getLinearFactor")),angularFactor:vector(get(b,"getAngularFactor")),
          shape:s?{margin:get(s,"getMargin"),radius:get(s,"getRadius"),halfHeight:get(s,"getHalfHeight"),
            halfExtents:vector(get(s,"getHalfExtentsWithMargin"))}:null};
      });
      var jointNames=Object.keys(human.joints).sort();
      var joints=jointNames.map(function(name){
        var record=world.simulator.joints["human."+name],j=record&&record.joint;
        return {name:name,authored:human.joints[name],initialAngle:j&&typeof j.angle==="number"?j.angle:null,
          limitNm:j&&typeof j.torqueLimit==="number"?j.torqueLimit:j&&typeof j._torqueLimit==="number"?j._torqueLimit:null};
      });
      var peakJointNm={},torqueSamples=0;
      function observeNextTorques(){
        jointNames.forEach(function(name){
          var record=world.simulator.joints["human."+name],j=record&&record.joint;
          if(!j||typeof j.getLimitedTorque!=="function")return;
          var v=j.getLimitedTorque();
          if(!v||v.length!==3||!Array.from(v).every(Number.isFinite))return;
          var nm=Math.hypot(v[0],v[1],v[2]);
          peakJointNm[name]=Math.max(peakJointNm[name]||0,nm);
        });
        torqueSamples++;
      }
      var qualified=0,consecutive=0,longestConsecutive=0,flight=false,fallS=null,maxTilt=0,steps=Math.round(12/world.dt),nonfinite=null;
      var every=Math.round(.04/world.dt);
      if(Math.abs(every*world.dt-.04)>1e-10)throw new Error("Incompatible reference sample rate");
      function sample(i){
        var pose=names.map(read);
        frames.push({t:i*world.dt,phase:window.hplReadPhase(),pose:pose,clearance:[clearance("lFoot"),clearance("rFoot")]});
      }
      sample(0);
      var i;
      for(i=1;i<=steps;i++){
        try{
          // Observe the bounded commands about to be consumed by the next
          // physics step. The upstream method is a read-only getter.
          observeNextTorques();
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
            if(landing){
              if(accepted){qualified++;consecutive++;longestConsecutive=Math.max(longestConsecutive,consecutive);}
              else consecutive=0;
              flight=false;
            }
            phase=next;
          }
          if(i%every===0||i===steps)sample(i);
          if(fallS!==null&&i*world.dt>=fallS+.8)break;
        }catch(e){nonfinite=String(e);break;}
      }
      var last=frames[frames.length-1],lastPelvis=last.pose[names.indexOf("pelvis")];
      report({status:"ok",dt:world.dt,planar:world.is2D,partNames:names,frames:frames,transitions:transitions,initialization:initialization,modelAudit:{bodies:bodies,joints:joints},torqueAudit:{peakJointNm:peakJointNm,samples:torqueSamples},
        metrics:{executedSteps:Math.min(i,steps),simulatedS:Math.min(i,steps)*world.dt,qualifiedLandings:qualified,
          longestConsecutive:longestConsecutive,fallS:fallS,nonfinite:nonfinite,maxTiltDeg:maxTilt,forwardM:lastPelvis[0]-initial[0],
          massKg:names.reduce(function(s,n){return s+human.parts[n].mass;},0),
          twentyLandings:longestConsecutive>=20&&fallS===null&&nonfinite===null}});
    }catch(e){report({status:"error",message:String(e),stack:String(e.stack||"")});}
  };
})();
`;

interface ReferenceRun {
  status:string; message?:string; stack?:string; dt?:number; planar?:boolean;
  partNames?:string[]; frames?:Array<{t:number;phase:number;pose:number[][];clearance:number[]}>;
  transitions?:unknown[];modelAudit?:unknown;torqueAudit?:unknown;metrics?:{executedSteps:number;simulatedS:number;qualifiedLandings:number;longestConsecutive:number;fallS:number|null;nonfinite:string|null;maxTiltDeg:number;forwardM:number;massKg:number;twentyLandings:boolean};
}
describe("Pinned upstream SIMBICON planar reference (not HPL 3D validation)",()=>{
  it.skipIf(!enabled).each([.00025,.002])("executes the pinned model and controller at dt=%s",referenceDt=>{
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
      console.log("CLASSICAL_REFERENCE_ENV "+JSON.stringify({sha:SHA,referenceDt,browser:run(chrome,["--version"]).trim(),blobs:pinnedFiles}));
      const original=readFileSync(join(checkout,"static/simbicon.js"),"utf8");
      const phaseDecl="var phase = 0;";
      if(original.split(phaseDecl).length!==2)throw new Error("Unexpected source phase declaration");
      // Copy, never edit the pinned source; expose a read-only phase probe.
      const dtDecl="var dt = 0.00025;";
      if(original.split(dtDecl).length!==2)throw new Error("Unexpected source timestep");
      const source=referenceDt===.00025?original:original.replace(dtDecl,"var dt = "+referenceDt+";");
      writeFileSync(join(checkout,"static/hpl-reference-simbicon.js"),source.replace(phaseDecl,phaseDecl+"\nwindow.hplReadPhase = function(){return phase;};"));
      writeFileSync(join(checkout,"static/hpl-reference-probe.js"),"window.hplReferenceDt="+referenceDt+";\n"+browserProbe);
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
      console.log("CLASSICAL_REFERENCE "+JSON.stringify({...result,frames:undefined,modelAudit:undefined}));
      if(referenceDt===.00025) console.log("CLASSICAL_REFERENCE_MODEL "+JSON.stringify(result.modelAudit));
      expect(result.status,result.message+" "+result.stack).toBe("ok");
      expect(result.dt).toBe(referenceDt);expect(result.planar).toBe(true);
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
      for(let i=0;i<count;i++)console.log("CLASSICAL_REFERENCE_REPLAY "+JSON.stringify({dt:referenceDt,index:i,count,sha256:hash,data:packed.slice(i*chunkSize,(i+1)*chunkSize)}));
      // Physical failures are preserved as diagnostic evidence, not made
      // green by changing the model or thresholds.
    }finally{rmSync(root,{recursive:true,force:true});}
  },300000);
});
