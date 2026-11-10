import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { rootTiltDegrees } from "../../src/physics/standingPush";
import { estimateContactPatchSlip, type MeasuredFootPose } from "../../src/physics/contactPatchSlip";
import type { GroundSupportObservation } from "../../src/control/footContacts";

/** Opt-in long MuJoCo WASM diagnostics.
 * Run locally: HPL_RUN_V3_BENCH=1 npx vitest run tests/manual/articulatedForefootV3Bench.test.ts
 * This emits physical results but does not reinterpret a FALL/ABORT as success.
 * CI does not spend credits on a 10s + sweep campaign by default.
 */
const runBench=process.env.HPL_RUN_V3_BENCH==="1";
const keys=["left.rearfoot","left.forefoot","right.rearfoot","right.forefoot"] as const;
type Key=typeof keys[number];
type Side="left"|"right";
type Part="rearfoot"|"forefoot";
const round=(n:number|null,digits=4)=>n===null ? null : +n.toFixed(digits);

function segmentIds(sim:MujocoSimulation):Readonly<Record<Key,number>> {
  const poses=new Map(sim.geometryPoses().map(p=>[p.id,p]));
  const boxes=sim.geometrySpecs().filter(g=>g.shape==="box");
  const pick=(halfX:number,side:Side):number=>{
    const matches=boxes.filter(g=>
      Math.abs(g.size[0]-halfX)<1e-7 &&
      (side==="left" ? poses.get(g.id)!.position[1]>0 : poses.get(g.id)!.position[1]<0));
    if(matches.length!==1) throw new Error("Geometría v3 no identificable: "+side+" "+halfX);
    return matches[0]!.id;
  };
  return {
    "left.rearfoot":pick(0.0825,"left"),
    "left.forefoot":pick(0.0525,"left"),
    "right.rearfoot":pick(0.0825,"right"),
    "right.forefoot":pick(0.0525,"right"),
  };
}

class MaterialSlipMeter {
  private readonly ids:Readonly<Record<Key,number>>;
  private previous=new Map<Key,MeasuredFootPose>();
  private previouslyLoaded=new Map<Key,boolean>();
  private previousTime:number|null=null;
  readonly samples=Object.fromEntries(keys.map(k=>[k,0])) as Record<Key,number>;
  readonly estimatedSlipM=Object.fromEntries(keys.map(k=>[k,0])) as Record<Key,number>;
  readonly peakSpeedMps=Object.fromEntries(keys.map(k=>[k,0])) as Record<Key,number>;
  readonly forwardImpulseNs=Object.fromEntries(keys.map(k=>[k,0])) as Record<Key,number>;
  constructor(sim:MujocoSimulation){ this.ids=segmentIds(sim); }

  sample(sim:MujocoSimulation,ground:GroundSupportObservation,timeS:number):void{
    const poses=new Map(sim.geometryPoses().map(p=>[p.id,p]));
    const dt=this.previousTime===null ? 0 : timeS-this.previousTime;
    for(const key of keys){
      const [side,part]=key.split(".") as [Side,Part];
      const segment=ground[side].segments?.[part];
      if(!segment) throw new Error("El benchmark v3 requiere cargas por segmento");
      const p=poses.get(this.ids[key]);
      if(!p) throw new Error("Segmento físico no renderizado");
      const current:MeasuredFootPose={centerWorldM:p.position,rotationWorld:p.rotation};
      const loaded=segment.verticalLoadN>40 && segment.verticalLoadCentroidWorldM!==null;
      // A new contact has no previous loaded material point: do not count
      // pre-impact swing velocity as floor slip.
      if(loaded && this.previouslyLoaded.get(key) && dt>0 && dt<=0.25){
        const measured=estimateContactPatchSlip(
          this.previous.get(key)!,current,segment.verticalLoadCentroidWorldM!,dt);
        this.estimatedSlipM[key]+=measured.estimatedSlipDistanceThisSampleM;
        this.peakSpeedMps[key]=Math.max(this.peakSpeedMps[key],measured.tangentialSpeedMps);
        this.forwardImpulseNs[key]+=segment.tangentialReactionXYN[0]*dt;
        this.samples[key]++;
      }
      this.previous.set(key,current);
      this.previouslyLoaded.set(key,Boolean(loaded));
    }
    this.previousTime=timeS;
  }
  report(){
    return Object.fromEntries(keys.map(k=>[k,{
      loadedSamples:this.samples[k],
      estimatedSlipM:this.samples[k]>0?round(this.estimatedSlipM[k]):null,
      peakMaterialSpeedMps:this.samples[k]>0?round(this.peakSpeedMps[k]):null,
      longitudinalImpulseNs:this.samples[k]>0?round(this.forwardImpulseNs[k]):null,
    }]));
  }
}

function groundedSnapshot(sim:MujocoSimulation){
  const state=sim.state();
  const ground=sim.humanoidGroundSupportObservation();
  return {state,ground,tilt:rootTiltDegrees(state.qpos.slice(0,7))};
}
async function standTrial(xml:string){
  const sim=await MujocoSimulation.create(xml);
  try {
    sim.enableExperimentalStanding();
    const meter=new MaterialSlipMeter(sim);
    let minPelvisM=Infinity,maxTiltDeg=0,loadedDurationS=0,settledLoadSamples=0;
    let minLoadAfter1s=Infinity;
    for(let n=0;n<500;n++){
      sim.step(10);
      const {state,ground,tilt}=groundedSnapshot(sim);
      const dt=0.02;
      minPelvisM=Math.min(minPelvisM,state.qpos[2]!);
      maxTiltDeg=Math.max(maxTiltDeg,tilt);
      if(ground.left.verticalLoadN>40 && ground.right.verticalLoadN>40)loadedDurationS+=dt;
      if(state.time>=1){
        settledLoadSamples++;
        minLoadAfter1s=Math.min(minLoadAfter1s,ground.totalVerticalLoadN);
      }
      meter.sample(sim,ground,state.time);
    }
    const {state,ground}=groundedSnapshot(sim);
    return {
      timeS:round(state.time),pelvisEndZ:round(state.qpos[2]!),
      pelvisMinZ:round(minPelvisM),maxRootTiltDeg:round(maxTiltDeg),
      bilateralLoadedDurationS:round(loadedDurationS),
      totalLoadN:round(ground.totalVerticalLoadN),
      minTotalLoadAfter1sN:settledLoadSamples?round(minLoadAfter1s):null,
      segmentEndLoadsN:{
        leftRear:round(ground.left.segments!.rearfoot.verticalLoadN),
        leftToe:round(ground.left.segments!.forefoot.verticalLoadN),
        rightRear:round(ground.right.segments!.rearfoot.verticalLoadN),
        rightToe:round(ground.right.segments!.forefoot.verticalLoadN),
      },
      segmentMaterialSlip:meter.report(),
      // Engineering diagnostic only; cannot certify humanlike stance.
      standingCandidate:minPelvisM>0.65 && maxTiltDeg<45 && loadedDurationS>=9,
    };
  } finally { sim.dispose(); }
}
async function stepTrial(xml:string,side:Side,friction:number){
  const sim=await MujocoSimulation.create(xml);
  try {
    sim.enableExperimentalStanding();
    sim.enableExperimentalStepTrial(side,[0.12,0],0.8,0.52,1,0.48);
    const meter=new MaterialSlipMeter(sim);
    let peakForefootLoadN=0,peakForefootFraction=0;
    let maxLoadedCentroidForwardM=Number.NEGATIVE_INFINITY;
    let bilateralLoadedAfterLandingS=0;
    let minPelvisHeightM=Infinity;
    let maxRootTiltDeg=0;
    let firstFallTimeS:number|null=null;
    const phaseChanges:Array<{
      timeS:number;phase:string;pelvisHeightM:number;rootTiltDeg:number;
      leftRearN:number;leftToeN:number;rightRearN:number;rightToeN:number;
      supportToeFraction:number|null;
    }>=[];
    let lastPhase="";
    for(let n=0;n<300;n++){
      sim.step(10);
      const ground=sim.humanoidGroundSupportObservation();
      const state=sim.state();
      const support=side==="left"?"right":"left";
      const f=ground[support];
      const toe=f.segments!.forefoot;
      peakForefootLoadN=Math.max(peakForefootLoadN,toe.verticalLoadN);
      if(f.verticalLoadN>40){
        peakForefootFraction=Math.max(peakForefootFraction,toe.verticalLoadN/f.verticalLoadN);
      }
      if(f.verticalLoadCentroidXY && f.verticalLoadN>40) {
        maxLoadedCentroidForwardM=Math.max(maxLoadedCentroidForwardM,f.verticalLoadCentroidXY[0]);
      }
      const snapshot=sim.experimentalStepStatus()!;
      const tiltDeg=rootTiltDegrees(state.qpos.slice(0,7));
      minPelvisHeightM=Math.min(minPelvisHeightM,state.qpos[2]!);
      maxRootTiltDeg=Math.max(maxRootTiltDeg,tiltDeg);
      if(snapshot.phase==="FALL" && firstFallTimeS===null)firstFallTimeS=state.time;
      if(snapshot.phase!==lastPhase){
        lastPhase=snapshot.phase;
        phaseChanges.push({
          timeS:round(state.time)!,phase:snapshot.phase,
          pelvisHeightM:round(state.qpos[2]!)!,
          rootTiltDeg:round(tiltDeg)!,
          leftRearN:round(ground.left.segments!.rearfoot.verticalLoadN)!,
          leftToeN:round(ground.left.segments!.forefoot.verticalLoadN)!,
          rightRearN:round(ground.right.segments!.rearfoot.verticalLoadN)!,
          rightToeN:round(ground.right.segments!.forefoot.verticalLoadN)!,
          supportToeFraction:f.verticalLoadN>40 ?
            round(toe.verticalLoadN/f.verticalLoadN) : null,
        });
      }
      if(snapshot.stepContactConfirmed && ground.left.verticalLoadN>40 &&
          ground.right.verticalLoadN>40)bilateralLoadedAfterLandingS+=0.02;
      meter.sample(sim,ground,state.time);
    }
    const s=sim.experimentalStepStatus()!;
    return {
      side,friction,phase:s.phase,stepContactConfirmed:s.stepContactConfirmed,
      targetErrorM:round(s.targetErrorXYM),
      soleMarkerLandingSlipM:round(s.maxLandingSlipM),
      settledDurationS:round(s.settledDurationS),
      peakSupportToeLoadN:round(peakForefootLoadN),
      peakSupportToeLoadFraction:round(peakForefootFraction),
      maxLoadedCentroidForwardWorldX:round(Number.isFinite(maxLoadedCentroidForwardM) ?
        maxLoadedCentroidForwardM:null),
      bilateralLoadedAfterLandingS:round(bilateralLoadedAfterLandingS),
      minPelvisHeightM:round(minPelvisHeightM),
      maxRootTiltDeg:round(maxRootTiltDeg),
      firstFallTimeS:round(firstFallTimeS),
      phaseChanges,
      segmentMaterialSlip:meter.report(),
    };
  } finally { sim.dispose(); }
}

describe.skipIf(!runBench)("Manual articulated forefoot v3: solved 10s stance + contact sweep",()=>{
  it("reports v3 stance without manufacturing a pass",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const result=await standTrial(xml);
    console.log("V3-10S-STANDING "+JSON.stringify(result));
    expect(result.timeS).toBeCloseTo(10,6);
    expect([result.pelvisEndZ,result.pelvisMinZ,result.maxRootTiltDeg]
      .every(x=>x!==null && Number.isFinite(x))).toBe(true);
    // Do NOT assert standingCandidate: measured failures must be reported.
  },180000);
  it("reports both legs, friction 0.9/1.0/1.1 and segment-local slip",async()=>{
    const original=await readFile(
      new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const rows=[];
    for(const friction of [0.9,1,1.1]){
      const xml=original.replace(/friction="1.0 0.025 0.002"/g,
        `friction="${friction} 0.025 0.002"`);
      for(const side of ["left","right"] as const){
        const row=await stepTrial(xml,side,friction);
        console.log("V3-STEP-SEGMENTED "+JSON.stringify(row));
        rows.push(row);
      }
    }
    expect(rows).toHaveLength(6);
    expect(rows.every(r=>r.phase==="COMPLETE" || r.phase==="ABORT" || r.phase==="FALL" ||
      ["STAND","SHIFT_WEIGHT","LIFT","SWING","LAND","RECOVER"].includes(r.phase)))
      .toBe(true);
    // No relabeling of ABORT/FALL and no changes to the existing FSM gates.
  },300000);
});
