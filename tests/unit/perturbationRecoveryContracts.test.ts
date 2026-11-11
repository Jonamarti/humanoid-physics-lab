import { describe, expect, it } from "vitest";
import { capturePointXY, insideConvexHullXY, type FootGeom, type XY } from "../../src/control/balanceSensors";
import { summarizeGroundSupport } from "../../src/control/footContacts";
import { planRecovery } from "../../src/control/recoveryPlanner";
import { ReactiveStepCoordinator } from "../../src/control/reactiveStepCoordinator";
import { ExperimentalStepSequence, type StepFeedback } from "../../src/control/experimentalStep";
import { controlledStepPosture } from "../../src/control/controlledStepPosture";
import { editedForceAtTime, editedForceImpulseNs, type EditedForce } from "../../src/physics/forceSequence";
import { ankleBalanceTorques } from "../../src/control/standingBaseline";

/** Deterministic contracts, NOT a demonstration of physical walking.
 * Real impulses/contact/knee trajectories are checked separately in MuJoCo.
 */
const R=[1,0,0,0,1,0,0,0,1] as const;
const rear=(y:number):FootGeom=>({
  centerM:[0.0125,y,0.035],rotationWorld:R,halfSizeM:[0.0825,0.055,0.035],
});
const toe=(y:number):FootGeom=>({
  centerM:[0.1475,y,0.035],rotationWorld:R,halfSizeM:[0.0525,0.055,0.035],
});
const ground=()=>{
  const coords=[0.105,-0.105];
  const contacts=coords.flatMap((y,i)=>[
    {geom1:0,geom2:i+1,positionWorldM:[0,y,0] as const,
      contactNormalWorld:[0,0,1] as const,contactNormalForceN:328},
    {geom1:0,geom2:i+3,positionWorldM:[0.16,y,0] as const,
      contactNormalWorld:[0,0,1] as const,contactNormalForceN:40},
  ]);
  return summarizeGroundSupport({
    floorGeomId:0,leftFootGeomId:1,rightFootGeomId:2,
    leftToeGeomId:3,rightToeGeomId:4,
    leftFoot:rear(0.105),rightFoot:rear(-0.105),
    leftToe:toe(0.105),rightToe:toe(-0.105),contacts,
  });
};
const recovery=(velocity:XY, com:XY=[0.025,0])=>({
  comXY:com,comVelocityXY:velocity,comHeightM:0.95,
  unstableDurationS:0.25,ground:ground(),
  excessiveTrunkTilt:false,maxStepReachM:0.34,
});
const feedback=(timeS:number,overrides:Partial<StepFeedback>={}):StepFeedback=>({
  timeS,comWorldM:[0,0,0.95],comVelocityWorldMps:[0,0,0],
  comProjectionInsideLoadedSupport:true,capturePointInsideLoadedSupport:true,
  pelvisHeightM:0.99,rootTiltDeg:1,
  swingSoleWorldM:[0.065,0.105,0.001],
  supportSoleWorldM:[0.065,-0.105,0.001],
  supportLoadN:370,swingLoadN:370,...overrides,
});

describe("Step-recovery physics contracts: impulse, reach and anatomical intent",()=>{
  it("applies a finite-time impulse and does not invent a constant force afterwards",()=>{
    const pulse=(force:number):EditedForce=>({
      id:"sternum",site:"sternum_impact",forceN:[force,0,0],
      startS:1,durationS:0.16,waveform:"half-sine",
    });
    for(const f of [80,160,240,360]){
      const a=pulse(f),b=pulse(-f);
      expect(editedForceImpulseNs(a)[0]).toBeCloseTo(f*0.16*2/Math.PI,9);
      expect(editedForceImpulseNs(b)[0]).toBeCloseTo(-editedForceImpulseNs(a)[0],9);
      expect(editedForceAtTime(0.99,a)).toEqual([0,0,0]);
      expect(editedForceAtTime(1.16,a)).toEqual([0,0,0]);
    }
  });

  it("uses COM velocity / capture point, not displacement or force alone, to trigger a step",()=>{
    const g=ground();
    const still=planRecovery(recovery([0,0]));
    expect(still.decision).toBe("NONE");
    const cp=capturePointXY([0.025,0],[0.9,0],0.95);
    expect(cp[0]).toBeGreaterThan(0.2);
    expect(insideConvexHullXY(cp,g.loadedSupportPolygonXY)).toBe(false);
    const moving=planRecovery(recovery([0.9,0]));
    expect(["SHIFT_WEIGHT","STEP"]).toContain(moving.decision);
    expect(moving.capturePointXY[0]).toBeCloseTo(cp[0],9);
    // A sensor spike shorter than 120 ms does not authorize an automatic step.
    const coordinator=new ReactiveStepCoordinator();
    const sample=(timeS:number,vx:number)=>({
      timeS,comWorldM:[0.025,0,0.95] as const,
      comVelocityWorldMps:[vx,0,0] as const,
      totalMassKg:75,rootTiltDeg:0,pelvisHeightM:0.99,ground:g,
    });
    for(let i=0;i<4;i++)expect(coordinator.observe(sample(i*.02,0.9))).toBeNull();
    expect(coordinator.observe(sample(.08,0))).toBeNull();
    expect(coordinator.snapshot().attempts).toBe(0);
  });

  it("lengthens a feasible forward step as capture displacement grows, but never exceeds joint reach",()=>{
    const speeds=[0.78,0.84,0.90,0.96];
    const lengths=speeds.map(v=>{
      const plan=planRecovery(recovery([v,0]));
      expect(["SHIFT_WEIGHT","STEP"]).toContain(plan.decision);
      expect(plan.targetFootXY).not.toBeNull();
      const swing=ground()[plan.swingFoot!];
      const length=Math.hypot(plan.targetFootXY![0]-swing.centerXY[0],
        plan.targetFootXY![1]-swing.centerXY[1]);
      expect(length).toBeLessThanOrEqual(0.34+1e-8);
      return length;
    });
    for(let i=1;i<lengths.length;i++)
      expect(lengths[i]!).toBeGreaterThanOrEqual(lengths[i-1]!-1e-7);
    // Severe capture displacement is not a magically reachable infinite stride.
    expect(planRecovery(recovery([4,0])).decision).toBe("FALL");
  });

  it.each([
    ["forward",[0.9,0] as XY,0,1],
    ["backward",[-0.9,0] as XY,0,-1],
    ["left",[0,0.9] as XY,1,1],
    ["right",[0,-0.9] as XY,1,-1],
  ])("proposes a %s interception toward COM motion, or explicitly reports it unreachable",
    (_name,velocity,axis,sign)=>{
      const plan=planRecovery(recovery(velocity));
      expect(["SHIFT_WEIGHT","STEP","FALL"]).toContain(plan.decision);
      if(plan.targetFootXY){
        const delta:XY=[
          plan.targetFootXY[0]-ground()[plan.swingFoot!].centerXY[0],
          plan.targetFootXY[1]-ground()[plan.swingFoot!].centerXY[1],
        ];
        expect(delta[axis]*sign).toBeGreaterThan(0);
        expect(delta[0]*velocity[0]+delta[1]*velocity[1]).toBeGreaterThan(0);
        expect(Math.hypot(...delta)).toBeLessThanOrEqual(0.34+1e-8);
        const g=ground();
        const targetY=plan.targetFootXY[1];
        if(plan.swingFoot==="left")
          expect(targetY).toBeGreaterThanOrEqual(g.right.centerXY[1]+0.04-1e-8);
        else expect(targetY).toBeLessThanOrEqual(g.left.centerXY[1]-0.04+1e-8);
      }else{
        expect(plan.decision).toBe("FALL");
      }
    });

  it("bends the swing knee to shorten the leg, then extends before loaded impact",()=>{
    for(const side of ["left","right"] as const){
      for(const displacement of [[0.12,0],[-0.12,0],[0,side==="left"?0.15:-0.15]] as XY[]){
        const lift=controlledStepPosture({phase:"LIFT",elapsedS:0.30,
          swingSide:side,targetDeltaXY:displacement,intensity:1});
        const swing=controlledStepPosture({phase:"SWING",elapsedS:0.30,
          swingSide:side,targetDeltaXY:displacement,intensity:1,
          swingExtensionGuard:0.48,swingProgress01:0.8});
        const landing=controlledStepPosture({phase:"LAND",elapsedS:0.20,
          swingSide:side,targetDeltaXY:displacement,intensity:1});
        expect(lift.targetsRad[side+"_knee"]).toBeLessThan(-0.9);
        expect(swing.targetsRad[side+"_knee"]).toBeGreaterThan(lift.targetsRad[side+"_knee"]!);
        // Knee still flexed at the planned landing, never locked straight.
        expect(landing.targetsRad[side+"_knee"]).toBeLessThan(-0.08);
        expect(landing.targetsRad[side+"_knee"]).toBeGreaterThan(-0.6);
        if(displacement[0]>0)expect(lift.targetsRad[side+"_hip_pitch"]).toBeLessThan(0);
        if(displacement[0]<0)expect(lift.targetsRad[side+"_hip_pitch"]).toBeGreaterThan(0);
        if(displacement[1]!==0)
          expect(swing.targetsRad[side+"_hip_roll"]!*displacement[1]).toBeGreaterThan(0);
      }
    }
  });

  it("keeps lateral ankle torque available even when walking intent is sagittal",()=>{
    const gains={kpX:1200,kdX:300,kpY:1200,kdY:300,ankleLimitNm:160};
    const centered=ankleBalanceTorques([0,0],[0,0],[0,0],[0,0],75,gains);
    const left=ankleBalanceTorques([0,0.06],[0,0.05],[0,0],[0,0],75,gains);
    const right=ankleBalanceTorques([0,-0.06],[0,-0.05],[0,0],[0,0],75,gains);
    expect(centered[1]).toBeCloseTo(0,12);
    expect(left[1]).toBeLessThan(0);
    expect(right[1]).toBeGreaterThan(0);
  });

  it("does not declare a step before measured unload, knee clearance and sustained landing",()=>{
    const seq=new ExperimentalStepSequence("left",[0.12,0],0.8,1,0.48,true,false,0.115);
    seq.observe(feedback(.8));
    expect(seq.snapshot().phase).toBe("SHIFT_WEIGHT");
    // Even though the controller can command its foot forward, 75% support
    // is not enough to release the v3 articulated foot.
    seq.observe(feedback(1.10,{supportLoadN:555,swingLoadN:185}));
    seq.observe(feedback(1.30,{supportLoadN:555,swingLoadN:185}));
    expect(seq.snapshot().phase).toBe("SHIFT_WEIGHT");
    expect(seq.snapshot().stepContactConfirmed).toBe(false);
    // Require 80% sustained solved contact and a physically unloaded foot.
    for(let i=0;i<10;i++)
      seq.observe(feedback(1.32+i*.02,{supportLoadN:625,swingLoadN:105}));
    expect(seq.snapshot().phase).toBe("LIFT");
    seq.observe(feedback(1.54,{supportLoadN:735,swingLoadN:0,
      swingSoleWorldM:[.065,.105,.05]}));
    expect(seq.snapshot().phase).toBe("SWING");
    expect(seq.snapshot().stepContactConfirmed).toBe(false);
    const cmd=seq.command(1.56,[0,0]);
    expect(cmd.soleTargetWorldM).not.toBeNull();
    expect(Object.keys(cmd.targetsRad).every(k=>!k.includes("root"))).toBe(true);
    // Landing is not inferred from task-space target or time alone.
    seq.observe(feedback(1.68,{supportLoadN:730,swingLoadN:0,
      swingSoleWorldM:[.125,.105,.05]}));
    expect(seq.snapshot().phase).toBe("LAND");
    seq.observe(feedback(1.80,{supportLoadN:740,swingLoadN:0,
      swingSoleWorldM:[.175,.105,.003]}));
    expect(seq.snapshot().stepContactConfirmed).toBe(false);
    expect(seq.snapshot().phase).toBe("LAND");
  });

  it.todo("after a genuinely stabilized loaded landing, a still-escaping capture point may request step #2");
  it.todo("after a successful second stance, return to neutral or alternate a third stride with toe push-off");
  it.todo("confirm actual forefoot/heel slip, knee torque and joint-stop loads during a physically landed step");
});
