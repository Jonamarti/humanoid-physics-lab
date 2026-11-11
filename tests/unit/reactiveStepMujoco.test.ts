import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { rootTiltDegrees } from "../../src/physics/standingPush";
import { editedForceAtTime, type EditedForce } from "../../src/physics/forceSequence";

const xml=()=>readFile(new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
describe("Opt-in reactive stepping with measured MuJoCo support",()=>{
  it("preserves research settings on reset and repeats the same free-root physical trajectory",async()=>{
    const sim=await MujocoSimulation.create(await xml());
    try {
      sim.enableExperimentalStanding();
      sim.enableExperimentalReactiveRecovery({captureLookaheadS:.2,planningDelayS:.04});
      const pulse:EditedForce={id:"repeat",site:"sternum_impact",forceN:[0,300,0],
        startS:1,durationS:.16,waveform:"half-sine"};
      const run=()=>{
        sim.step(3000,t=>{
          const f=editedForceAtTime(t,pulse);
          if(Math.hypot(...f)>0)sim.applyWorldForceAtSite(pulse.site,f);
        });
        expect(sim.experimentalControlObservation().freeRootApplied).toEqual([0,0,0,0,0,0]);
        expect(sim.experimentalControlObservation().pelvisAppliedWrench).toEqual([0,0,0,0,0,0]);
        return {state:sim.state(),status:sim.experimentalReactiveRecoveryStatus()};
      };
      const first=run();sim.reset();const second=run();
      expect(second).toEqual(first);
      expect(first.status!.lastIntent!.timeS).toBeLessThan(1.2);
      expect(first.status!.phase).toBe("FALL");
    }finally{sim.dispose();}
  },60000);
  it("is v3 only and remains at MONITORING during nominal unperturbed standing",async()=>{
    const v1Text=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const [v1,sim]=await Promise.all([
      MujocoSimulation.create(v1Text),MujocoSimulation.create(await xml()),
    ]);
    try {
      v1.enableExperimentalStanding();
      expect(()=>v1.enableExperimentalReactiveRecovery()).toThrow();
      expect(v1.experimentalReactiveRecoveryStatus()).toBeNull();
      sim.enableExperimentalStanding();
      expect(sim.experimentalReactiveRecoveryStatus()).toBeNull();
      sim.enableExperimentalReactiveRecovery();
      sim.step(500); // first second nominal stance
      const a=sim.experimentalReactiveRecoveryStatus()!;
      expect(a.attempts).toBe(0);
      expect(a.phase).toBe("MONITORING");
      expect(sim.experimentalStepStatus()).toBeNull();
      expect(sim.experimentalWholeBodyBalanceStatus()).not.toBeNull();
      sim.reset();
      expect(sim.experimentalReactiveRecoveryStatus()!.attempts).toBe(0);
      expect(sim.experimentalStepStatus()).toBeNull();
      sim.step(200);
      expect(sim.experimentalReactiveRecoveryStatus()!.attempts).toBe(0);
      sim.disableExperimentalReactiveRecovery();
      expect(sim.experimentalReactiveRecoveryStatus()).toBeNull();
      expect(sim.experimentalWholeBodyBalanceStatus()).not.toBeNull();
    }finally{v1.dispose();sim.dispose();}
  },60000);
  it("applies a diagonal force sequence without fabricated step success or repeated attempts",async()=>{
    const sim=await MujocoSimulation.create(await xml());
    const forces:readonly EditedForce[]=[
      {id:"back",site:"sternum_impact",forceN:[-370,30,0],
        startS:.8,durationS:.18,waveform:"half-sine"},
      {id:"side",site:"left_shoulder_impact",forceN:[0,70,0],
        startS:1,durationS:.10,waveform:"half-sine"},
    ];
    try{
      sim.enableExperimentalStanding();
      sim.enableExperimentalReactiveRecovery();
      let peakTilt=0,maxAttempts=0;
      for(let i=0;i<300;i++){
        sim.step(10,t=>{
          for(const f of forces){
            const force=editedForceAtTime(t,f);
            if(force.some(x=>x!==0)) sim.applyWorldForceAtSite(f.site,force);
          }
        });
        const state=sim.state();
        peakTilt=Math.max(peakTilt,rootTiltDegrees(state.qpos));
        maxAttempts=Math.max(maxAttempts,sim.experimentalReactiveRecoveryStatus()!.attempts);
      }
      const reactive=sim.experimentalReactiveRecoveryStatus()!;
      const physical=sim.experimentalStepStatus();
      expect(sim.state().time).toBeCloseTo(6,6);
      expect(reactive.attempts).toBeLessThanOrEqual(1);
      expect(maxAttempts).toBeLessThanOrEqual(1);
      expect(Number.isFinite(peakTilt)).toBe(true);
      expect(sim.state().qpos.every(Number.isFinite)).toBe(true);
      if(reactive.attempts===1){
        expect(reactive.lastIntent).not.toBeNull();
        expect(physical).not.toBeNull();
        expect(Math.hypot(...reactive.lastIntent!.deltaXY)).toBeLessThanOrEqual(.38);
        if(physical?.phase!=="COMPLETE")
          expect(reactive.phase).not.toBe("STEP_COMPLETE");
      }else{
        expect(physical).toBeNull();
        expect(reactive.phase).not.toBe("STEP_COMPLETE");
      }
      console.log("REACTIVE-DIAGONAL-V3 "+JSON.stringify({
        peakTiltDeg:peakTilt,phase:reactive.phase,
        attempts:reactive.attempts,reason:reactive.reason,
        target:reactive.lastIntent,physicalPhase:physical?.phase,
        stepContactConfirmed:physical?.stepContactConfirmed??false,
        finalPelvisM:sim.state().qpos[2],
      }));
    }finally{sim.dispose();}
  },120000);
});
