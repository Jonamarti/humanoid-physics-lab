import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import {
  editedForceAtTime, editedSequenceHorizonS,
  type EditedForce,
} from "../../src/physics/forceSequence";

const sequence:readonly EditedForce[]=[
  {id:"f1",site:"sternum_impact",forceN:[-120,0,0],
    startS:0.45,durationS:0.12,waveform:"half-sine"},
  {id:"f2",site:"right_shoulder_impact",forceN:[0,90,0],
    startS:0.65,durationS:0.18,waveform:"half-sine"},
  {id:"f3",site:"pelvis_origin",forceN:[35,35,0],
    startS:0.8,durationS:0.1,waveform:"constant"},
];

describe("Multi-angle force editor routes actual forces through optional v3 balance",()=>{
  it("uses the same three physical pushes while enabling bounded whole-body responses",async()=>{
    const xml=await readFile(
      new URL("../../public/models/humanoid_gait_v3.xml",import.meta.url),"utf8");
    const run=async(wholeBody:boolean)=>{
      const sim=await MujocoSimulation.create(xml);
      try{
        sim.enableExperimentalStanding();
        if(wholeBody)sim.enableExperimentalWholeBodyBalance();
        let active=0,maxArmTarget=0,netForceSamples=0;
        for(let i=0;i<125;i++){
          sim.step(10,t=>{
            for(const force of sequence){
              const v=editedForceAtTime(t,force);
              if(!v.some(x=>x!==0))continue;
              netForceSamples++;
              sim.applyWorldForceAtSite(force.site,v);
            }
          });
          const cmd=sim.experimentalWholeBodyBalanceStatus();
          if(cmd?.active){
            active++;
            maxArmTarget=Math.max(maxArmTarget,
              Math.abs(cmd.targetsRad.left_shoulder_pitch??0),
              Math.abs(cmd.targetsRad.left_shoulder_roll??0));
          }
        }
        const state=sim.state();
        return {state,active,maxArmTarget,netForceSamples,
          ground:sim.humanoidGroundSupportObservation()};
      }finally{sim.dispose();}
    };
    const base=await run(false);
    const adapted=await run(true);
    expect(editedSequenceHorizonS(sequence)).toBe(6);
    expect(base.state.time).toBeCloseTo(2.5,6);
    expect(adapted.state.time).toBeCloseTo(base.state.time,8);
    expect(base.netForceSamples).toBe(adapted.netForceSamples);
    expect(adapted.netForceSamples).toBeGreaterThan(0);
    expect(base.active).toBe(0);
    expect(adapted.active).toBeGreaterThan(0);
    expect(adapted.maxArmTarget).toBeGreaterThan(0.001);
    expect(adapted.maxArmTarget).toBeLessThanOrEqual(0.33);
    expect(adapted.state.qpos.every(Number.isFinite)).toBe(true);
    expect(adapted.ground.totalVerticalLoadN).toBeGreaterThanOrEqual(0);
    // Movement follows the solver, not a hardcoded forced animation.
    const differences=base.state.qpos.map((v,i)=>Math.abs(v-adapted.state.qpos[i]!));
    expect(Math.max(...differences)).toBeGreaterThan(1e-6);
  },60000);
});
