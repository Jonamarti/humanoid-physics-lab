import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

type Row = {
  side: "left"|"right";
  comFraction: number;
  hold: number;
  phase: string;
  contact: boolean;
  slipCm: number;
  supportSlipCm: number;
  goalErrorCm: number;
  stableS: number;
  minPelvisM: number;
  maxTiltDeg: number;
};

/** Same 75kg model, free pelvis, knee anatomy, physics integrator, action
 * limits, pose and integration length in all runs; only recovery COM
 * placement and grounded horizontal foot impedance differ.
 */
describe("Measured landing COM target vs. load-gated horizontal foot hold",()=>{
  it("compares fall/abort/complete and quantified foot slip without synthetic success",async()=>{
    const xml=await readFile(
      new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const combinations: Array<readonly [number,number]>=[
      [0.5,0],[0.65,0],[0.8,0],[0.9,0],
      [0.65,0.25],[0.8,0.25],[0.8,0.5],[0.9,0.5],
    ];
    const results:Row[]=[];
    for (const side of ["left","right"] as const) {
      const trials=side==="left"?combinations: [[0.5,0],[0.8,0.25]] as const;
      for(const [comFraction,hold] of trials) {
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial(side,[0.12,0],0.8,0.2,1,
            comFraction,hold);
          const state=sim.step(2600);
          const s=sim.experimentalStepStatus()!;
          const row:Row={
            side,comFraction,hold,phase:s.phase,contact:s.stepContactConfirmed,
            slipCm:+(s.maxLandingSlipM*100).toFixed(2),
            supportSlipCm:+(s.maxSupportFootSlipM*100).toFixed(2),
            goalErrorCm:+(s.targetErrorXYM*100).toFixed(2),
            stableS:+s.settledDurationS.toFixed(3),
            minPelvisM:+s.minPelvisM.toFixed(3),
            maxTiltDeg:+s.maxRootTiltDeg.toFixed(1),
          };
          results.push(row);
          expect([...state.qpos,...state.qvel].every(Number.isFinite)).toBe(true);
          expect([row.slipCm,row.supportSlipCm,row.goalErrorCm,
            row.stableS,row.minPelvisM,row.maxTiltDeg].every(Number.isFinite)).toBe(true);
          if(s.phase==="COMPLETE") {
            expect(s.stepContactConfirmed).toBe(true);
            expect(s.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(s.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(s.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
          }
        }finally{sim.dispose();}
      }
    }
    console.log("LANDING-COM-HOLD-MUJOCO "+JSON.stringify(results));
    expect(results).toHaveLength(10);
  },120000);

  it("guards opt-in parameters and preserves legacy default",async()=>{
    const xml=await readFile(
      new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const sim=await MujocoSimulation.create(xml);
    try {
      sim.enableExperimentalStanding();
      expect(()=>sim.enableExperimentalStepTrial("left",[0.12,0],0.8,
        0.2,1,1.01,0)).toThrow(RangeError);
      expect(()=>sim.enableExperimentalStepTrial("left",[0.12,0],0.8,
        0.2,1,0.5,-0.1)).toThrow(RangeError);
      expect(()=>sim.enableExperimentalStepTrial("left",[0.12,0],0.8,
        0.2,1,NaN,0)).toThrow(RangeError);
      expect(()=>sim.enableExperimentalStepTrial("left",[0.12,0],0.8,
        0.2,1,0.5,0)).not.toThrow();
    }finally{sim.dispose();}
  },30000);
});
