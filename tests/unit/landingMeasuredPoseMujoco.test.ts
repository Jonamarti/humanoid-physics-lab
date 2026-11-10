import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Measured touchdown-pose retention vs original step in MuJoCo",()=>{
  it("compares joint pose hold without altering free-root physics",async()=>{
    const xml=await readFile(
      new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const scenarios=[
      ...[0,0.25,0.5,0.75,1].map(hold=>({side:"left" as const,step:0.12,servo:0.2,hold})),
      ...[0,0.25,0.5].map(hold=>({side:"left" as const,step:0.10,servo:0.5,hold})),
      ...[0,0.5].map(hold=>({side:"right" as const,step:0.12,servo:0.2,hold})),
    ];
    const rows=[];
    for(const trial of scenarios){
      const sim=await MujocoSimulation.create(xml);
      try{
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial(trial.side,[trial.step,0],0.8,
          trial.servo,1,trial.hold);
        const state=sim.step(2600);
        const s=sim.experimentalStepStatus()!;
        const row={
          ...trial,outcome:s.phase,contact:s.stepContactConfirmed,
          slipCm:+(s.maxLandingSlipM*100).toFixed(2),
          supportSlipCm:+(s.maxSupportFootSlipM*100).toFixed(2),
          goalErrorCm:+(s.targetErrorXYM*100).toFixed(2),
          settledS:+s.settledDurationS.toFixed(3),
          pelvisM:+state.qpos[2]!.toFixed(3),
          gates:s.stability?.failures??[],
        };
        rows.push(row);
        expect([...state.qpos,...state.qvel].every(Number.isFinite)).toBe(true);
        expect([row.slipCm,row.supportSlipCm,row.goalErrorCm,
          row.settledS,row.pelvisM].every(Number.isFinite)).toBe(true);
        if(s.phase==="COMPLETE") {
          expect(s.stepContactConfirmed).toBe(true);
          expect(s.maxLandingSlipM).toBeLessThanOrEqual(0.02);
          expect(s.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
          expect(s.targetErrorXYM).toBeLessThanOrEqual(0.05);
          expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
        }
      }finally{sim.dispose();}
    }
    console.log("LANDING-MEASURED-POSE-MUJOCO "+JSON.stringify(rows));
    expect(rows).toHaveLength(10);
  },120000);
});
