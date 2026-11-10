import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { STEP_STABILITY_CHECKS, type StepStabilityCheck } from "../../src/control/stepStabilityDiagnostics";

describe("Measured blockers of physical landing in nominal gait trials",()=>{
  it("counts which real stability gates fail during landing and recovery",async()=>{
    const xml=await readFile(
      new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const cases=[
      {side:"left" as const,step:0.12,servo:0.2},
      {side:"right" as const,step:0.12,servo:0.2},
      {side:"left" as const,step:0.10,servo:0.5},
    ];
    const results=[];
    for(const trial of cases) {
      const sim=await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial(trial.side,[trial.step,0],0.8,trial.servo,1);
        const counts=Object.fromEntries(STEP_STABILITY_CHECKS.map(k=>[k,0])) as Record<StepStabilityCheck,number>;
        let samples=0,simultaneousPassing=0;
        let firstReport=null as ReturnType<typeof sim.experimentalStepStatus>;
        for(let n=0;n<260;n++) {
          sim.step(10);
          const s=sim.experimentalStepStatus()!;
          if(!s.stability) continue;
          samples++;
          firstReport??=s;
          for(const failure of s.stability.failures) counts[failure]++;
          if(s.stability.passes) simultaneousPassing++;
        }
        const s=sim.experimentalStepStatus()!;
        const row={
          ...trial,phase:s.phase,landing:s.stepContactConfirmed,
          samples,simultaneousPassing,
          failures:counts,
          firstAfterLandingChecks:firstReport?.stability?.checks??null,
          terminalChecks:s.stability?.checks??null,
          slipM:+s.maxLandingSlipM.toFixed(4),
          targetErrorM:+s.targetErrorXYM.toFixed(4),
          settledS:+s.settledDurationS.toFixed(3),
        };
        results.push(row);
        expect(samples).toBeGreaterThan(0);
        expect(Object.values(counts).every(Number.isFinite)).toBe(true);
        if(s.phase==="COMPLETE"){
          expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
          expect(s.stability?.passes).toBe(true);
        }
      } finally {sim.dispose();}
    }
    console.log("GAIT-STABILITY-GATES "+JSON.stringify(results));
    expect(results).toHaveLength(3);
  },120000);
});
