import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Optional physical robustness sweep, intentionally NOT part of default CI.
 * Results are evidence, never predeclared success; failed steps remain FALL
 * or ABORT, and no acceptance gate is weakened.
 */
describe("Rear-foot contact feedback under modest changes in real ground friction",()=>{
  const experiment=process.env.RUN_STANCE_FRICTION_SWEEP==="1" ? it : it.skip;
  experiment("records two legs, three real friction values and baseline versus opt-in",async()=>{
    const referenceXml=await readFile(
      new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const frictionMarker='friction="1.0 0.025 0.002"';
    expect(referenceXml.split(frictionMarker).length).toBeGreaterThan(2);
    const rows=[];
    for(const mu of [0.9,1.0,1.1]){
      const xml=referenceXml.replaceAll(frictionMarker,
        'friction="'+mu.toFixed(1)+' 0.025 0.002"');
      for(const side of ["left","right"] as const){
        for(const contactGain of [0,0.35]){
          const sim=await MujocoSimulation.create(xml);
          try{
            sim.enableExperimentalStanding();
            sim.enableExperimentalStepTrial(side,[0.12,0],0.8,0.52,1,0.48,contactGain);
            const initial=sim.humanoidBalanceObservation().comWorldM;
            sim.step(2600);
            const result=sim.experimentalStepStatus()!;
            const final=sim.humanoidBalanceObservation().comWorldM;
            const row={mu,side,contactGain,phase:result.phase,
              comForwardM:+(final[0]-initial[0]).toFixed(4),
              maxLandingSoleDriftM:+result.maxLandingSlipM.toFixed(4),
              maxSupportSoleDriftM:+result.maxSupportFootSlipM.toFixed(4),
              targetErrorM:+result.targetErrorXYM.toFixed(4),
              sustainedSupportS:+result.settledDurationS.toFixed(3),
              landingVerified:result.stepContactConfirmed,
            };
            rows.push(row);
            expect([row.comForwardM,row.maxLandingSoleDriftM,
              row.maxSupportSoleDriftM,row.targetErrorM,
              row.sustainedSupportS].every(Number.isFinite)).toBe(true);
            if(result.phase==="COMPLETE"){
              expect(result.stepContactConfirmed).toBe(true);
              expect(result.maxLandingSlipM).toBeLessThanOrEqual(0.02);
              expect(result.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
              expect(result.targetErrorXYM).toBeLessThanOrEqual(0.05);
              expect(result.settledDurationS).toBeGreaterThanOrEqual(1);
            }
          }finally{sim.dispose();}
        }
      }
    }
    console.log("STANCE-FRICTION-SENSITIVITY "+JSON.stringify(rows));
    expect(rows).toHaveLength(12);
  },240000);
});
