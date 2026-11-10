import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Search in a narrow region around the physically promising soft guard
 * WITHOUT adjusting any certification limits: slip<2cm, target<5cm,
 * loaded COM support and balance continuously for >=1s.
 */
describe("Fine sweep for stable forward foot placement",()=>{
  it("measures missed stability gates for each physically simulated landing",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const rows=[];
    for(const length of [0.08,0.10,0.11,0.12]) {
      for(const servo of [0.45,0.5]) {
        for(const guard of [0.35,0.5]){
          const sim=await MujocoSimulation.create(xml);
          try{
            sim.enableExperimentalStanding();
            sim.enableExperimentalStepTrial("left",[length,0],0.8,servo,1,guard);
            let longestStableS=0;
            let gotAllGates=false;
            let postTouchMinError=Infinity;
            let finalSnapshot=sim.experimentalStepStatus()!;
            for(let s=0;s<260;s++){
              sim.step(10);
              finalSnapshot=sim.experimentalStepStatus()!;
              if(finalSnapshot.stepContactConfirmed){
                postTouchMinError=Math.min(postTouchMinError,
                  finalSnapshot.targetErrorXYM);
                longestStableS=Math.max(longestStableS,
                  finalSnapshot.settledDurationS);
                if(finalSnapshot.stabilityGates &&
                  Object.values(finalSnapshot.stabilityGates).every(Boolean))
                  gotAllGates=true;
              }
            }
            const st=finalSnapshot;
            const row={
              length,servo,guard,phase:st.phase,
              contacted:st.stepContactConfirmed,
              landingS:st.firstLandingTimeS,
              slipCm:+(st.maxLandingSlipM*100).toFixed(2),
              supportSlipCm:+(st.maxSupportFootSlipM*100).toFixed(2),
              targetErrorCm:+(st.targetErrorXYM*100).toFixed(2),
              bestPostContactErrorCm:Number.isFinite(postTouchMinError)?
                +(postTouchMinError*100).toFixed(2):null,
              stableS:+longestStableS.toFixed(2),
              allGatesEver:gotAllGates,
              failedGates:Object.entries(st.stabilityGates??{})
                .filter(([,value])=>!value).map(([name])=>name),
              minPelvis:+st.minPelvisM.toFixed(3),
            };
            rows.push(row);
            expect([row.slipCm,row.supportSlipCm,row.targetErrorCm,
              row.minPelvis].every(Number.isFinite)).toBe(true);
            if(st.phase==="COMPLETE"){
              expect(st.stepContactConfirmed).toBe(true);
              expect(st.settledDurationS).toBeGreaterThanOrEqual(1);
              expect(st.maxLandingSlipM).toBeLessThanOrEqual(0.02);
              expect(st.targetErrorXYM).toBeLessThanOrEqual(0.05);
            }
          }finally{sim.dispose();}
        }
      }
    }
    console.log("GAIT-STABILITY-FOCUSED-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(16);
  },120000);
});
