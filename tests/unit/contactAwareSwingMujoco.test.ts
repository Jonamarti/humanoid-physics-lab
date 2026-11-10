import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

/** Reproducible A/B/C: no fake root displacement or contact. Controls:
 * same 75 kg model, 0.002 s timestep, torso COM strategy, original stage
 * planner and Jacobian foot servo. Only swing knee-extension guard changes.
 */
describe("Contact-aware swing phase: measured forward step comparisons",()=>{
  it("compares timing and physical results of early vs progress-gated knee extension",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const rows=[];
    for(const [stepLength,servo] of [[0.12,0.2],[0.10,0.5],[0.12,0.5]] as const){
      for(const guard of [0,0.5,1]){
        const sim=await MujocoSimulation.create(xml);
        try{
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[stepLength,0],0.8,servo,1,guard);
          const initial=sim.siteWorldPosition("left_sole");
          let maxSwingX=0,maxSwingRise=0,maxKneeRad=0;
          let firstLoadedTime:number|null=null;
          let xAtFirstLoad:number|null=null;
          for(let i=0;i<260;i++){
            const s=sim.step(10);
            const st=sim.experimentalStepStatus()!;
            const foot=sim.siteWorldPosition("left_sole");
            if(["LIFT","SWING","LAND"].includes(st.phase)){
              maxSwingX=Math.max(maxSwingX,foot[0]-initial[0]);
              maxSwingRise=Math.max(maxSwingRise,foot[2]-initial[2]);
            }
            maxKneeRad=Math.max(maxKneeRad,-sim.jointAngleRad("left_knee"));
            if(st.stepContactConfirmed && firstLoadedTime===null){
              firstLoadedTime=s.time;
              xAtFirstLoad=foot[0]-initial[0];
            }
          }
          const st=sim.experimentalStepStatus()!;
          const row={
            stepLength,servo,guard,phase:st.phase,
            kneeDeg:+(maxKneeRad*180/Math.PI).toFixed(1),
            swingAdvanceM:+maxSwingX.toFixed(4),
            swingRiseM:+maxSwingRise.toFixed(4),
            firstTouchS:firstLoadedTime===null?null:+firstLoadedTime.toFixed(3),
            xAtFirstTouchM:xAtFirstLoad===null?null:+xAtFirstLoad.toFixed(4),
            landed:st.stepContactConfirmed,
            slipM:+st.maxLandingSlipM.toFixed(4),
            supportSlipM:+st.maxSupportFootSlipM.toFixed(4),
            errorM:+st.targetErrorXYM.toFixed(4),
            stableS:+st.settledDurationS.toFixed(2),
            pelvisMinM:+st.minPelvisM.toFixed(3),
          };
          rows.push(row);
          expect([row.kneeDeg,row.swingAdvanceM,row.swingRiseM,row.slipM,
            row.supportSlipM,row.errorM,row.pelvisMinM].every(Number.isFinite)).toBe(true);
          if(st.phase==="COMPLETE"){
            expect(st.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(st.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
            expect(st.targetErrorXYM).toBeLessThanOrEqual(0.05);
            expect(st.settledDurationS).toBeGreaterThanOrEqual(1);
          }
        } finally {sim.dispose();}
      }
    }
    console.log("GAIT-SWING-PROGRESS-GATE "+JSON.stringify(rows));
    expect(rows).toHaveLength(9);
  },120000);
});
