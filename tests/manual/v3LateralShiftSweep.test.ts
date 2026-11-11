import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { rootTiltDegrees } from "../../src/physics/standingPush";

/** Physically measured 6s sweep; unchanged acceptance gates.
 * Only changes the bounded lateral COM reference on articulated v3.
 * Run explicitly: HPL_RUN_V3_SHIFT_SWEEP=1 npx vitest run tests/manual/v3LateralShiftSweep.test.ts
 */
describe.skipIf(process.env.HPL_RUN_V3_SHIFT_SWEEP !== "1")(
  "V3 lateral COM reference gain vs solved physical loading", () => {
  it("reports support fraction, transfer and failure for both legs", async () => {
    const xml = await readFile(new URL("../../public/models/humanoid_gait_v3.xml", import.meta.url),"utf8");
    const results = [];
    for(const shiftM of [0.075,0.095,0.115,0.135]){
      for(const side of ["left","right"] as const){
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial(side,[0.12,0],0.8,0.52,1,0.48,shiftM);
          let maxSupportFraction=0;
          let maxPreLiftFraction=0;
          let minPelvisM=Infinity;
          let maxTiltDeg=0;
          let firstLiftS:number|null=null;
          let firstFallS:number|null=null;
          let maxSupportToeN=0;
          const trace: Array<{t:number;phase:string;supportFraction:number;pelvisM:number;toeN:number}>=[];
          let lastPhase="";
          for(let frame=0;frame<300;frame++){
            sim.step(10);
            const snapshot=sim.experimentalStepStatus()!;
            const state=sim.state();
            const ground=sim.humanoidGroundSupportObservation();
            const support=ground[side==="left"?"right":"left"];
            const swing=ground[side];
            const sum=support.verticalLoadN+swing.verticalLoadN;
            const fraction=sum>10?support.verticalLoadN/sum:0;
            maxSupportFraction=Math.max(maxSupportFraction,fraction);
            if(["STAND","SHIFT_WEIGHT"].includes(snapshot.phase))
              maxPreLiftFraction=Math.max(maxPreLiftFraction,fraction);
            minPelvisM=Math.min(minPelvisM,state.qpos[2]!);
            maxTiltDeg=Math.max(maxTiltDeg,rootTiltDegrees(state.qpos.slice(0,7)));
            maxSupportToeN=Math.max(maxSupportToeN,support.segments!.forefoot.verticalLoadN);
            if(snapshot.phase==="LIFT" && firstLiftS===null)firstLiftS=state.time;
            if(snapshot.phase==="FALL" && firstFallS===null)firstFallS=state.time;
            if(snapshot.phase!==lastPhase){
              lastPhase=snapshot.phase;
              trace.push({t:+state.time.toFixed(3),phase:snapshot.phase,
                supportFraction:+fraction.toFixed(4),pelvisM:+state.qpos[2]!.toFixed(4),
                toeN:+support.segments!.forefoot.verticalLoadN.toFixed(2)});
            }
          }
          results.push({side,shiftM,phase:sim.experimentalStepStatus()!.phase,
            reason:sim.experimentalStepStatus()!.reason,
            maxSupportFraction:+maxSupportFraction.toFixed(4),
            maxPreLiftFraction:+maxPreLiftFraction.toFixed(4),
            minPelvisM:+minPelvisM.toFixed(4),
            maxTiltDeg:+maxTiltDeg.toFixed(3),
            maxSupportToeN:+maxSupportToeN.toFixed(1),
            firstLiftS:firstLiftS===null?null:+firstLiftS.toFixed(3),
            firstFallS:firstFallS===null?null:+firstFallS.toFixed(3),
            landed:sim.experimentalStepStatus()!.stepContactConfirmed,
            settledS:sim.experimentalStepStatus()!.settledDurationS,
            trace});
        }finally{sim.dispose();}
      }
    }
    for(const result of results)console.log("V3-LATERAL-SHIFT-SWEEP "+JSON.stringify(result));
    expect(results).toHaveLength(8);
    expect(results.every(r=>Number.isFinite(r.maxSupportFraction) &&
      Number.isFinite(r.minPelvisM) && Number.isFinite(r.maxTiltDeg))).toBe(true);
    // No assertion of successful walking; real ABORT/FALL remains meaningful.
  },180000);
});
