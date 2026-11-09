import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Task-space foot control gain sweep (diagnostic, not an acceptance claim)",()=>{
  it("records six-second objective and stability measurements for a controlled gain sweep",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows=[];
    for (const [direction,delta] of [
      ["forward",[0.12,0]],
      ["lateral",[0,0.12]],
      ["diagonal",[0.12,0.08]],
    ] as const) {
      for(const servo of [0,0.2,0.5,1]) {
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",delta,0.8,servo);
          const first=sim.siteWorldPosition("left_sole");
          sim.step(2600);
          const last=sim.siteWorldPosition("left_sole");
          const status=sim.experimentalStepStatus()!;
          const progress=((last[0]-first[0])*delta[0]+
            (last[1]-first[1])*delta[1])/Math.hypot(...delta);
          const error=Math.hypot(
            last[0]-first[0]-delta[0],last[1]-first[1]-delta[1]);
          rows.push({
            direction,servo,phase:status.phase,
            landing:status.stepContactConfirmed,
            rise:+status.maxSoleRiseM.toFixed(3),
            progress:+progress.toFixed(3),
            finalError:+error.toFixed(3),
            landingSlip:+status.maxLandingSlipM.toFixed(3),
            pelvisMin:+status.minPelvisM.toFixed(3),
            maxTilt:+status.maxRootTiltDeg.toFixed(1),
          });
          expect([progress,error,status.maxRootTiltDeg,status.minPelvisM]
            .every(Number.isFinite)).toBe(true);
          if(status.phase==="COMPLETE") {
            expect(status.stepContactConfirmed).toBe(true);
            expect(status.maxLandingSlipM).toBeLessThan(0.046);
            expect(progress).toBeGreaterThan(0.04);
          }
        } finally {sim.dispose();}
      }
    }
    console.log("STEP-CARTESIAN-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(12);
  },150000);
});
