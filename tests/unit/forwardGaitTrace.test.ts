import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";

describe("Forward swing physics trace with posterior knee flexion",()=>{
  it("records q/knee/hip COM and actual toe path through phase transitions",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const sim=await MujocoSimulation.create(xml);
    try {
      sim.enableExperimentalStanding();
      sim.enableExperimentalStepTrial("left",[0.12,0],0.8,0.2,1);
      const initialFoot=sim.siteWorldPosition("left_sole");
      const com0=sim.humanoidBalanceObservation().comWorldM;
      const rows=[];
      let prevPhase="",nextTime=0;
      for(let i=0;i<195;i++){
        const state=sim.step(10);
        const status=sim.experimentalStepStatus()!;
        if(status.phase!==prevPhase || state.time>=nextTime) {
          prevPhase=status.phase;
          nextTime=state.time+0.16;
          const p=sim.siteWorldPosition("left_sole");
          const b=sim.humanoidBalanceObservation();
          const grf=sim.humanoidGroundSupportObservation();
          rows.push({
            t:+state.time.toFixed(2),phase:status.phase,
            hipDeg:+(sim.jointAngleRad("left_hip_pitch")*180/Math.PI).toFixed(1),
            kneeDeg:+(sim.jointAngleRad("left_knee")*180/Math.PI).toFixed(1),
            lumbarDeg:+(sim.jointAngleRad("lumbar_pitch")*180/Math.PI).toFixed(1),
            dx:+(p[0]-initialFoot[0]).toFixed(3),
            dz:+(p[2]-initialFoot[2]).toFixed(3),
            comX:+(b.comWorldM[0]-com0[0]).toFixed(3),
            comY:+(b.comWorldM[1]-com0[1]).toFixed(3),
            loadL:+grf.left.verticalLoadN.toFixed(0),
            loadR:+grf.right.verticalLoadN.toFixed(0),
            pelvis:+state.qpos[2]!.toFixed(3),
          });
        }
      }
      console.log("GAIT-FORWARD-TRACES "+JSON.stringify(rows));
      expect(rows.length).toBeGreaterThan(10);
    } finally {sim.dispose();}
  },45000);
});
