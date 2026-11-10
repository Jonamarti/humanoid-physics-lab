import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { estimateContactPatchSlip, type MeasuredFootPose } from "../../src/physics/contactPatchSlip";

/** Evidence-only comparison. A loaded contact moving to a different heel/toe
 * point does NOT imply the same physical material point slid along the floor.
 * This reports both site displacement and a frame-differenced contact-velocity
 * estimate; neither diagnostic alone constitutes a successful recovery step.
 */
describe("Real MuJoCo patch slip vs moving sole-site in step landing",()=>{
  it("compares rotating contact patch and sole site under two controller gains",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows=[];
    for(const servo of [0,0.2]){
      const sim=await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo);
        const boxes=sim.geometrySpecs().filter(g=>g.shape==="box" &&
          Math.abs(g.size[2]-0.035)<1e-6);
        expect(boxes).toHaveLength(2);
        const geoms=sim.geometryPoses();
        const leftGeomId=boxes.find(g=>
          geoms.find(p=>p.id===g.id)!.position[1]>0)!.id;
        const pose=():MeasuredFootPose=>{
          const p=sim.geometryPoses().find(g=>g.id===leftGeomId)!;
          return {centerWorldM:p.position,rotationWorld:p.rotation};
        };
        let prev=pose(),prevTime=sim.state().time;
        let loadedSamples=0;
        let totalContactPatchSlipM=0;
        let maxContactPatchVelocityMps=0;
        let firstAfterLanding: number|null=null;
        let maxContactFriction=0;
        for(let n=0;n<260;n++){
          const state=sim.step(10);
          const current=pose();
          const s=sim.experimentalStepStatus()!;
          const ground=sim.humanoidGroundSupportObservation();
          if(s.stepContactConfirmed && firstAfterLanding===null) {
            firstAfterLanding=state.time;
          }
          const contact=ground.left.verticalLoadCentroidWorldM;
          if(firstAfterLanding!==null && contact && ground.left.verticalLoadN>40) {
            const velocity=estimateContactPatchSlip(prev,current,contact,state.time-prevTime);
            totalContactPatchSlipM += velocity.estimatedSlipDistanceThisSampleM;
            maxContactPatchVelocityMps=Math.max(maxContactPatchVelocityMps,
              velocity.tangentialSpeedMps);
            loadedSamples++;
            maxContactFriction=Math.max(maxContactFriction,
              ground.left.frictionUtilizationMean ?? 0);
          }
          prev=current;prevTime=state.time;
        }
        const s=sim.experimentalStepStatus()!;
        const row={
          servo,phase:s.phase,firstAfterLandingTimeS:firstAfterLanding,
          loadedSamples,measuredPatchDistanceM:+totalContactPatchSlipM.toFixed(4),
          peakPatchVelocityMps:+maxContactPatchVelocityMps.toFixed(4),
          soleSiteDriftAfterLandingM:+s.maxLandingSlipM.toFixed(4),
          maxFriction:+maxContactFriction.toFixed(3),
          destinationErrorM:+s.targetErrorXYM.toFixed(4),
        };
        rows.push(row);
        if (loadedSamples === 0) {
          // A physical FALL before landing has no loaded contact material
          // to measure. It is a legitimate negative gait result, not zero slip.
          expect(s.phase === "FALL" || s.phase === "ABORT").toBe(true);
          expect(s.stepContactConfirmed).toBe(false);
        }
        expect([row.measuredPatchDistanceM,row.peakPatchVelocityMps,
          row.soleSiteDriftAfterLandingM,row.destinationErrorM].every(Number.isFinite)).toBe(true);
      } finally {sim.dispose();}
    }
    console.log("CONTACT-PATCH-SLIP-VS-SOLE-SITE "+JSON.stringify(rows));
    expect(rows).toHaveLength(2);
  },120000);
});
