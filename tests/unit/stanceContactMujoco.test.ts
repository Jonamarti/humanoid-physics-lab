import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { estimateContactPatchSlip, type MeasuredFootPose } from "../../src/physics/contactPatchSlip";

/** Explicit opt-in, because many physics trials consume CI credits.
 * Set RUN_LONG_BENCHMARKS=1 and run this file locally. No result is hardcoded:
 * the solver must reveal whether virtual backward foot effort genuinely
 * increases forward GRF without slipping or falling.
 */
describe("Rear-foot push-off versus material contact slip in real MuJoCo",()=>{
  const experiment=process.env.RUN_LONG_BENCHMARKS==="1" ? it : it.skip;
  experiment("compares measured A/B stance reactions and physical COMPLETE for both legs",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8");
    const results=[];
    for(const side of ["left","right"] as const) {
      for(const contactGain of [0,0.35]) {
        const sim=await MujocoSimulation.create(xml);
        try {
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial(side,[0.12,0],0.8,0.52,1,0.48,contactGain);
          const supportSide=side==="left"?"right":"left";
          const targetY=supportSide==="left"?1:-1;
          const footGeoms=sim.geometrySpecs().filter(g=>g.shape==="box" &&
            Math.abs(g.size[2]-0.035)<1e-6);
          const poses0=sim.geometryPoses();
          const geom=footGeoms.find(g=>Math.sign(poses0.find(p=>p.id===g.id)!.position[1])===targetY);
          expect(geom).toBeDefined();
          const swingGeom=footGeoms.find(g=>Math.sign(
            poses0.find(p=>p.id===g.id)!.position[1])===-targetY);
          expect(swingGeom).toBeDefined();
          const pose=():MeasuredFootPose=>{
            const p=sim.geometryPoses().find(x=>x.id===geom!.id)!;
            return {centerWorldM:p.position,rotationWorld:p.rotation};
          };
          const swingPose=():MeasuredFootPose=>{
            const p=sim.geometryPoses().find(x=>x.id===swingGeom!.id)!;
            return {centerWorldM:p.position,rotationWorld:p.rotation};
          };
          let previous=pose(),previousSwing=swingPose(),previousTime=sim.state().time;
          let actualPositiveImpulseNs=0,actualNegativeImpulseNs=0;
          let measuredMaterialPathM=0,maxMaterialSpeedMps=0;
          let landingMaterialPathM=0,landingPeakMaterialSpeedMps=0;
          let loadedLandingSamples=0;
          let loadedSamples=0,controlSamples=0,requestedBackwardImpulseNs=0;
          for(let tick=0;tick<2600;tick++){
            const state=sim.step(1);
            const current=pose(),currentSwing=swingPose();
            const ground=sim.humanoidGroundSupportObservation();
            const support=ground[supportSide];
            const fsm=sim.experimentalStepStatus()!;
            const dt=state.time-previousTime;
            if(support.verticalLoadN>40 && support.verticalLoadCentroidWorldM && dt>0 &&
                ["LIFT","SWING","LAND"].includes(fsm.phase)){
              const patch=estimateContactPatchSlip(previous,current,
                support.verticalLoadCentroidWorldM,dt);
              measuredMaterialPathM+=patch.estimatedSlipDistanceThisSampleM;
              maxMaterialSpeedMps=Math.max(maxMaterialSpeedMps,patch.tangentialSpeedMps);
              const longitudinalN=support.tangentialReactionXYN[0];
              actualPositiveImpulseNs+=Math.max(0,longitudinalN)*dt;
              actualNegativeImpulseNs+=Math.max(0,-longitudinalN)*dt;
              loadedSamples++;
              const command=sim.experimentalStanceContactStatus();
              if(command){
                controlSamples++;
                requestedBackwardImpulseNs+=command.intendedPushOffN*dt;
              }
            }
            const receiving=ground[side];
            if(fsm.stepContactConfirmed && receiving.verticalLoadN>40 &&
                receiving.verticalLoadCentroidWorldM && dt>0){
              const landingPatch=estimateContactPatchSlip(previousSwing,currentSwing,
                receiving.verticalLoadCentroidWorldM,dt);
              landingMaterialPathM+=landingPatch.estimatedSlipDistanceThisSampleM;
              landingPeakMaterialSpeedMps=Math.max(landingPeakMaterialSpeedMps,
                landingPatch.tangentialSpeedMps);
              loadedLandingSamples++;
            }
            previous=current;
            previousSwing=currentSwing;
            previousTime=state.time;
          }
          const status=sim.experimentalStepStatus()!;
          const row={side,contactGain,phase:status.phase,
            actualForwardGRFImpulseNs:+actualPositiveImpulseNs.toFixed(4),
            actualBackwardGRFImpulseNs:+actualNegativeImpulseNs.toFixed(4),
            intendedBackwardFootImpulseNs:+requestedBackwardImpulseNs.toFixed(4),
            measuredContactMaterialPathM:+measuredMaterialPathM.toFixed(4),
            receivingContactMaterialPathM:+landingMaterialPathM.toFixed(4),
            receivingPeakMaterialSpeedMps:+landingPeakMaterialSpeedMps.toFixed(4),
            loadedLandingSamples,
            maxMaterialSpeedMps:+maxMaterialSpeedMps.toFixed(4),
            supportSoleMarkerDriftM:+status.maxSupportFootSlipM.toFixed(4),
            landingSoleMarkerDriftM:+status.maxLandingSlipM.toFixed(4),
            targetErrorM:+status.targetErrorXYM.toFixed(4),
            stableSeconds:+status.settledDurationS.toFixed(3),
            loadedSamples,controlSamples};
          results.push(row);
          expect(loadedSamples).toBeGreaterThan(0);
          if(contactGain===0) expect(controlSamples).toBe(0);
          if(status.phase==="COMPLETE") {
            expect(status.settledDurationS).toBeGreaterThanOrEqual(1);
            expect(status.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(status.targetErrorXYM).toBeLessThanOrEqual(0.05);
          }
          expect([row.actualForwardGRFImpulseNs,row.actualBackwardGRFImpulseNs,
            row.measuredContactMaterialPathM,row.maxMaterialSpeedMps,
            row.receivingContactMaterialPathM,row.receivingPeakMaterialSpeedMps,
            row.targetErrorM].every(Number.isFinite)).toBe(true);
        } finally {sim.dispose();}
      }
    }
    console.log("REAR-FOOT-CONTACT-A-B "+JSON.stringify(results));
    expect(results).toHaveLength(4);
  },240000);
});
