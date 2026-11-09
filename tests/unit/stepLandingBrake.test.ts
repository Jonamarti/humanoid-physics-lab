import { readFile } from "node:fs/promises";
import { describe,expect,it } from "vitest";
import { MujocoSimulation } from "../../src/physics/mujoco";
import { ExperimentalStepSequence,type StepFeedback } from "../../src/control/experimentalStep";

describe("Bounded landing velocity feedback, with real measured contact",()=>{
  it("records the velocity and normal load of first observed loaded landing",()=>{
    const feedback=(t:number, extras:Partial<StepFeedback>={}):StepFeedback=>({
      timeS:t,comWorldM:[0,0,0.95],comVelocityWorldMps:[0,0,0],
      comProjectionInsideLoadedSupport:true,capturePointInsideLoadedSupport:true,
      pelvisHeightM:0.99,rootTiltDeg:1,
      swingSoleWorldM:[0.065,0.105,0.001],
      swingSoleVelocityWorldMps:[0,0,0],
      supportSoleWorldM:[0.065,-0.105,0.001],
      supportLoadN:360,swingLoadN:370,...extras,
    });
    const step=new ExperimentalStepSequence("left",[0.12,0.08],0.8);
    step.observe(feedback(0));
    step.observe(feedback(0.8));
    step.observe(feedback(1.15,{supportLoadN:630,swingLoadN:100}));
    expect(step.snapshot().phase).toBe("LIFT");
    step.observe(feedback(1.24,{swingSoleWorldM:[0.065,0.105,0.06],
      supportLoadN:735,swingLoadN:0}));
    expect(step.snapshot().phase).toBe("SWING");
    step.observe(feedback(1.4,{swingSoleWorldM:[0.17,0.18,0.06],
      supportLoadN:735,swingLoadN:0}));
    expect(step.snapshot().phase).toBe("LAND");
    expect(step.snapshot().firstLandingHorizontalSpeedMps).toBeNull();
    step.observe(feedback(1.54,{swingSoleWorldM:[0.17,0.18,0.003],
      swingSoleVelocityWorldMps:[0.24,-0.18,-0.39],
      supportLoadN:590,swingLoadN:180}));
    expect(step.snapshot().phase).toBe("RECOVER");
    expect(step.snapshot().firstLandingHorizontalSpeedMps).toBeCloseTo(0.30);
    expect(step.snapshot().firstLandingDownwardSpeedMps).toBeCloseTo(0.39);
    expect(step.snapshot().firstLandingNormalForceN).toBe(180);
  });

  it("rejects invalid braking gain without changing the actual solver",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const sim=await MujocoSimulation.create(xml);
    try {
      sim.enableExperimentalStanding();
      expect(()=>sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,0.2,NaN))
        .toThrow(RangeError);
      expect(()=>sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,0.2,1.1))
        .toThrow(RangeError);
    }finally{sim.dispose();}
  });

  it("measures before/after damping with equal physics in MuJoCo",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows=[];
    for(const servo of [0,0.2]){
      for(const braking of [0,0.35,0.7,1]){
        const sim=await MujocoSimulation.create(xml);
        try{
          sim.enableExperimentalStanding();
          sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo,braking);
          sim.step(2600);
          const s=sim.experimentalStepStatus()!;
          rows.push({servo,braking,phase:s.phase,
            contact:s.stepContactConfirmed,
            landingSpeedXY:s.firstLandingHorizontalSpeedMps===null?null:
              +s.firstLandingHorizontalSpeedMps.toFixed(3),
            landingSpeedDown:s.firstLandingDownwardSpeedMps===null?null:
              +s.firstLandingDownwardSpeedMps.toFixed(3),
            firstContactN:s.firstLandingNormalForceN===null?null:
              +s.firstLandingNormalForceN.toFixed(0),
            slip:+s.maxLandingSlipM.toFixed(3),
            supportSlip:+s.maxSupportFootSlipM.toFixed(3),
            targetError:+s.targetErrorXYM.toFixed(3),
            stableS:+s.settledDurationS.toFixed(2),
            minPelvis:+s.minPelvisM.toFixed(3)});
          expect(s.maxLandingSlipM).toBeGreaterThanOrEqual(0);
          expect(s.targetErrorXYM).toBeGreaterThanOrEqual(0);
          if(s.phase==="COMPLETE"){
            expect(s.settledDurationS).toBeGreaterThanOrEqual(1);
            expect(s.maxLandingSlipM).toBeLessThanOrEqual(0.02);
            expect(s.maxSupportFootSlipM).toBeLessThanOrEqual(0.02);
          }
        }finally{sim.dispose();}
      }
    }
    console.log("STEP-LANDING-BRAKE-SWEEP "+JSON.stringify(rows));
    expect(rows).toHaveLength(8);
  },120000);
});
