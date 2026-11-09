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

  it("measures first loaded contact speed in an unmodified physical step",async()=>{
    const xml=await readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8");
    const rows=[];
    for(const servo of [0,0.2]){
      const sim=await MujocoSimulation.create(xml);
      try {
        sim.enableExperimentalStanding();
        sim.enableExperimentalStepTrial("left",[0.12,0.08],0.8,servo);
        sim.step(2600);
        const s=sim.experimentalStepStatus()!;
        rows.push({servo,phase:s.phase,contact:s.stepContactConfirmed,
          landingSpeedXY:s.firstLandingHorizontalSpeedMps,
          landingSpeedDown:s.firstLandingDownwardSpeedMps,
          firstContactN:s.firstLandingNormalForceN,
          slip:s.maxLandingSlipM,
          supportSlip:s.maxSupportFootSlipM});
        if (s.stepContactConfirmed) {
          expect(s.firstLandingHorizontalSpeedMps).not.toBeNull();
          expect(s.firstLandingDownwardSpeedMps).not.toBeNull();
          expect(s.firstLandingNormalForceN).toBeGreaterThan(40);
        }
        expect([s.maxLandingSlipM,s.maxSupportFootSlipM]
          .every(Number.isFinite)).toBe(true);
      }finally{sim.dispose();}
    }
    console.log("STEP-FIRST-CONTACT-TELEMETRY "+JSON.stringify(rows));
    expect(rows).toHaveLength(2);
  },60000);
});
