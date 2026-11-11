import {describe,expect,it} from "vitest";
import {ExperimentalStepSequence,type StepFeedback} from "../../src/control/experimentalStep";

const sample=(timeS:number,overrides:Partial<StepFeedback>={}):StepFeedback=>({
  timeS,
  comWorldM:[0,0,.99],comVelocityWorldMps:[.40,0,0],
  comProjectionInsideLoadedSupport:true,
  capturePointInsideLoadedSupport:false,
  pelvisHeightM:.98,rootTiltDeg:3,
  swingSoleWorldM:[.065,.11,0],supportSoleWorldM:[.065,-.11,0],
  supportLoadN:545,swingLoadN:191,
  ...overrides,
});
const seq=(reactive:boolean)=>new ExperimentalStepSequence(
  "left",[-.22,0],0,1,.48,true,reactive);

describe("Emergency capture-point step transfer (reactive v3 only)",()=>{
  it("starts a physical lift only after 72% supported load has persisted",()=>{
    const emergency=seq(true);
    const nominal=seq(false);
    emergency.observe(sample(0));nominal.observe(sample(0));
    for(let i=1;i<=26;i++){
      const t=+(i*.02).toFixed(2);
      emergency.observe(sample(t));
      nominal.observe(sample(t));
    }
    expect(emergency.snapshot().phase).toBe("LIFT");
    expect(nominal.snapshot().phase).toBe("SHIFT_WEIGHT");
    const cmd=emergency.command(.52,[0,0]);
    expect(cmd.targetsRad.left_knee).toBeLessThan(0);
    expect(cmd.referenceComXY[1]).toBeLessThan(-.07);
    expect(cmd.targetsRad.pelvis_root).toBeUndefined();
    expect(emergency.snapshot().stepContactConfirmed).toBe(false);
  });
  it("will not release swing foot with inadequate load or excessive tilt",()=>{
    const emergency=seq(true);
    emergency.observe(sample(0));
    for(let i=1;i<=28;i++){
      const t=i*.02;
      emergency.observe(sample(t,{supportLoadN:510,swingLoadN:226}));
    }
    expect(emergency.snapshot().phase).toBe("SHIFT_WEIGHT");
    for(let i=29;i<=35;i++){
      emergency.observe(sample(i*.02,{rootTiltDeg:28}));
    }
    expect(emergency.snapshot().phase).toBe("SHIFT_WEIGHT");
  });
  it("does not change legacy v1/v2 or guarded v3 lift thresholds",()=>{
    const legacy=new ExperimentalStepSequence("left",[.12,0],0,1,.48,false,false);
    const guarded=seq(false);
    legacy.observe(sample(0));guarded.observe(sample(0));
    legacy.observe(sample(.3,{supportLoadN:540,swingLoadN:196}));
    guarded.observe(sample(.3,{supportLoadN:540,swingLoadN:196}));
    expect(legacy.snapshot().phase).toBe("LIFT");
    expect(guarded.snapshot().phase).toBe("SHIFT_WEIGHT");
  });
});
