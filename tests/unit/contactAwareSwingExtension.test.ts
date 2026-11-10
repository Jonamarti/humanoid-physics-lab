import { describe, expect, it } from "vitest";
import { controlledStepPosture } from "../../src/control/controlledStepPosture";
import { ExperimentalStepSequence } from "../../src/control/experimentalStep";

const params={
  phase:"SWING" as const,
  elapsedS:0.36,
  swingSide:"left" as const,
  targetDeltaXY:[0.12,0] as const,
  intensity:1,
};

describe("Foot-progress-driven knee extension, physically opt-in",()=>{
  it("keeps knee folded when the foot is far from destination and unfolds near target",()=>{
    const old=controlledStepPosture({...params,swingProgress01:0.3,swingExtensionGuard:0});
    const guarded=controlledStepPosture({...params,swingProgress01:0.3,swingExtensionGuard:1});
    const nearGoal=controlledStepPosture({...params,swingProgress01:0.85,swingExtensionGuard:1});
    expect(guarded.requestedKneeFlexionRad).toBeLessThan(old.requestedKneeFlexionRad);
    expect(nearGoal.requestedKneeFlexionRad).toBeGreaterThan(guarded.requestedKneeFlexionRad);
    expect(nearGoal.requestedKneeFlexionRad).toBeCloseTo(old.requestedKneeFlexionRad,10);
  });

  it("does not alter legacy postures unless enabled; forbids invalid controller settings",()=>{
    const a=controlledStepPosture(params);
    const b=controlledStepPosture({...params,swingProgress01:-10,swingExtensionGuard:0});
    expect(a).toEqual(b);
    for(const bad of [-1,1.01,Infinity,NaN]){
      expect(()=>controlledStepPosture({...params,swingExtensionGuard:bad})).toThrow(RangeError);
      expect(()=>new ExperimentalStepSequence("left",[0.12,0],0.8,1,bad)).toThrow(RangeError);
    }
  });

  it("does not modify the trunk / COM targets while only gating knee extension",()=>{
    const a=controlledStepPosture({...params,swingProgress01:0.3,swingExtensionGuard:0});
    const b=controlledStepPosture({...params,swingProgress01:0.3,swingExtensionGuard:1});
    expect(a.forwardComBiasM).toBe(b.forwardComBiasM);
    expect(a.targetsRad.lumbar_pitch).toBe(b.targetsRad.lumbar_pitch);
    expect(a.targetsRad.thorax_pitch).toBe(b.targetsRad.thorax_pitch);
    expect(a.targetsRad.left_hip_pitch).toBe(b.targetsRad.left_hip_pitch);
  });
});
