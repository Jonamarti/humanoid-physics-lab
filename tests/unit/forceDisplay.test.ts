import { describe, expect, it } from "vitest";
import { forceDisplayForFrame } from "../../src/renderer/forceDisplay";
import { pushForceAtTime, type PushTrial } from "../../src/physics/standingPush";

const trial: PushTrial = { direction:"forward-left", peakForceN: 450,
  startS: 1, durationS: 0.12 };

describe("Force arrow: persistent reference, highlight only on real applied force", () => {
  it("keeps a direction reference before and after the push but displays zero active force", () => {
    const before = forceDisplayForFrame(0, trial);
    const after = forceDisplayForFrame(4, trial);
    expect(before.phase).toBe("pending");
    expect(after.phase).toBe("applied");
    expect(before.referenceForceN).toEqual(after.referenceForceN);
    expect(before.displayForceN).toEqual(after.displayForceN);
    expect(before.activeMagnitudeN).toBe(0);
    expect(after.activeFraction).toBe(0);
  });
  it("highlights a very short real force even if the frame advances past pulse end", () => {
    const actual = pushForceAtTime(1.06, trial);
    const sample = forceDisplayForFrame(1.14, trial, actual);
    expect(sample.phase).toBe("active");
    expect(sample.activeMagnitudeN).toBeCloseTo(450,10);
    expect(sample.activeFraction).toBeCloseTo(1);
    expect(sample.displayForceN).toEqual(actual);
    const nextFrame = forceDisplayForFrame(1.16, trial);
    expect(nextFrame.phase).toBe("applied");
    expect(nextFrame.activeMagnitudeN).toBe(0);
  });
  it("supports a longer load without pretending it continues after the solver stops", () => {
    const longer = { ...trial, durationS: 0.8 };
    const a = forceDisplayForFrame(1.3, longer,
      pushForceAtTime(1.28, longer));
    expect(a.phase).toBe("active");
    const end = forceDisplayForFrame(1.85, longer);
    expect(end.phase).toBe("applied");
  });
  it("refuses non-finite force or time", () => {
    expect(() => forceDisplayForFrame(NaN,trial)).toThrow(RangeError);
    expect(() => forceDisplayForFrame(1,trial,[Infinity,0,0])).toThrow(RangeError);
  });
});
