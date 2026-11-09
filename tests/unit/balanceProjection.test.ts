import { describe, expect, it } from "vitest";
import { projectBalanceToGround } from "../../src/renderer/balanceProjection";

const support = [
  [-0.08, -0.17], [0.2,-0.17], [0.2,0.17], [-0.08,0.17],
] as const;

describe("Physical COM projection and foot-loaded hull", () => {
  it("projects vertical Z into floor without moving COM horizontally", () => {
    expect(projectBalanceToGround([0.06, 0.02, 1.0], support)).toEqual({
      groundPoint: [0.06,0.02,0], status:"inside",
    });
    expect(projectBalanceToGround([0.25,0,0.8],support).status).toBe("outside");
  });
  it("reports unknown rather than incorrectly claiming a fall when no feet have load", () => {
    expect(projectBalanceToGround([1,0,0.2], []).status).toBe("unknown");
    expect(projectBalanceToGround([1,0,0.2], [[0,0],[1,0]]).status).toBe("unknown");
  });
  it("rejects non-finite coordinates", () => {
    expect(() => projectBalanceToGround([NaN,0,1],support)).toThrow(RangeError);
    expect(() => projectBalanceToGround([0,0,1],[[0,Infinity]])).toThrow(RangeError);
  });
});
