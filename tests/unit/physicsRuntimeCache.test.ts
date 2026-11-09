import { describe, expect, it } from "vitest";
import { loadPhysicsModule } from "../../src/physics/mujoco";

describe("MuJoCo initialization is cached across physical scenarios", () => {
  it("shares exactly the same WASM module initialization promise", async () => {
    const one = loadPhysicsModule();
    const two = loadPhysicsModule();
    const three = loadPhysicsModule();
    expect(two).toBe(one);
    expect(three).toBe(one);
    const module = await one;
    expect(module).toBeTruthy();
    expect(await loadPhysicsModule()).toBe(module);
  }, 30000);
});
