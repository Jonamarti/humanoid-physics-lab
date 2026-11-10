import { readFile } from "node:fs/promises";
import { describe, expect, it } from "vitest";

describe("No regression to published standing/push humanoid",()=>{
  it("separates knee-corrected gait MJCF from nominal 75kg model",async()=>{
    const [nominal,gait]=await Promise.all([
      readFile(new URL("../../public/models/humanoid_v1.xml",import.meta.url),"utf8"),
      readFile(new URL("../../public/models/humanoid_step_v2.xml",import.meta.url),"utf8"),
    ]);
    for(const side of ["left","right"]) {
      expect(nominal).toContain(`<joint name="${side}_knee" type="hinge" axis="0 1 0" range="-2.45 0"/>`);
      expect(gait).toContain(`<joint name="${side}_knee" type="hinge" axis="0 -1 0" range="-2.45 0"/>`);
    }
    // Remove only XML documentation and model name; all other physical
    // parameters must match once the two corrected knee axes are undone.
    const normalize=(xml:string)=>xml.replace(/<!--[\s\S]*?-->/g,"")
      .replace(/model="humanoid_step_v2"/g,'model="humanoid_v1"')
      .replace(/(name="(?:left|right)_knee" type="hinge" axis=")0 -1 0"/g,'$10 1 0"')
      .replace(/\s+/g," ").trim();
    expect(normalize(gait)).toEqual(normalize(nominal));
  });
});
