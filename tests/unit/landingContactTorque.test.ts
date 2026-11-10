import { describe, expect, it } from "vitest";
import { attenuateSwingHipTorques, contactHipTorqueScale } from "../../src/control/landingContactTorque";

describe("Friction-aware swing hip torque relief (opt-in only)", () => {
  const typical = {
    phase:"RECOVER",normalForceN:300,frictionUtilization:0.95,
    soleTangentialSpeedMps:0.3,relief:1,
  };
  it("only engages with grounded, friction-limited sliding", () => {
    expect(contactHipTorqueScale(typical)).toBeCloseTo(0.25,8);
    expect(contactHipTorqueScale({...typical,relief:0})).toBe(1);
    expect(contactHipTorqueScale({...typical,phase:"SWING"})).toBe(1);
    expect(contactHipTorqueScale({...typical,normalForceN:0})).toBe(1);
    expect(contactHipTorqueScale({...typical,normalForceN:39})).toBe(1);
    expect(contactHipTorqueScale({...typical,frictionUtilization:null})).toBe(1);
    expect(contactHipTorqueScale({...typical,frictionUtilization:0.35})).toBe(1);
    expect(contactHipTorqueScale({...typical,soleTangentialSpeedMps:0.01})).toBe(1);
    expect(contactHipTorqueScale({...typical,relief:0.5})).toBeCloseTo(0.625,8);
  });
  it("rejects invalid physical measurements and never increases torque", () => {
    expect(() => contactHipTorqueScale({...typical,relief:1.01})).toThrow(RangeError);
    expect(() => contactHipTorqueScale({...typical,normalForceN:NaN})).toThrow(RangeError);
    expect(() => contactHipTorqueScale({...typical,frictionUtilization:-1})).toThrow(RangeError);
    expect(() => contactHipTorqueScale({...typical,soleTangentialSpeedMps:-1})).toThrow(RangeError);
    for(const utilization of [0,0.2,0.5,0.6,0.8,1,1.1]) {
      const output=contactHipTorqueScale({...typical,frictionUtilization:utilization});
      expect(output).toBeGreaterThanOrEqual(0.25);
      expect(output).toBeLessThanOrEqual(1);
    }
  });
  it("attenuates only internal hip DOFs and leaves knee, ankles, support leg and root unchanged", () => {
    const names=["hip_pitch","hip_roll","hip_yaw"];
    const joints = new Map(names.map((name,i)=>["left_"+name,i+1] as const));
    const model={
      njnt:4,
      jnt:(name:string)=>({id:joints.get(name)??-1}),
      jnt_type:[0,3,3,3],
      jnt_dofadr:[0,6,7,8],
    };
    const torques=new Float64Array(15);
    torques[6]=40;torques[7]=-20;torques[8]=12;
    torques[9]=50;torques[10]=-30;
    attenuateSwingHipTorques(model,torques,"left",0.6);
    expect(Array.from(torques.slice(0,6))).toEqual([0,0,0,0,0,0]);
    expect(torques[6]).toBe(24);
    expect(torques[7]).toBe(-12);
    expect(torques[8]).toBeCloseTo(7.2,10);
    expect(torques[9]).toBe(50);
    expect(torques[10]).toBe(-30);
    expect(() => attenuateSwingHipTorques(model,torques,"right",0.4)).toThrow();
    expect(() => attenuateSwingHipTorques(model,torques,"left",0.1)).toThrow(RangeError);
  });
});
