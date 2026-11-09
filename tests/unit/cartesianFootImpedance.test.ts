import { describe,expect,it } from "vitest";
import {
  hingeSiteJacobianColumn,desiredFootCartesianForce,
  applyCartesianFootImpedance,
} from "../../src/control/cartesianFootImpedance";

describe("World-space hinge Jacobian and bounded foot impedance",()=>{
  it("derives correct signed motion of hip pitch and hip roll",()=>{
    const p:[number,number,number]=[0,0,-0.84];
    const pitch=hingeSiteJacobianColumn([0,1,0],[0,0,0],p);
    const roll=hingeSiteJacobianColumn([1,0,0],[0,0,0],p);
    const yaw=hingeSiteJacobianColumn([0,0,1],[0,0,0],p);
    expect(pitch[0]).toBeCloseTo(-0.84,12);
    expect(roll[1]).toBeCloseTo(0.84,12);
    expect(yaw.every(v=>Math.abs(v)<1e-12)).toBe(true);
  });
  it("limits force with physically defined stiffness and measured velocity",()=>{
    const base={targetWorldM:[0.2,0.1,0.05] as const,
      currentWorldM:[0,0,0] as const,
      measuredVelocityWorldMps:[0.1,0,0] as const,
      gainsNPerM:[800,700,900] as const,
      dampingNsPerM:[100,100,90] as const,
      maxFootForceN:100,blend:1};
    expect(desiredFootCartesianForce(base)).toEqual([100,70,45]);
    expect(desiredFootCartesianForce({...base,blend:0})).toEqual([0,0,0]);
    expect(()=>desiredFootCartesianForce({...base,blend:Infinity})).toThrow(RangeError);
    expect(()=>desiredFootCartesianForce({...base,
      currentWorldM:[Infinity,0,0]})).toThrow(RangeError);
  });
  it("applies only limited internal torques to stance/swing hinge DOFs",()=>{
    const names=["hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll"];
    const map=new Map(names.map((name,i)=>["left_"+name,i+1] as const));
    const model={
      njnt:7,
      jnt:(name:string)=>({id:map.get(name)??-1}),
      jnt_type:[0,3,3,3,3,3,3],
      jnt_dofadr:[0,6,7,8,9,10,11],
      jnt_qposadr:[0,7,8,9,10,11,12],
    };
    const data={
      qpos:new Float64Array(14),qvel:new Float64Array(14),
      qfrc_applied:new Float64Array(14),
      xaxis:new Float64Array(21),
      xanchor:new Float64Array(21),
    };
    data.xaxis[4]=1; // joint 1 axis +Y
    data.xaxis[6]=1; // joint 2 axis +X
    const gains=new Map(names.map((_,i)=>[i+1,{
      targetRad:0,stiffnessNmPerRad:350,dampingNmsPerRad:30,maxTorqueNm:40,
    }] as const));
    const params={
      targetWorldM:[0.1,0.1,0] as const,
      currentWorldM:[0,0,-0.8] as const,
      measuredVelocityWorldMps:[0,0,0] as const,
      gainsNPerM:[1000,1000,0] as const,
      dampingNsPerM:[80,80,80] as const,
      maxFootForceN:150,blend:1,
    };
    const out=applyCartesianFootImpedance(model,data,gains,"left",params);
    expect(data.qfrc_applied.slice(0,6)).toEqual(new Float64Array(6));
    expect(data.qfrc_applied[6]).toBeCloseTo(-40);
    expect(data.qfrc_applied[7]).toBeCloseTo(40);
    expect(out.maxAbsTorqueNm).toBeLessThanOrEqual(40);
  });
});
