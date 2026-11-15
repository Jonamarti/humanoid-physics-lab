import { describe, expect, it } from "vitest";
import { createCoupledGripState, resolveCoupledGrip, stepCoupledGrip } from "../../src/effectors/coupledGrip";
import { getMujoco } from "../../src/scenario/mujocoModule";
import type { Vec3 } from "../../src/scenario/schema";
const zero: Vec3 = [0,0,0];
const p = resolveCoupledGrip({ max_force_n: 100 });
const input = { t: 0, dt: .002, engage_s: 0, release_s: 2, hand_m: zero, hand_velocity_mps: zero, grip_m: zero, grip_velocity_mps: zero };
describe("agarre físico: adquisición, capacidad finita y suelta", () => {
  it("no tira a distancia; falla una sola vez y no vuelve a agarrar", () => {
    const r = stepCoupledGrip(p, createCoupledGripState(), { ...input, t: .4, hand_m: [1,0,0] });
    expect(r.state.status).toBe("missed"); expect(r.force_n).toEqual(zero);
    const next = stepCoupledGrip(p, r.state, { ...input, t: .5 });
    expect(next.state.status).toBe("missed"); expect(next.event).toBeUndefined();
  });
  it("espera la proximidad real y la activación antes de cerrar", () => {
    const early = stepCoupledGrip(p, createCoupledGripState(), { ...input, engage_s: .5 });
    expect(early.state.status).toBe("waiting");
    const closed = stepCoupledGrip(p, early.state, { ...input, t: .5, engage_s: .5, hand_m: [.05,0,0] });
    expect(closed.event).toBe("held"); expect(closed.force_n).toEqual(zero);
    expect(closed.state.rest_length_m).toBeCloseTo(.07);
  });
  it("usa velocidad relativa: trasladar ambos puntos no cambia la demanda", () => {
    const state = { status: "held" as const, rest_length_m: .02, saturated_s: 0 };
    const a = stepCoupledGrip(p, state, { ...input, hand_m: [.05,0,0] });
    const b = stepCoupledGrip(p, state, { ...input, hand_m: [.05,0,0], hand_velocity_mps: [.1,0,0], grip_velocity_mps: [.1,0,0] });
    expect(b.force_n).toEqual(a.force_n);
    for(let i=0;i<3;i++) expect(a.force_n[i]!+a.reaction_n[i]!).toBe(0);
  });
  it("respeta fuerza y potencia, y suelta por sobrecarga sostenida", () => {
    let state = { status: "held" as const, rest_length_m: .02, saturated_s: 0 };
    const cap = resolveCoupledGrip({ max_force_n: 100, max_power_w: 10 });
    const r = stepCoupledGrip(cap, state, { ...input, hand_m: [.2,0,0], hand_velocity_mps: [2,0,0], grip_velocity_mps: [2,0,0] });
    expect(r.power_limited).toBe(true); expect(r.force_n[0]).toBe(5); expect(r.power_on_actor_w).toBe(10);
    let last = r;
    for(let i=0;i<110;i++) {
      last = stepCoupledGrip(p, state, { ...input, t:i*.002, hand_m:[.2,0,0] });
      state = last.state as typeof state;
      expect(Math.hypot(...last.force_n)).toBeLessThanOrEqual(100);
    }
    expect(last.state.status).toBe("slipped"); expect(last.force_n).toEqual(zero);
  });
  it("suelta por separación excesiva y por instante programado", () => {
    const held = { status:"held" as const, rest_length_m:.02, saturated_s:0 };
    expect(stepCoupledGrip(p, held, {...input,hand_m:[.3,0,0]}).state.status).toBe("slipped");
    expect(stepCoupledGrip(p, held, {...input,t:2}).state.status).toBe("released");
  });
  it("rechaza límites no finitos y una distancia de rotura menor que la captura", () => {
    for(const key of ["max_force_n","max_power_w","capture_radius_m","break_distance_m","slip_after_s"] as const) {
      expect(()=>resolveCoupledGrip({max_force_n:100,[key]:NaN})).toThrow();
      expect(()=>resolveCoupledGrip({max_force_n:100,[key]:-1})).toThrow();
    }
    expect(()=>resolveCoupledGrip({max_force_n:100,capture_radius_m:.3})).toThrow(/break_distance/);
  });
});
it("dos cuerpos libres: el muelle redistribuye el impulso y disipa energía (WASM real, tres timesteps)", async () => {
  const mj=await getMujoco();
  for(const dt of [.001,.002,.004]) {
    const m=mj.MjModel.from_xml_string(`<mujoco><option timestep="${dt}" gravity="0 0 0"/><worldbody><body name="a" pos="0 0 0"><freejoint/><geom type="sphere" size=".02" mass="2" contype="0" conaffinity="0"/></body><body name="b" pos=".08 0 0"><freejoint/><geom type="sphere" size=".02" mass="5" contype="0" conaffinity="0"/></body></worldbody></mujoco>`);
    const d=new mj.MjData(m);
    try {
      d.qvel[0]=-.2; d.qvel[6]=.1;
      let state = { status:"held" as const, rest_length_m:.04, saturated_s:0 };
      const params=resolveCoupledGrip({max_force_n:500,break_distance_m:1});
      const momentum=2*d.qvel[0]!+5*d.qvel[6]!;
      const initial=.5*2*.2**2+.5*5*.1**2+.5*2500*.04**2;
      let maximumEnergy=initial;
      for(let i=0;i<Math.round(.8/dt);i++) {
        mj.mj_forward(m,d);
        const r=stepCoupledGrip(params,state,{...input,t:i*dt,dt,hand_m:[d.qpos[0]!,0,0],grip_m:[d.qpos[7]!,0,0],hand_velocity_mps:[d.qvel[0]!,0,0],grip_velocity_mps:[d.qvel[6]!,0,0]});
        state=r.state as typeof state;
        d.xfrc_applied.fill(0);d.xfrc_applied[6]=r.reaction_n[0];d.xfrc_applied[12]=r.force_n[0];
        mj.mj_step(m,d);
        expect(2*d.qvel[0]!+5*d.qvel[6]!).toBeCloseTo(momentum,10);
        const energy=.5*2*d.qvel[0]!**2+.5*5*d.qvel[6]!**2+.5*2500*Math.max(0,Math.abs(d.qpos[7]!-d.qpos[0]!)-.04)**2;
        maximumEnergy=Math.max(maximumEnergy,energy);
      }
      expect(maximumEnergy).toBeLessThanOrEqual(initial*1.001);
    } finally {d.delete();m.delete();}
  }
});
