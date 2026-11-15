/** Compliant grip between two simulated points. No pose targets or root forces. */
import type { Vec3 } from "../scenario/schema";

export type GripStateName = "waiting" | "held" | "missed" | "slipped" | "released";
export interface CoupledGripOptions {
  max_force_n: number;
  max_power_w?: number;
  capture_radius_m?: number;
  capture_window_s?: number;
  break_distance_m?: number;
  slip_after_s?: number;
}
export interface CoupledGripParams {
  max_force_n: number; max_power_w: number; capture_radius_m: number; capture_window_s: number;
  break_distance_m: number; slip_after_s: number; k_n_per_m: number; c_ns_per_m: number; slack_m: number;
}
export interface CoupledGripState {
  status: GripStateName;
  rest_length_m: number;
  saturated_s: number;
}
export interface CoupledGripInput {
  t: number; dt: number; engage_s: number; release_s: number;
  hand_m: Vec3; hand_velocity_mps: Vec3; grip_m: Vec3; grip_velocity_mps: Vec3;
}
export interface CoupledGripResult {
  state: CoupledGripState; force_n: Vec3; reaction_n: Vec3; distance_m: number;
  force_limited: boolean; power_limited: boolean; power_on_actor_w: number;
  event?: GripStateName;
}
export function resolveCoupledGrip(o: CoupledGripOptions): CoupledGripParams {
  const p: CoupledGripParams = { max_force_n: o.max_force_n, max_power_w: o.max_power_w ?? 600,
    capture_radius_m: o.capture_radius_m ?? 0.12, capture_window_s: o.capture_window_s ?? 0.35,
    break_distance_m: o.break_distance_m ?? 0.25, slip_after_s: o.slip_after_s ?? 0.2,
    k_n_per_m: 2500, c_ns_per_m: 80, slack_m: 0.02 };
  for (const [key, value] of Object.entries(p)) {
    if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${key}: debe ser finito y > 0`);
  }
  if (p.break_distance_m <= p.capture_radius_m) throw new RangeError("break_distance_m debe superar capture_radius_m");
  return p;
}
export function createCoupledGripState(): CoupledGripState {
  return { status: "waiting", rest_length_m: 0, saturated_s: 0 };
}
/** Pure step: caller applies BOTH forces at grip_m, so net force and moment are zero.
 * Damping uses relative velocity; the elastic force is central and has a finite cap.
 * Acquisition waits for a real wrist within reach; terminal failures never reacquire.
 */
export function stepCoupledGrip(p: CoupledGripParams, previous: CoupledGripState, s: CoupledGripInput): CoupledGripResult {
  if (!(Number.isFinite(s.t) && Number.isFinite(s.dt) && s.dt > 0) ||
      ![...s.hand_m, ...s.hand_velocity_mps, ...s.grip_m, ...s.grip_velocity_mps].every(Number.isFinite)) {
    throw new RangeError("Estado o paso temporal del agarre no finito");
  }
  const state = { ...previous };
  const delta = [s.hand_m[0]-s.grip_m[0], s.hand_m[1]-s.grip_m[1], s.hand_m[2]-s.grip_m[2]] as Vec3;
  const distance = Math.hypot(...delta);
  let event: GripStateName | undefined;
  if ((state.status === "held" || state.status === "waiting") && s.t >= s.release_s) {
    state.status = "released"; event = "released";
  } else if (state.status === "waiting" && s.t >= s.engage_s) {
    if (distance <= p.capture_radius_m) {
      state.status = "held"; state.rest_length_m = distance + p.slack_m; event = "held";
    } else if (s.t >= s.engage_s + p.capture_window_s) {
      state.status = "missed"; event = "missed";
    }
  }
  let magnitude = 0, forceLimited = false, powerLimited = false;
  const dir = distance > 1e-12 ? [delta[0]/distance, delta[1]/distance, delta[2]/distance] as Vec3 : [0, 0, 0] as Vec3;
  if (state.status === "held") {
    if (distance > p.break_distance_m) { state.status = "slipped"; event = "slipped"; }
    else {
      const extension = Math.max(0, distance - state.rest_length_m);
      const relative = dir.reduce((v, x, i) => v + x * (s.hand_velocity_mps[i]! - s.grip_velocity_mps[i]!), 0);
      const requested = extension > 0 ? Math.max(0, p.k_n_per_m * extension + p.c_ns_per_m * relative) : 0;
      forceLimited = requested >= p.max_force_n;
      state.saturated_s = forceLimited ? state.saturated_s + s.dt : 0;
      if (state.saturated_s + 1e-12 >= p.slip_after_s) { state.status = "slipped"; event = "slipped"; }
      else {
        magnitude = Math.min(requested, p.max_force_n);
        const velocityAlong = dir.reduce((v, x, i) => v + x * s.grip_velocity_mps[i]!, 0);
        if (magnitude * velocityAlong > p.max_power_w) {
          magnitude = p.max_power_w / velocityAlong; powerLimited = true;
        }
      }
    }
  }
  const force = [dir[0]*magnitude, dir[1]*magnitude, dir[2]*magnitude] as Vec3;
  return { state, force_n: force, reaction_n: [-force[0], -force[1], -force[2]], distance_m: distance,
    force_limited: forceLimited, power_limited: powerLimited,
    power_on_actor_w: force.reduce((v, x, i) => v + x * s.grip_velocity_mps[i]!, 0),
    ...(event ? { event } : {}) };
}
