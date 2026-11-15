/** Finite compliant grips with real reaction wrenches on both articulated bodies. */
import { createCoupledGripState, resolveCoupledGrip, stepCoupledGrip } from "../effectors/coupledGrip";
import type { EffectorPeaks, EffectorSample } from "../recording/schema";
import type { MujocoModule } from "./mujocoModule";
import type { ActorGripSpec, Vec3 } from "./schema";
type Model = InstanceType<MujocoModule["MjModel"]>;
type Data = InstanceType<MujocoModule["MjData"]>;
const triple = (a: ArrayLike<number>, o: number): Vec3 => [a[o]!, a[o + 1]!, a[o + 2]!];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export class ArticulatedGrips {
  private readonly grips;
  constructor(private readonly model: Model, private readonly data: Data, specs: ActorGripSpec[], prefixOf: (id: string) => string, peaks: Record<string, EffectorPeaks>) {
    this.grips = specs.map(spec => {
      const sourceSite = model.site(prefixOf(spec.source.actor) + spec.source.site).id, targetSite = model.site(prefixOf(spec.target.actor) + spec.target.site).id;
      const peak: EffectorPeaks = { max_force_n: 0, impulse_ns: [0, 0, 0], work_j: 0, reaction_work_j: 0, limit_n: spec.max_force_n, max_power_w: 0, grip_events: [] };
      peaks[spec.id] = peak;
      return { spec, sourceSite, targetSite, sourceBody: model.site_bodyid[sourceSite]!, targetBody: model.site_bodyid[targetSite]!, params: resolveCoupledGrip(spec), state: createCoupledGripState(), peak,
        previous: undefined as {
          force: Vec3;
          targetVelocity: Vec3;
          sourceVelocity: Vec3;
        } | undefined };
    });
  }
  private velocity(body: number, point: Vec3): Vec3 {
    const d = this.data, root = this.model.body_rootid[body]!;
    return add(triple(d.cvel, 6 * body + 3), cross(triple(d.cvel, 6 * body), sub(point, triple(d.subtree_com, 3 * root))));
  }
  private apply(body: number, point: Vec3, force: Vec3): void {
    const torque = cross(sub(point, triple(this.data.xipos, 3 * body)), force), f = this.data.xfrc_applied;
    for (let k = 0; k < 3; k++) {
      f[6 * body + k] += force[k]!;
      f[6 * body + 3 + k] += torque[k]!;
    }
  }
  step(t: number, dt: number, count: boolean): EffectorSample[] {
    return this.grips.map(g => {
      const hand = triple(this.data.site_xpos, 3 * g.sourceSite), point = triple(this.data.site_xpos, 3 * g.targetSite);
      const targetVelocity = this.velocity(g.targetBody, point), sourceVelocity = this.velocity(g.sourceBody, point);
      if (g.previous) {
        g.peak.work_j += dot(g.previous.force, add(g.previous.targetVelocity, targetVelocity)) * dt / 2;
        g.peak.reaction_work_j! -= dot(g.previous.force, add(g.previous.sourceVelocity, sourceVelocity)) * dt / 2;
      }
      const r = stepCoupledGrip(g.params, g.state, { t, dt, engage_s: g.spec.engage_s, release_s: g.spec.release_s ?? Infinity, hand_m: hand, hand_velocity_mps: this.velocity(g.sourceBody, hand), grip_m: point, grip_velocity_mps: targetVelocity });
      // Both wrenches share one world point: exactly zero resultant force and moment.
      this.apply(g.targetBody, point, r.force_n);
      this.apply(g.sourceBody, point, r.reaction_n);
      if (count) {
        g.state = r.state;
        g.peak.max_force_n = Math.max(g.peak.max_force_n, Math.hypot(...r.force_n));
        g.peak.max_power_w = Math.max(g.peak.max_power_w!, r.power_on_actor_w);
        g.peak.impulse_ns = add(g.peak.impulse_ns, r.force_n.map(v => v * dt) as unknown as Vec3);
        if (r.event)
          g.peak.grip_events!.push({ t_s: t, status: r.event });
      }
      g.previous = count ? { force: r.force_n, targetVelocity, sourceVelocity } : undefined;
      return { id: g.spec.id, point_m: point, force_n: r.force_n, coupled_grip: { status: r.state.status, hand_point_m: hand, distance_m: r.distance_m, reaction_n: r.reaction_n, force_limited: r.force_limited, power_limited: r.power_limited } };
    });
  }
}
