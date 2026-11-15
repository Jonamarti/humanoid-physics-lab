/** Experimental protective response. Sensor-triggered joint targets, always bounded.
 * This is a classical control heuristic, not validated ukemi or an outcome rule.
 */
import type { StandingModel, StandingState } from "../control/standingBaseline";
import type { HumanoidActor, Vec3 } from "../scenario/schema";
import { jointGroupOf, scaledGains } from "./gains";
import { jointRangeRad } from "./rangeOfMotion";
import { UKE_HINGE_JOINTS } from "./stances";

export interface FallObservation {
  t: number; dt: number; capture_margin_m: number | null;
  com_velocity_mps: Vec3; tilt_deg: number; heading_rad: number;
  nonfoot_ground_contact: boolean;
}
export class FallReflex {
  private outsideSince: number | undefined;
  private activatedAt: number | undefined;
  private impactAt: number | undefined;
  private direction = 0;
  private maxRatio = 0;
  private readonly joints;
  constructor(actor: HumanoidActor, model: StandingModel) {
    const gains = scaledGains(actor.mass_kg, actor.height_m, actor.strength);
    this.joints = UKE_HINGE_JOINTS.map(name => {
      const id = model.jnt(name).id;
      return { name, q: model.jnt_qposadr[id]!, v: model.jnt_dofadr[id]!,
        gain: gains.groups[jointGroupOf(name)], range: jointRangeRad(name) };
    });
  }
  step(data: StandingState, o: FallObservation): void {
    const margin = o.capture_margin_m;
    if (this.activatedAt === undefined) {
      // Capture outside alone is insufficient: a recoverable 25 N.s forward push also leaves support.
      // Require sustained loss of support AND actual downward COM motion.
      const diverging = margin !== null && margin < -0.05 &&
        (o.com_velocity_mps[2] < -0.15 || (o.tilt_deg > 30 && o.com_velocity_mps[2] < -0.05));
      const airborneFall = margin === null && o.tilt_deg > 30 && o.com_velocity_mps[2] < -0.2;
      if (diverging || airborneFall) this.outsideSince ??= o.t;
      else this.outsideSince = undefined;
      // 100 ms sustained evidence + 150 ms reaction latency; no fall is declared here.
      if (this.outsideSince !== undefined && o.t - this.outsideSince >= 0.25) {
        this.activatedAt = o.t;
        this.direction = Math.atan2(o.com_velocity_mps[1], o.com_velocity_mps[0]) - o.heading_rad;
      }
    }
    if (this.activatedAt === undefined) return;
    if (o.nonfoot_ground_contact) this.impactAt ??= o.t;
    const ramp = Math.min(1, Math.max(0, (o.t - this.activatedAt) / 0.12));
    const relax = this.impactAt === undefined ? 1 : 1 - 0.7 * Math.min(1, (o.t - this.impactAt) / 0.2);
    const forward = Math.max(0, Math.cos(this.direction));
    const lateral = Math.sin(this.direction);
    const targets: Record<string, number> = {
      neck_pitch: 0.55 * (1 - forward) - 0.2 * forward, lumbar_pitch: 0.45 * (1 - forward), thorax_pitch: 0.25 * (1 - forward),
      left_hip_pitch: -0.5, right_hip_pitch: -0.5, left_knee: -0.95, right_knee: -0.95,
      left_shoulder_pitch: -1.5 * forward + 0.25 * (1 - forward),
      right_shoulder_pitch: -1.5 * forward + 0.25 * (1 - forward),
      left_shoulder_roll: 0.55 + 0.25 * Math.max(0, lateral),
      right_shoulder_roll: -0.55 - 0.25 * Math.max(0, -lateral),
      left_elbow: -0.2, right_elbow: -0.2,
    };
    for (const j of this.joints) {
      const desired = Math.min(j.range[1], Math.max(j.range[0], targets[j.name] ?? 0));
      // Damping already lives in model.dof_damping (implicit). No duplicate explicit damping.
      const requested = j.gain.kp * relax * (desired - data.qpos[j.q]!);
      const reflex = Math.max(-j.gain.maxNm, Math.min(j.gain.maxNm, requested));
      const blended = (1 - ramp) * data.qfrc_applied[j.v]! + ramp * reflex;
      data.qfrc_applied[j.v] = Math.max(-j.gain.maxNm, Math.min(j.gain.maxNm, blended));
      this.maxRatio = Math.max(this.maxRatio, Math.abs(data.qfrc_applied[j.v]!) / j.gain.maxNm);
    }
  }
  status(): Record<string, unknown> {
    return { phase: this.activatedAt === undefined ? "posture" : this.impactAt === undefined ? "protect" : "settle",
      activated_s: this.activatedAt ?? null, impact_s: this.impactAt ?? null, max_torque_ratio: this.maxRatio };
  }
}
