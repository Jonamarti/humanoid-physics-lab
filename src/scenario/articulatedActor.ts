/** Runtime and observation of the second articulated actor. No root pose tracking. */
import type { StandingModel, StandingState } from "../control/standingBaseline";
import { capturePoint, signedMargin, supportPolygon } from "../metrics/balance";
import { classifyHumanoidTrajectory, fallDirectionDeg, tiltDeg, FOOT_BODIES, type BalanceFrame, type GroundTouch } from "../metrics/outcome";
import type { ActorSample, ActorSummary, ContactSample } from "../recording/schema";
import { createUkeController, type UkeController, type UkeControllerParams } from "../uke/attitudes";
import { attitudeJointDamping, setJointDamping, type DampingModel } from "../uke/damping";
import { FallReflex } from "../uke/fallReflex";
import { applyStance, UKE_HINGE_JOINTS } from "../uke/stances";
import { ArtifactProbe } from "./artifactProbe";
import type { MujocoModule } from "./mujocoModule";
import { ACTOR_BODIES, type HumanoidActor, type Vec3 } from "./schema";
type Model = InstanceType<MujocoModule["MjModel"]>;
type Data = InstanceType<MujocoModule["MjData"]>;
const triple = (a: ArrayLike<number>, i: number): Vec3 => [a[i]!, a[i + 1]!, a[i + 2]!];
export class ArticulatedActor {
  private controller: UkeController;
  private readonly params: UkeControllerParams;
  private readonly root: number;
  private readonly rootDof: number;
  private readonly chest: number;
  private readonly sole: {
    left: number;
    right: number;
  };
  private readonly joints: Array<{
    name: string;
    dof: number;
  }>;
  private readonly probe: ArtifactProbe;
  private readonly reflex?: FallReflex;
  private readonly frames: BalanceFrame[] = [];
  private readonly touches: GroundTouch[] = [];
  private readonly touched = new Set<string>();
  private readonly bodyNames: Map<number, string>;
  private initialPelvis = 0;
  private initialCom = 0;
  private settleSpeed = 0;
  private heading = 0;
  private minPelvis = Infinity;
  private maxTilt = 0;
  private rootForce = 0;
  constructor(private readonly mj: MujocoModule, private readonly model: Model, private readonly data: Data, readonly actor: HumanoidActor, readonly prefix: string, bodies: string[], joints: string[], private readonly dt: number) {
    this.bodyNames = new Map(ACTOR_BODIES.humanoid.map(name => [model.body(prefix + name).id, name]));
    this.root = model.body(prefix + "pelvis").id;
    this.rootDof = model.jnt_dofadr[model.jnt(prefix + "pelvis_root").id]!;
    this.chest = model.body(prefix + "chest").id;
    this.sole = { left: model.site(prefix + "left_sole").id, right: model.site(prefix + "right_sole").id };
    this.joints = UKE_HINGE_JOINTS.map(name => ({ name, dof: model.jnt_dofadr[model.jnt(prefix + name).id]! }));
    const placement = applyStance(mj, model, data, actor.stance, prefix);
    setJointDamping(model as unknown as DampingModel, attitudeJointDamping("tense", actor.mass_kg, actor.height_m, actor.strength), prefix);
    mj.mj_forward(model, data);
    mj.mj_subtreeVel(model, data);
    this.params = { ...actor, attitude: "tense", targets: placement.targets, prefix, scope_to_actor: true };
    this.controller = createUkeController(this.params, model as unknown as StandingModel, data as unknown as StandingState);
    this.probe = new ArtifactProbe(mj, model, data, { bodies, joints, actorBodies: ACTOR_BODIES.humanoid.map(b => prefix + b), humanoid: true, dt, prefix, rootDof: this.rootDof, energyScope: "world", measureJointLimits: true });
    if (actor.fall === "instinctive")
      this.reflex = new FallReflex(actor, model as unknown as StandingModel, prefix);
  }
  control(t: number): void {
    this.controller.step(this.model as unknown as StandingModel, this.data as unknown as StandingState, t);
    for (let k = 0; k < 6; k++)
      this.rootForce = Math.max(this.rootForce, Math.abs(this.data.qfrc_applied[this.rootDof + k]!));
    if (this.rootForce !== 0)
      throw new Error("El controlador secundario escribió fuerza sobre su pelvis libre");
  }
  protect(t: number, contacts: ContactSample[]): void {
    if (this.reflex) {
      const com = triple(this.data.subtree_com, 3 * this.root), velocity = triple(this.data.subtree_linvel, 3 * this.root);
      this.reflex.step(this.data as unknown as StandingState, { t, dt: this.dt, com_velocity_mps: velocity,
        capture_margin_m: signedMargin(capturePoint(com, velocity), supportPolygon(contacts, b => FOOT_BODIES.some(f => b === this.prefix + f))),
        tilt_deg: Math.max(tiltDeg(this.data.xmat, 9 * this.chest), tiltDeg(this.data.xmat, 9 * this.root)), heading_rad: this.heading,
        nonfoot_ground_contact: contacts.some(c => [c.body_a, c.body_b].includes("world") && [c.body_a, c.body_b].some(b => b.startsWith(this.prefix) && !FOOT_BODIES.some(f => b === this.prefix + f))),
      });
    }
  }
  start(): void {
    this.mj.mj_forward(this.model, this.data);
    this.mj.mj_subtreeVel(this.model, this.data);
    if (this.actor.attitude !== "tense") {
      this.controller = createUkeController({ ...this.params, attitude: this.actor.attitude }, this.model as unknown as StandingModel, this.data as unknown as StandingState);
      setJointDamping(this.model as unknown as DampingModel, attitudeJointDamping(this.actor.attitude, this.actor.mass_kg, this.actor.height_m, this.actor.strength), this.prefix);
    }
    this.initialPelvis = this.data.xpos[3 * this.root + 2]!;
    this.initialCom = this.data.subtree_com[3 * this.root + 2]!;
    this.settleSpeed = Math.hypot(...triple(this.data.subtree_linvel, 3 * this.root));
    this.heading = Math.atan2(this.data.xmat[9 * this.root + 3]!, this.data.xmat[9 * this.root]!);
  }
  state(t: number): void {
    this.probe.state(t);
    this.minPelvis = Math.min(this.minPelvis, this.data.xpos[3 * this.root + 2]!);
    this.maxTilt = Math.max(this.maxTilt, tiltDeg(this.data.xmat, 9 * this.chest));
  }
  sample(t: number, contacts: ContactSample[], externalWork: number): ActorSample {
    this.probe.sample(t, externalWork);
    this.mj.mj_subtreeVel(this.model, this.data);
    const com = triple(this.data.subtree_com, 3 * this.root), velocity = triple(this.data.subtree_linvel, 3 * this.root);
    const xcom = capturePoint(com, velocity), polygon = supportPolygon(contacts, b => FOOT_BODIES.some(f => b === this.prefix + f));
    const result: ActorSample = { com, com_velocity_mps: velocity, angular_momentum_nms: triple(this.data.subtree_angmom, 3 * this.root),
      joint_torques_nm: Object.fromEntries(this.joints.map(j => [j.name, this.data.qfrc_applied[j.dof]!])), ground_reaction_n: { left: [0, 0, 0], right: [0, 0, 0] },
      xcom: [xcom[0], xcom[1]], support_polygon: polygon.map(p => [p[0], p[1]]), capture_margin_m: signedMargin(xcom, polygon) };
    for (const side of ["left", "right"] as const) {
      const foot = this.prefix + side + "_foot";
      for (const c of contacts) {
        const sign = c.body_a === "world" && c.body_b === foot ? 1 : c.body_b === "world" && c.body_a === foot ? -1 : 0;
        result.ground_reaction_n[side] = result.ground_reaction_n[side].map((v, k) => v + sign * c.force_n[k]!) as unknown as Vec3;
      }
    }
    const sites = this.data.site_xpos;
    this.frames.push({ t, com, com_velocity: velocity, xcom, support_polygon: polygon, pelvis_z: this.data.xpos[3 * this.root + 2]!,
      feet: { left: [sites[3 * this.sole.left]!, sites[3 * this.sole.left + 1]!], right: [sites[3 * this.sole.right]!, sites[3 * this.sole.right + 1]!] },
      trunk_dir_deg: fallDirectionDeg(this.data.xmat, 9 * this.chest) });
    return result;
  }
  beforeStep(): void { this.probe.beforeStep(); }
  afterStep(t: number): void {
    this.probe.afterStep(t);
    const contacts = this.data.contact;
    try {
      for (let i = 0; i < contacts.size(); i++) {
        const c = contacts.get(i);
        if (!c)
          throw new Error("Contacto inaccesible");
        try {
          const a = this.model.geom_bodyid[c.geom1]!, b = this.model.geom_bodyid[c.geom2]!;
          const body = a === 0 ? b : b === 0 ? a : -1;
          if (body < 0)
            continue;
          const semantic = this.bodyNames.get(body);
          if (!semantic || FOOT_BODIES.includes(semantic) || this.touched.has(semantic))
            continue;
          this.touched.add(semantic);
          const ref = triple(this.data.subtree_com, 3 * this.root), p = triple(c.pos, 0), w = triple(this.data.cvel, 6 * body), v = triple(this.data.cvel, 6 * body + 3);
          const r = [p[0] - ref[0], p[1] - ref[1], p[2] - ref[2]];
          this.touches.push({ body: semantic, t_s: t, velocity_mps: [v[0] + w[1] * r[2]! - w[2] * r[1]!, v[1] + w[2] * r[0]! - w[0] * r[2]!, v[2] + w[0] * r[1]! - w[1] * r[0]!] });
        }
        finally {
          c.delete();
        }
      }
    }
    finally {
      contacts.delete();
    }
  }
  summary(): ActorSummary {
    const verdict = classifyHumanoidTrajectory({ frames: this.frames, touches: this.touches, initial_pelvis_z: this.initialPelvis, heading_rad: this.heading });
    return { outcome: verdict.outcome, t_outcome_s: verdict.t_outcome_s, max_trunk_tilt_deg: this.maxTilt, initial_pelvis_height_m: this.initialPelvis,
      min_pelvis_height_m: this.minPelvis, initial_com_height_m: this.initialCom, settle_com_speed_mps: this.settleSpeed, max_root_generalized_force: this.rootForce,
      controller: { ...this.controller.status(), ...(this.reflex ? { fall_response: this.reflex.status() } : {}) }, balance: verdict.balance, fall: verdict.fall, step: verdict.step, unbalance: verdict.unbalance, diagnostics: this.probe.diagnostics() };
  }
}
