/**
 * Sonda del detector de artefactos (issue #75 PR 5, solo Node): acumula en cada paso físico (500 Hz) lo que no se puede sacar de
 * las muestras a `record_hz` y lo entrega como `RecordingDiagnostics` (`src/recording/schema.ts`). La evaluación contra los
 * umbrales la hace `src/metrics/artifacts.ts`. Qué mide cada magnitud y sus límites: `docs/ARTIFACTS.md`.
 *
 * Solo lee MuJoCo: no escribe en `data` ni cambia la física. El ejecutor la llama en cuatro puntos:
 * `state(t)` al empezar cada iteración, `beforeStep()` y `afterStep(t)` alrededor de `mj_step` y `sample(...)` en cada muestra.
 */
import { decodeSolvedContactForce, hasCompleteSolvedContactRows } from "../physics/contactForces";
import { FRICTION_SATURATED_UTILIZATION, FOOT_LOADED_WEIGHT_FRACTION, TREMOR_MIN_AMPLITUDE_RAD, TREMOR_WINDOW_S } from "../metrics/artifacts";
import type { PeakAt, RecordingDiagnostics } from "../recording/schema";
import type { MujocoModule } from "./mujocoModule";

type Mj = MujocoModule;
type Model = InstanceType<Mj["MjModel"]>;
type Data = InstanceType<Mj["MjData"]>;

const GRAVITY = 9.81;
/** Tipo de articulación bisagra de MuJoCo (`mjJNT_HINGE`). */
const JNT_HINGE = 3;
const FOOT_BODIES = ["left_foot", "right_foot"] as const;

export interface ProbeContext {
  /** Nombres de los cuerpos en el orden de MuJoCo. */
  bodies: readonly string[];
  /** Nombres de las articulaciones en el orden de MuJoCo. */
  joints: readonly string[];
  /** Cuerpos del actor (`ACTOR_BODIES`). */
  actorBodies: readonly string[];
  humanoid: boolean;
  /** Paso físico, en s. */
  dt: number;
  /** Amplitud mínima del temblor; por defecto la del detector (parámetro solo para medir sensibilidad). */
  tremorAmplitudeRad?: number;
}

interface JointTrack {
  name: string;
  dof: number;
  qposAdr: number;
  dir: 0 | 1 | -1;
  cand: number;
  started: boolean;
  reversals: number[];
}

interface FootTrack {
  name: string;
  bodyId: number;
  /** Desplazamiento neto acumulado en el tramo continuo cargado y sin rozamiento saturado, en m. */
  dx: number;
  dy: number;
  load: number;
  weightedVx: number;
  weightedVy: number;
  saturated: boolean;
  seenContact: boolean;
}

function peak(): PeakAt {
  return { value: 0, t_s: 0, where: "" };
}

function raise(p: PeakAt, value: number, t: number, where: string): void {
  if (value > p.value) {
    p.value = value;
    p.t_s = t;
    p.where = where;
  }
}

export class ArtifactProbe {
  private readonly mj: Mj;
  private readonly model: Model;
  private readonly data: Data;
  private readonly ctx: ProbeContext;
  private readonly isActor: boolean[];
  private readonly floorGeom: number;
  private readonly pyramidal: boolean;
  private readonly amplitude: number;
  private readonly massKg: number;
  private readonly joints: JointTrack[] = [];
  private readonly feet: FootTrack[] = [];
  private readonly groundPen = peak();
  private readonly segmentPen = peak();
  private readonly jointSpeed = peak();
  private readonly contactForce = peak();
  private readonly footSlip = peak();
  private readonly tremor = peak();
  private rootForce = 0;
  private qvelBefore: Float64Array;
  private qfrcBefore: Float64Array;
  private torqueWorkJ = 0;
  private energy0: number | undefined;
  private energyScaleJ = 0;
  private energyExcess = { max_excess_j: 0, t_s: 0 };
  private energySamples = 0;
  private armed = false;

  constructor(mj: Mj, model: Model, data: Data, ctx: ProbeContext) {
    this.mj = mj;
    this.model = model;
    this.data = data;
    this.ctx = ctx;
    this.amplitude = ctx.tremorAmplitudeRad ?? TREMOR_MIN_AMPLITUDE_RAD;
    const names = new Set(ctx.actorBodies);
    this.isActor = ctx.bodies.map((b) => names.has(b));
    this.floorGeom = model.geom("floor").id;
    this.pyramidal = Boolean(mj.mj_isPyramidal(model));
    const mass = model.body_mass as Float64Array;
    this.massKg = ctx.bodies.reduce((s, _, i) => s + (this.isActor[i] ? mass[i]! : 0), 0);
    const jtype = model.jnt_type as Int32Array;
    const dofAdr = model.jnt_dofadr as Int32Array;
    const qAdr = model.jnt_qposadr as Int32Array;
    ctx.joints.forEach((name, j) => {
      if (jtype[j] !== JNT_HINGE) return;
      const body = (model.jnt_bodyid as Int32Array)[j]!;
      if (!this.isActor[body]) return;
      this.joints.push({ name, dof: dofAdr[j]!, qposAdr: qAdr[j]!, dir: 0, cand: 0, started: false, reversals: [] });
    });
    if (ctx.humanoid) {
      for (const name of FOOT_BODIES) {
        this.feet.push({
          name, bodyId: ctx.bodies.indexOf(name), dx: 0, dy: 0, load: 0, weightedVx: 0, weightedVy: 0, saturated: false, seenContact: false,
        });
      }
    }
    this.qvelBefore = new Float64Array(model.nv);
    this.qfrcBefore = new Float64Array(model.nv);
  }

  /** Estado al principio de la iteración (`t >= 0`, ya con `mj_kinematics`/`mj_comVel`): velocidad y temblor de las articulaciones. */
  state(t: number): void {
    if (this.joints.length === 0) return;
    const qvel = this.data.qvel as Float64Array;
    const qpos = this.data.qpos as Float64Array;
    for (const j of this.joints) {
      raise(this.jointSpeed, Math.abs(qvel[j.dof]!), t, j.name);
      this.trackTremor(j, qpos[j.qposAdr]!, t);
    }
  }

  private trackTremor(j: JointTrack, q: number, t: number): void {
    const a = this.amplitude;
    let reversed = false;
    if (!j.started) {
      j.started = true;
      j.cand = q;
    } else if (j.dir === 0) {
      if (q - j.cand >= a) { j.dir = 1; j.cand = q; } else if (j.cand - q >= a) { j.dir = -1; j.cand = q; }
    } else if (j.dir === 1) {
      if (q > j.cand) j.cand = q;
      else if (j.cand - q >= a) { reversed = true; j.dir = -1; j.cand = q; }
    } else if (q < j.cand) j.cand = q;
    else if (q - j.cand >= a) { reversed = true; j.dir = 1; j.cand = q; }
    if (!reversed) return;
    j.reversals.push(t);
    while (j.reversals.length > 0 && j.reversals[0]! <= t - TREMOR_WINDOW_S + 1e-9) j.reversals.shift();
    raise(this.tremor, j.reversals.length, t, j.name);
  }

  /** Justo antes de `mj_step` (con los pares ya escritos): guarda velocidades y pares para el trabajo de los pares internos. */
  beforeStep(): void {
    this.qvelBefore.set(this.data.qvel as Float64Array);
    this.qfrcBefore.set(this.data.qfrc_applied as Float64Array);
    if (this.ctx.humanoid) {
      for (let k = 0; k < 6; k++) this.rootForce = Math.max(this.rootForce, Math.abs(this.qfrcBefore[k]!));
    }
    this.armed = true;
  }

  private contactPointVelocity(bodyId: number, p: readonly [number, number, number]): [number, number, number] {
    const cvel = this.data.cvel as Float64Array;
    const root = (this.model.body_rootid as Int32Array)[bodyId]!;
    const com = this.data.subtree_com as Float64Array;
    const w: [number, number, number] = [cvel[6 * bodyId]!, cvel[6 * bodyId + 1]!, cvel[6 * bodyId + 2]!];
    const r: [number, number, number] = [p[0] - com[3 * root]!, p[1] - com[3 * root + 1]!, p[2] - com[3 * root + 2]!];
    return [
      cvel[6 * bodyId + 3]! + w[1] * r[2] - w[2] * r[1],
      cvel[6 * bodyId + 4]! + w[2] * r[0] - w[0] * r[2],
      cvel[6 * bodyId + 5]! + w[0] * r[1] - w[1] * r[0],
    ];
  }

  /**
   * Justo después de `mj_step`: los contactos y `efc_force` que quedan son los del estado al inicio del paso (resueltos con los
   * pares y fuerzas de ese paso), el mismo estado en que se evaluó `state(t)`. Mide penetraciones, fuerza de contacto y
   * deslizamiento de los pies, y suma el trabajo de los pares internos del paso.
   */
  afterStep(t: number): void {
    if (!this.armed) return;
    const dt = this.ctx.dt;
    const qvel = this.data.qvel as Float64Array;
    const qfrc = this.qfrcBefore;
    for (let k = 0; k < qfrc.length; k++) {
      if (qfrc[k] !== 0) this.torqueWorkJ += qfrc[k]! * 0.5 * (this.qvelBefore[k]! + qvel[k]!) * dt;
    }

    for (const f of this.feet) {
      f.load = 0; f.weightedVx = 0; f.weightedVy = 0; f.saturated = false; f.seenContact = false;
    }
    const bodyOfGeom = this.model.geom_bodyid as Int32Array;
    const efcForce = this.data.efc_force as Float64Array;
    const contacts = this.data.contact;
    try {
      for (let c = 0; c < contacts.size(); c++) {
        const contact = contacts.get(c);
        if (!contact) throw new Error("Contacto MuJoCo inaccesible");
        try {
          const g1 = contact.geom1;
          const g2 = contact.geom2;
          const b1 = bodyOfGeom[g1]!;
          const b2 = bodyOfGeom[g2]!;
          const a1 = this.isActor[b1]!;
          const a2 = this.isActor[b2]!;
          if (!a1 && !a2) continue;
          const penetration = -contact.dist;
          let floorBody = -1;
          if (g1 === this.floorGeom && a2) floorBody = b2;
          else if (g2 === this.floorGeom && a1) floorBody = b1;
          if (floorBody >= 0) raise(this.groundPen, penetration, t, this.ctx.bodies[floorBody]!);
          else if (a1 && a2 && b1 !== b2) {
            const [lo, hi] = b1 < b2 ? [b1, b2] : [b2, b1];
            raise(this.segmentPen, penetration, t, `${this.ctx.bodies[lo]}|${this.ctx.bodies[hi]}`);
          }
          if (!hasCompleteSolvedContactRows(contact.efc_address, contact.dim, this.pyramidal, this.data.nefc, efcForce.length)) continue;
          const solved = decodeSolvedContactForce({
            solverRows: efcForce, efcAddress: contact.efc_address, dim: contact.dim, pyramidal: this.pyramidal,
            friction: contact.friction as Float64Array, adhesionN: contact.adhesion,
          });
          if (!(solved.normalN > 0)) continue;
          const pair = floorBody >= 0
            ? `${this.ctx.bodies[floorBody]}|world`
            : `${this.ctx.bodies[b1]}|${this.ctx.bodies[b2]}`;
          raise(this.contactForce, solved.normalN, t, pair);
          if (floorBody >= 0) {
            const foot = this.feet.find((f) => f.bodyId === floorBody);
            if (foot) {
              const pos = contact.pos as Float64Array;
              const v = this.contactPointVelocity(floorBody, [pos[0]!, pos[1]!, pos[2]!]);
              foot.seenContact = true;
              foot.load += solved.normalN;
              foot.weightedVx += solved.normalN * v[0];
              foot.weightedVy += solved.normalN * v[1];
              // `null` (sin dato) cuenta como saturado: no se acusa de patinar sin saber si el rozamiento aguantaba.
              if (solved.frictionUtilization === null || solved.frictionUtilization > FRICTION_SATURATED_UTILIZATION) foot.saturated = true;
            }
          }
        } finally {
          contact.delete();
        }
      }
    } finally {
      contacts.delete();
    }

    const loadedN = FOOT_LOADED_WEIGHT_FRACTION * this.massKg * GRAVITY;
    for (const f of this.feet) {
      if (f.seenContact && f.load >= loadedN && !f.saturated) {
        f.dx += (f.weightedVx / f.load) * dt;
        f.dy += (f.weightedVy / f.load) * dt;
        raise(this.footSlip, Math.hypot(f.dx, f.dy), t, f.name);
      } else {
        f.dx = 0; f.dy = 0;
      }
    }
  }

  /**
   * En cada muestra, tras `mj_forward`: energía mecánica (cinética + potencial + resortes de MuJoCo) frente a la aportada.
   * `effectorWorkJ` es el trabajo acumulado de los efectores hasta el paso anterior (`peaks.*.work_j` sumados).
   * Exceso = `(E - E_0) - (trabajo de pares internos + trabajo de efectores)`; lo disipan el amortiguamiento y los contactos, así
   * que sin una fuente oculta nunca debería ser positivo salvo la tolerancia.
   */
  sample(t: number, effectorWorkJ: number): void {
    this.mj.mj_energyPos(this.model, this.data);
    this.mj.mj_energyVel(this.model, this.data);
    const e = this.data.energy as Float64Array;
    const total = e[0]! + e[1]!;
    if (this.energy0 === undefined) {
      this.energy0 = total;
      const xipos = this.data.xipos as Float64Array;
      const mass = this.model.body_mass as Float64Array;
      let mz = 0;
      for (let b = 0; b < this.isActor.length; b++) if (this.isActor[b]) mz += mass[b]! * xipos[3 * b + 2]!;
      this.energyScaleJ = GRAVITY * mz;
    }
    const excess = (total - this.energy0) - (this.torqueWorkJ + effectorWorkJ);
    this.energySamples++;
    if (this.energySamples === 1 || excess > this.energyExcess.max_excess_j) this.energyExcess = { max_excess_j: excess, t_s: t };
  }

  diagnostics(): RecordingDiagnostics {
    const d: RecordingDiagnostics = {
      actor_mass_kg: this.massKg,
      ground_penetration: this.groundPen,
      contact_force: this.contactForce,
    };
    if (this.ctx.humanoid) {
      d.segment_penetration = this.segmentPen;
      d.foot_slip = this.footSlip;
      d.root_generalized_force = this.rootForce;
    }
    if (this.joints.length > 0) {
      d.joint_speed = this.jointSpeed;
      d.tremor = { ...this.tremor, amplitude_rad: this.amplitude };
    }
    if (this.energySamples > 0) d.energy = { ...this.energyExcess, scale_j: this.energyScaleJ };
    return d;
  }
}
