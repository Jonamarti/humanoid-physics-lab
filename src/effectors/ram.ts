/**
 * Efector `ram` (ariete, issue #75 PR 7, solo Node). Diseño y cifras: `docs/GOLPES.md`.
 *
 * Un ariete es un cuerpo libre del MJCF (`ram_<id>`: esfera de `RAM_RADIUS_M`, masa efectiva, `gravcomp="1"` para que vuele en
 * línea recta sin gravedad) que NO aplica ninguna fuerza sobre Uke: solo actúa por contacto. No colisiona con el suelo ni con los
 * otros arietes (`<exclude>`), sí con Uke y con los props. El ejecutor lo coloca al acabar el asentamiento, mide cada paso su
 * contacto con Uke y, `RAM_RETIRE_AFTER_CONTACT_S` después del primer contacto, desactiva su colisión y lo aparta.
 */
import { decodeSolvedContactForce, hasCompleteSolvedContactRows } from "../physics/contactForces";
import type { ContactSample, EffectorPeaks, EffectorSample, RamSummary } from "../recording/schema";
import type { MujocoModule } from "../scenario/mujocoModule";
import { RAM_RADIUS_M, RAM_RETIRE_AFTER_CONTACT_S, type RamEffector, type Vec3 } from "../scenario/schema";
import { unitVector } from "./impulse";

type Mj = MujocoModule;
type Model = InstanceType<Mj["MjModel"]>;
type Data = InstanceType<Mj["MjData"]>;

/** Rozamiento del ariete (deslizante, torsional, de rodadura). Con `priority="1"` manda sobre el de la geometría de Uke (si no, MuJoCo toma el máximo, 1,0). Estimación. */
export const RAM_FRICTION = "0.5 0.005 0.0001";
/** Contacto del ariete: el de las geometrías de Uke (`humanoid_step_v2.xml`). */
const RAM_CONTACT = `solref="0.015 1" solimp="0.9 0.95 0.001"`;
/** Velocidad con que se aparta el ariete tras retirarse, en m/s, en sentido contrario al de su llegada. Solo es estética del vídeo: ya no tiene colisión. */
export const RAM_RETREAT_SPEED_MPS = 2;
/** Los contactos con fuerza normal por debajo de esto (N) no cuentan (igual que las grabaciones). */
const MIN_CONTACT_NORMAL_N = 1e-6;

export const ramBodyName = (id: string): string => `ram_${id}`;
export const ramJointName = (id: string): string => `ram_${id}_free`;
export const ramGeomName = (id: string): string => `ram_${id}_geom`;

const f = (x: number): string => String(Number(x.toPrecision(10)));

/**
 * Inserta los arietes en el MJCF, DESPUÉS de todo lo que ya hay en `<worldbody>`: así la raíz libre de Uke sigue siendo la
 * primera articulación (`qpos[0..7]`, `qvel[0..6]`). Cada ariete nace aparcado a 30 m de altura; el ejecutor lo coloca al
 * acabar el asentamiento. Excluye su colisión con el suelo (cuerpo `world`) y con los otros arietes.
 */
export function injectRams(mjcf: string, rams: readonly Pick<RamEffector, "id" | "mass_kg">[]): string {
  if (rams.length === 0) return mjcf;
  const bodies = rams.map((r, i) => `    <body name="${ramBodyName(r.id)}" pos="${f(0.5 * i)} 0 30" gravcomp="1">
      <freejoint name="${ramJointName(r.id)}"/>
      <geom name="${ramGeomName(r.id)}" type="sphere" size="${f(RAM_RADIUS_M)}" mass="${f(r.mass_kg)}" rgba="0.95 0.55 0.10 1"
            friction="${RAM_FRICTION}" condim="3" priority="1" ${RAM_CONTACT}/>
    </body>`).join("\n");
  const excludes: string[] = [];
  rams.forEach((r, i) => {
    excludes.push(`<exclude body1="${ramBodyName(r.id)}" body2="world"/>`);
    for (const other of rams.slice(i + 1)) excludes.push(`<exclude body1="${ramBodyName(r.id)}" body2="${ramBodyName(other.id)}"/>`);
  });
  if (!mjcf.includes("</worldbody>")) throw new Error("El MJCF no tiene </worldbody>");
  let out = mjcf.replace("</worldbody>", `${bodies}\n  </worldbody>`);
  out = out.includes("</contact>")
    ? out.replace("</contact>", `${excludes.join("\n    ")}\n  </contact>`)
    : out.replace("</mujoco>", `  <contact>\n    ${excludes.join("\n    ")}\n  </contact>\n</mujoco>`);
  return out;
}

function rotate(m: ArrayLike<number>, v: Vec3): Vec3 {
  return [
    m[0]! * v[0] + m[1]! * v[1] + m[2]! * v[2],
    m[3]! * v[0] + m[4]! * v[1] + m[5]! * v[2],
    m[6]! * v[0] + m[7]! * v[1] + m[8]! * v[2],
  ];
}

interface PendingContact {
  bodyId: number;
  /** Punto del contacto en el sistema local del cuerpo de Uke (m). */
  local: Vec3;
  /** Fuerza del ariete sobre ese cuerpo, en el mundo (N). */
  force: Vec3;
  /** Velocidad del punto al inicio del paso (m/s). */
  velocity: Vec3;
}

interface RamState {
  spec: RamEffector;
  bodyId: number;
  geomId: number;
  qposAdr: number;
  dofAdr: number;
  dir: Vec3;
  peaks: EffectorPeaks;
  vStart: Vec3;
  vBefore: Vec3 | null;
  vAfter: Vec3 | null;
  tFirst: number | null;
  tLastEnd: number | null;
  contactSteps: number;
  maxPenetration: number;
  retiredAt: number | null;
  pending: PendingContact[];
  pendingDt: number;
}

export class RamSet {
  private readonly rams: RamState[];
  private readonly byBody = new Map<number, RamState>();
  private readonly pyramidal: boolean;

  /**
   * @param bodies nombres de los cuerpos del modelo en el orden de MuJoCo (`world` primero).
   * @param pointVelocityAt velocidad en el mundo del punto `p` del cuerpo `bodyId` con el `cvel` actual.
   */
  constructor(
    mj: Mj,
    private readonly model: Model,
    private readonly data: Data,
    private readonly bodies: readonly string[],
    specs: readonly RamEffector[],
    private readonly pointVelocityAt: (bodyId: number, p: Vec3) => Vec3,
  ) {
    this.pyramidal = Boolean(mj.mj_isPyramidal(model));
    this.rams = specs.map((spec) => {
      const bodyId = bodies.indexOf(ramBodyName(spec.id));
      if (bodyId < 0) throw new Error(`El modelo no tiene el cuerpo del ariete "${spec.id}"`);
      const jnt = model.jnt(ramJointName(spec.id)).id;
      const peaks: EffectorPeaks = { max_force_n: 0, impulse_ns: [0, 0, 0], work_j: 0 };
      if (spec.max_force_n !== undefined) peaks.limit_n = spec.max_force_n;
      const state: RamState = {
        spec, bodyId, geomId: model.geom(ramGeomName(spec.id)).id,
        qposAdr: (model.jnt_qposadr as Int32Array)[jnt]!, dofAdr: (model.jnt_dofadr as Int32Array)[jnt]!,
        dir: unitVector(spec.direction), peaks, vStart: [0, 0, 0], vBefore: null, vAfter: null, tFirst: null, tLastEnd: null,
        contactSteps: 0, maxPenetration: 0, retiredAt: null, pending: [], pendingDt: 0,
      };
      this.byBody.set(bodyId, state);
      return state;
    });
  }

  get length(): number {
    return this.rams.length;
  }

  /** `peaks.effectors` de los arietes (los mismos objetos que se actualizan durante la simulación). */
  peaks(): Record<string, EffectorPeaks> {
    return Object.fromEntries(this.rams.map((r) => [r.spec.id, r.peaks]));
  }

  /**
   * Coloca y lanza cada ariete. Se llama una vez, al acabar el asentamiento (t = 0), con la cinemática al día. Posición inicial:
   * `sitio − d·r − d·v·start_s` (d = dirección unitaria, r = radio): la superficie del ariete llega al sitio en `start_s`, con
   * velocidad constante `v·d` desde el principio (sin gravedad, sin fuerzas: llega con exactamente `speed_mps`). El sitio se toma
   * en su posición de t = 0.
   * @param actorRotation `xmat` (9 valores, filas) del cuerpo principal del actor en t = 0, para `frame: "actor_initial"`.
   */
  place(actorRotation: ArrayLike<number>): void {
    const siteXpos = this.data.site_xpos as Float64Array;
    const qpos = this.data.qpos as Float64Array;
    const qvel = this.data.qvel as Float64Array;
    for (const r of this.rams) {
      const sid = this.model.site(r.spec.site).id;
      const site: Vec3 = [siteXpos[3 * sid]!, siteXpos[3 * sid + 1]!, siteXpos[3 * sid + 2]!];
      const d = r.spec.frame === "actor_initial" ? rotate(actorRotation, r.dir) : r.dir;
      r.dir = d;
      const lead = RAM_RADIUS_M + r.spec.speed_mps * r.spec.start_s;
      for (let k = 0; k < 3; k++) {
        qpos[r.qposAdr + k] = site[k]! - d[k]! * lead;
        qvel[r.dofAdr + k] = d[k]! * r.spec.speed_mps;
        qvel[r.dofAdr + 3 + k] = 0;
      }
      qpos[r.qposAdr + 3] = 1;
      qpos[r.qposAdr + 4] = 0;
      qpos[r.qposAdr + 5] = 0;
      qpos[r.qposAdr + 6] = 0;
    }
  }

  private linearVelocity(r: RamState): Vec3 {
    const qvel = this.data.qvel as Float64Array;
    return [qvel[r.dofAdr]!, qvel[r.dofAdr + 1]!, qvel[r.dofAdr + 2]!];
  }

  /**
   * Al principio de cada iteración (cinemática al día): cierra el trabajo del paso anterior con la velocidad actual de cada punto
   * de contacto (regla del trapecio, como los demás efectores) y retira el ariete si han pasado `RAM_RETIRE_AFTER_CONTACT_S`
   * desde su primer contacto: desactiva su colisión (`contype` y `conaffinity` a 0) y lo aparta a `RAM_RETREAT_SPEED_MPS` en
   * sentido contrario al de llegada (solo para que no se quede dentro de Uke en el vídeo; ya no interactúa).
   */
  beginStep(t: number): void {
    const xpos = this.data.xpos as Float64Array;
    const xmat = this.data.xmat as Float64Array;
    for (const r of this.rams) {
      if (r.pending.length > 0) {
        let work = 0;
        for (const c of r.pending) {
          const p = this.worldPoint(xpos, xmat, c.bodyId, c.local);
          const v = this.pointVelocityAt(c.bodyId, p);
          work += (c.force[0] * (c.velocity[0] + v[0]) + c.force[1] * (c.velocity[1] + v[1]) + c.force[2] * (c.velocity[2] + v[2])) / 2;
        }
        r.peaks.work_j += work * r.pendingDt;
        r.pending = [];
      }
      if (r.retiredAt === null && r.tFirst !== null && t >= r.tFirst + RAM_RETIRE_AFTER_CONTACT_S - 1e-9) {
        (this.model.geom_contype as Int32Array)[r.geomId] = 0;
        (this.model.geom_conaffinity as Int32Array)[r.geomId] = 0;
        const qvel = this.data.qvel as Float64Array;
        for (let k = 0; k < 3; k++) {
          qvel[r.dofAdr + k] = -r.dir[k]! * RAM_RETREAT_SPEED_MPS;
          qvel[r.dofAdr + 3 + k] = 0;
        }
        r.retiredAt = t;
      }
      r.vStart = this.linearVelocity(r);
    }
  }

  private worldPoint(xpos: Float64Array, xmat: Float64Array, bodyId: number, local: Vec3): Vec3 {
    const p = rotate(xmat.subarray(9 * bodyId, 9 * bodyId + 9), local);
    return [xpos[3 * bodyId]! + p[0], xpos[3 * bodyId + 1]! + p[1], xpos[3 * bodyId + 2]! + p[2]];
  }

  /**
   * Justo después de `mj_step`: los contactos y `efc_force` que quedan son los del estado al inicio del paso (los mismos con
   * los que se integró). Acumula, por ariete, la fuerza que ejerce sobre el resto del modelo (Uke), su impulso, su pico, la
   * penetración máxima y los instantes de primer y último contacto. `t` es el inicio del paso y `dt` su duración.
   */
  afterStep(t: number, dt: number): void {
    if (this.rams.every((r) => r.retiredAt !== null)) return;
    const bodyOfGeom = this.model.geom_bodyid as Int32Array;
    const efcForce = this.data.efc_force as Float64Array;
    const xpos = this.data.xpos as Float64Array;
    const xmat = this.data.xmat as Float64Array;
    const stepForce = new Map<RamState, { force: [number, number, number]; contacts: PendingContact[] }>();
    const contacts = this.data.contact;
    try {
      for (let c = 0; c < contacts.size(); c++) {
        const contact = contacts.get(c);
        if (!contact) throw new Error("Contacto MuJoCo inaccesible");
        try {
          const b1 = bodyOfGeom[contact.geom1]!;
          const b2 = bodyOfGeom[contact.geom2]!;
          const ram = this.byBody.get(b1) ?? this.byBody.get(b2);
          if (!ram) continue;
          const ramIsFirst = ram.bodyId === b1;
          const other = ramIsFirst ? b2 : b1;
          if (!hasCompleteSolvedContactRows(contact.efc_address, contact.dim, this.pyramidal, this.data.nefc, efcForce.length)) continue;
          const solved = decodeSolvedContactForce({
            solverRows: efcForce, efcAddress: contact.efc_address, dim: contact.dim, pyramidal: this.pyramidal,
            friction: contact.friction as Float64Array, adhesionN: contact.adhesion,
          });
          if (!(solved.normalN > MIN_CONTACT_NORMAL_N)) continue;
          const frame = contact.frame as Float64Array;
          const [t1, t2] = solved.tangentN;
          // Fuerza de la geometría 1 sobre la 2 (convención de las grabaciones); la del ariete sobre el otro cuerpo es ± ella.
          const sign = ramIsFirst ? 1 : -1;
          const force: Vec3 = [
            sign * (solved.normalN * frame[0]! + t1 * frame[3]! + t2 * frame[6]!),
            sign * (solved.normalN * frame[1]! + t1 * frame[4]! + t2 * frame[7]!),
            sign * (solved.normalN * frame[2]! + t1 * frame[5]! + t2 * frame[8]!),
          ];
          const pos = contact.pos as Float64Array;
          const point: Vec3 = [pos[0]!, pos[1]!, pos[2]!];
          ram.maxPenetration = Math.max(ram.maxPenetration, -contact.dist);
          // Punto en el sistema local del cuerpo (xmat^T · (p − xpos)).
          const rel: Vec3 = [point[0] - xpos[3 * other]!, point[1] - xpos[3 * other + 1]!, point[2] - xpos[3 * other + 2]!];
          const m = xmat.subarray(9 * other, 9 * other + 9);
          const local: Vec3 = [
            m[0]! * rel[0] + m[3]! * rel[1] + m[6]! * rel[2],
            m[1]! * rel[0] + m[4]! * rel[1] + m[7]! * rel[2],
            m[2]! * rel[0] + m[5]! * rel[1] + m[8]! * rel[2],
          ];
          const entry = stepForce.get(ram) ?? { force: [0, 0, 0] as [number, number, number], contacts: [] };
          entry.force[0] += force[0]; entry.force[1] += force[1]; entry.force[2] += force[2];
          entry.contacts.push({ bodyId: other, local, force, velocity: this.pointVelocityAt(other, point) });
          stepForce.set(ram, entry);
        } finally {
          contact.delete();
        }
      }
    } finally {
      contacts.delete();
    }
    for (const [r, { force, contacts: list }] of stepForce) {
      if (r.tFirst === null) {
        r.tFirst = t;
        r.vBefore = r.vStart;
      }
      r.tLastEnd = t + dt;
      r.vAfter = this.linearVelocity(r);
      r.contactSteps++;
      const mag = Math.hypot(force[0], force[1], force[2]);
      if (mag > r.peaks.max_force_n) r.peaks.max_force_n = mag;
      r.peaks.impulse_ns = [
        r.peaks.impulse_ns[0] + force[0] * dt, r.peaks.impulse_ns[1] + force[1] * dt, r.peaks.impulse_ns[2] + force[2] * dt,
      ];
      r.pending = list;
      r.pendingDt = dt;
    }
  }

  /**
   * Fuerza del ariete sobre Uke para la muestra de grabación, a partir de los contactos ya grabados (misma convención: fuerza de
   * `body_a` sobre `body_b`). Punto: media de los puntos de contacto ponderada por su fuerza, o el centro del ariete sin contacto.
   */
  effectorSamples(contacts: readonly ContactSample[]): EffectorSample[] {
    const xpos = this.data.xpos as Float64Array;
    return this.rams.map((r) => {
      const name = this.bodies[r.bodyId]!;
      const force: [number, number, number] = [0, 0, 0];
      const point: [number, number, number] = [0, 0, 0];
      let weight = 0;
      for (const c of contacts) {
        const sign = c.body_a === name ? 1 : c.body_b === name ? -1 : 0;
        if (sign === 0) continue;
        const w = Math.hypot(c.force_n[0], c.force_n[1], c.force_n[2]);
        for (let k = 0; k < 3; k++) {
          force[k] += sign * c.force_n[k]!;
          point[k] += w * c.point_m[k]!;
        }
        weight += w;
      }
      const p: [number, number, number] = weight > 0
        ? [point[0] / weight, point[1] / weight, point[2] / weight]
        : [xpos[3 * r.bodyId]!, xpos[3 * r.bodyId + 1]!, xpos[3 * r.bodyId + 2]!];
      return { id: r.spec.id, point_m: p, force_n: force };
    });
  }

  /** Resumen por ariete (`summary.rams`). */
  summary(): Record<string, RamSummary> {
    return Object.fromEntries(this.rams.map((r) => {
      const impulse = r.peaks.impulse_ns;
      const s: RamSummary = {
        mass_kg: r.spec.mass_kg,
        speed_mps: r.spec.speed_mps,
        t_first_contact_s: r.tFirst,
        contact_duration_s: r.tFirst !== null && r.tLastEnd !== null ? r.tLastEnd - r.tFirst : null,
        contact_steps: r.contactSteps,
        peak_force_n: r.peaks.max_force_n,
        transmitted_impulse_ns: [impulse[0], impulse[1], impulse[2]],
        v_before_mps: r.vBefore,
        v_after_mps: r.vAfter,
        max_penetration_m: r.maxPenetration,
        retired_t_s: r.retiredAt,
      };
      return [r.spec.id, s];
    }));
  }
}
