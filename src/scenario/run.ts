/**
 * Ejecutor de escenarios `hpl.scenario.v1` (solo Node): construye el MJCF, asienta 0,5 s sin efectores,
 * aplica los efectores antes de cada `mj_step` y muestrea a `record_hz`. Usa el módulo MuJoCo
 * directamente (sin `MujocoSimulation`). La matemática de la fuerza en un punto es la de
 * `src/physics/mujoco.ts`: fuerza en `xfrc_applied` y par `r × F` respecto al centro de masas del cuerpo.
 *
 * `Recording.samples` y `peaks` conservan la precisión completa; el redondeo a 6 cifras
 * se aplica al escribir (`src/recording/write.ts`).
 */
import type { StandingModel, StandingState } from "../control/standingBaseline";
import { detectArtifacts } from "../metrics/artifacts";
import { capturePoint, signedMargin, supportPolygon } from "../metrics/balance";
import {
  FALL_MIN_TILT_DEG, FOOT_BODIES, classifyHumanoidTrajectory, classifyTilt, fallDirectionDeg, tiltDeg,
  type BalanceFrame, type GroundTouch,
} from "../metrics/outcome";
import { createUkeController, type UkeController, type UkeControllerParams } from "../uke/attitudes";
import { attitudeJointDamping, setJointDamping, type DampingModel } from "../uke/damping";
import { applyStance } from "../uke/stances";
import { limitPullPower, pullDemand, resolvePullParams, type PullParams } from "../effectors/pull";
import { ToriBudget } from "../effectors/toriBudget";
import { createHandState, resolveHandParams, stepHand, type HandParams, type HandState } from "../effectors/hand";
import {
  applyObstacleCommand, createObstacleState, readObstacleContactForce, readObstacleKinematics, resolveObstacleHandles,
  settleObstacleWork, stepObstacle, type ObstacleContactData, type ObstacleModelLike, type ObstacleState,
} from "../effectors/obstacle";
import {
  RECORDING_SCHEMA_ID, type ContactSample, type EffectorPeaks, type EffectorSample, type HandPeaks, type ObstaclePeaks,
  type Recording, type RecordingSample, type RecordingSummary,
} from "../recording/schema";
import { decodeSolvedContactForce, hasCompleteSolvedContactRows } from "../physics/contactForces";
import { ArtifactProbe } from "./artifactProbe";
import { buildScenarioModel } from "./buildModel";
import { sha256Hex } from "./sha256";
import { compareExpect } from "./expect";
import { getMujoco, type MujocoModule } from "./mujocoModule";
import { ACTOR_BODIES, type EffectorSpec, type PrescribedForceEffector, type Outcome, type Scenario, type Vec3 } from "./schema";
import { PHYSICS_HZ, PHYSICS_TIMESTEP_S } from "./worldOptions";

/** Asentamiento previo sin efectores, en s. El tiempo del escenario empieza después. */
export const SETTLE_S = 0.5;

/** Contactos con fuerza normal por debajo de este valor (N) no se graban. */
const MIN_CONTACT_NORMAL_N = 1e-6;

export interface RunOptions {
  /** Módulo MuJoCo ya cargado (el navegador pasa el del visor). Por defecto, el de `mujocoModule.ts`. */
  mujoco?: MujocoModule;
  /** Versión del motor para `meta.engine_version`. Por defecto `unknown`: el ejecutor no lee archivos (también corre en el navegador); en Node la pasan `cli.ts` y `galleryBuild.ts` con `readEngineVersion()`. */
  engineVersion?: string;
  /**
   * Observador opcional, llamado tras grabar cada muestra con el modelo y los datos de MuJoCo (solo lectura).
   * Lo usa la prueba de reproducción para comparar `xpos` de la simulación real con `mj_forward`.
   */
  onSample?: (model: unknown, data: unknown, sampleIndex: number) => void;
}

/** Ganancia (0 a 1) de la forma de onda en el instante `t` (s del escenario) de un efector. */
export function waveformGain(effector: Pick<PrescribedForceEffector, "start_s" | "duration_s" | "waveform">, t: number): number {
  const u = t - effector.start_s;
  if (u < 0 || u >= effector.duration_s) return 0;
  return effector.waveform === "half_sine" ? Math.sin((Math.PI * u) / effector.duration_s) : 1;
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

/** `R · v` con `R` en filas (9 valores desde `offset`, como `xmat` de MuJoCo). */
function rotate(m: ArrayLike<number>, offset: number, v: Vec3): Vec3 {
  return [
    m[offset]! * v[0] + m[offset + 1]! * v[1] + m[offset + 2]! * v[2],
    m[offset + 3]! * v[0] + m[offset + 4]! * v[1] + m[offset + 5]! * v[2],
    m[offset + 6]! * v[0] + m[offset + 7]! * v[1] + m[offset + 8]! * v[2],
  ];
}

function triple(a: ArrayLike<number>, i: number): Vec3 {
  return [a[i]!, a[i + 1]!, a[i + 2]!];
}

/** Hash del escenario validado (orden de claves fijado por el validador). */
export function scenarioSha256(scenario: Scenario): string {
  return sha256Hex(JSON.stringify(scenario));
}

interface PendingWork {
  force: Vec3;
  velocity: Vec3;
}

export async function runScenario(scenario: Scenario, options: RunOptions = {}): Promise<Recording> {
  const mj = options.mujoco ?? await getMujoco();
  const built = buildScenarioModel(scenario);
  const model = mj.MjModel.from_xml_string(built.mjcf);
  const data = new mj.MjData(model);
  try {
    if (Math.abs(model.opt.timestep - PHYSICS_TIMESTEP_S) > 1e-12) {
      throw new Error(`El paso del modelo (${model.opt.timestep}) no es ${PHYSICS_TIMESTEP_S}`);
    }
    if (model.nbody !== built.bodies.length || model.njnt !== built.joints.length) {
      throw new Error("El modelo compilado no coincide con los cuerpos o articulaciones declarados");
    }
    const dt = PHYSICS_TIMESTEP_S;
    const stepsPerSample = PHYSICS_HZ / scenario.record_hz;
    const settleSteps = Math.round(SETTLE_S / dt);
    const totalSteps = Math.round(scenario.duration_s / dt);
    const actorBodyId = built.bodies.indexOf(built.actorBody);
    const humanoid = built.humanoid;
    // Cuerpo cuya inclinación se mide: el tronco (`chest`) del humanoide o el propio actor.
    const tiltBodyId = humanoid ? built.bodies.indexOf("chest") : actorBodyId;

    // Amortiguación articular de la actitud, escrita en `model.dof_damping` en tiempo de ejecución (`src/uke/damping.ts`).
    // Un Uke `relaxed` asienta con la actitud `tense` (si no, se desplomaría durante el asentamiento y el escenario
    // empezaría ya cayendo) y pasa a `relaxed` (controlador y amortiguación) en t = 0.
    const settleAttitude = humanoid && humanoid.attitude === "relaxed" ? "tense" : humanoid?.attitude;
    if (humanoid) {
      setJointDamping(model as unknown as DampingModel, attitudeJointDamping(
        settleAttitude!, humanoid.mass_kg, humanoid.height_m, humanoid.strength,
      ));
    }

    const effectors = scenario.effectors.map((spec) => {
      let local: Vec3 = spec.local_point_m;
      if (spec.site !== undefined) {
        // Punto de un sitio con nombre: su posición en el cuerpo más el desplazamiento opcional (ejes del cuerpo).
        const siteId = model.site(spec.site).id;
        const sp = model.site_pos as Float64Array;
        local = [sp[3 * siteId]! + local[0], sp[3 * siteId + 1]! + local[1], sp[3 * siteId + 2]! + local[2]];
      }
      const entry: {
        spec: EffectorSpec; bodyId: number; local: Vec3;
        hand?: { params: HandParams; state: HandState; peaks: HandPeaks };
        pull?: PullParams;
      } = { spec, bodyId: built.bodies.indexOf(spec.body), local };
      if (spec.kind === "pull") entry.pull = resolvePullParams(spec);
      return entry;
    });
    // Obstáculos: cuerpos al final del modelo, con su orden en `qfrc_applied`. Durante el asentamiento se sostienen en `start_m`.
    const obstacles = built.obstacles.map((params) => ({
      params,
      handles: resolveObstacleHandles(model as unknown as ObstacleModelLike, params),
      state: createObstacleState() as ObstacleState,
      contact: [0, 0, 0] as Vec3,
      command: { force_n: [0, 0, 0] as Vec3, torque_nm: 0 },
    }));
    const actorBodyIds = new Set(ACTOR_BODIES[scenario.actors[0]!.kind].map((b) => built.bodies.indexOf(b)));
    const applyObstacleCommands = (): void => {
      for (const o of obstacles) applyObstacleCommand(o.handles, data as unknown as { qfrc_applied: Float64Array }, o.command);
    };
    const peaks: Record<string, EffectorPeaks> = {};
    for (const e of effectors) {
      peaks[e.spec.id] = { max_force_n: 0, impulse_ns: [0, 0, 0], work_j: 0 };
      // Tope declarado (si el efector lo trae): lo comprueba el detector de artefactos.
      const cap = e.spec.kind === "hand"
        ? resolveHandParams(e.spec).max_force_n
        : e.pull ? e.pull.max_force_n : (e.spec as { max_force_n?: number }).max_force_n;
      if (e.pull) peaks[e.spec.id]!.pull = { max_power_w: 0, time_power_limited_s: 0 };
      if (cap !== undefined) peaks[e.spec.id]!.limit_n = cap;
    }
    const pending: Array<PendingWork | null> = effectors.map(() => null);

    const xfrc = data.xfrc_applied as Float64Array;
    const bodyOfGeom = model.geom_bodyid as Int32Array;
    const rootOfBody = model.body_rootid as Int32Array;
    const pyramidal = Boolean(mj.mj_isPyramidal(model));

    const updateKinematics = (): void => {
      mj.mj_kinematics(model, data);
      mj.mj_comPos(model, data);
      mj.mj_comVel(model, data);
    };

    // Humanoide: postura inicial cinemática y controlador (se llama antes de cada `mj_step`, también al asentar).
    let uke: UkeController | undefined;
    let ukeParams: UkeControllerParams | undefined;
    let rootForceMax = 0;
    const runUke = (t: number): void => {
      mj.mj_subtreeVel(model, data);
      uke!.step(model as unknown as StandingModel, data as unknown as StandingState, t);
      const applied = data.qfrc_applied as Float64Array;
      for (let k = 0; k < 6; k++) rootForceMax = Math.max(rootForceMax, Math.abs(applied[k]!));
      if (rootForceMax !== 0) throw new Error("El controlador de Uke escribió fuerza generalizada en la raíz");
    };
    if (humanoid) {
      const placement = applyStance(mj, model, data, humanoid.stance);
      mj.mj_forward(model, data); // también deja `qfrc_bias` (gravedad) listo para el controlador
      ukeParams = {
        attitude: settleAttitude!, stance: humanoid.stance, mass_kg: humanoid.mass_kg,
        height_m: humanoid.height_m, strength: humanoid.strength, targets: placement.targets,
      };
      uke = createUkeController(ukeParams, model as unknown as StandingModel, data as unknown as StandingState);
    }

    // 1. Asentamiento sin efectores.
    for (let i = 0; i < settleSteps; i++) {
      xfrc.fill(0);
      if (uke) {
        updateKinematics();
        runUke(-SETTLE_S + i * dt);
      }
      for (const o of obstacles) {
        // Sin estado: solo sostiene el obstáculo en su posición inicial (objetivo de t = 0).
        o.command = stepObstacle(o.params, createObstacleState(), {
          t: 0, dt, ...readObstacleKinematics(o.params, o.handles, data as unknown as { qpos: Float64Array; qvel: Float64Array }),
        });
      }
      applyObstacleCommands();
      mj.mj_step(model, data);
    }
    if (humanoid && ukeParams && humanoid.attitude !== settleAttitude) {
      uke = createUkeController({ ...ukeParams, attitude: humanoid.attitude },
        model as unknown as StandingModel, data as unknown as StandingState);
      setJointDamping(model as unknown as DampingModel, attitudeJointDamping(
        humanoid.attitude, humanoid.mass_kg, humanoid.height_m, humanoid.strength,
      ));
    }

    // 2. Escenario. `data.time` no se usa: el tiempo del escenario es `i * dt`.

    const pointState = (bodyId: number, local: Vec3): { point: Vec3; velocity: Vec3 } => {
      const xpos = data.xpos as Float64Array;
      const xmat = data.xmat as Float64Array;
      const point = add(triple(xpos, 3 * bodyId), rotate(xmat, 9 * bodyId, local));
      const cvel = data.cvel as Float64Array; // [angular(3), lineal(3)] respecto a subtree_com de la raíz del árbol
      const ref = triple(data.subtree_com as Float64Array, 3 * rootOfBody[bodyId]!);
      const velocity = add(triple(cvel, 6 * bodyId + 3), cross(triple(cvel, 6 * bodyId), sub(point, ref)));
      return { point, velocity };
    };

    const readContacts = (): ContactSample[] => {
      const result: ContactSample[] = [];
      const efcForce = data.efc_force as Float64Array;
      const contacts = data.contact;
      try {
        for (let c = 0; c < contacts.size(); c++) {
          const contact = contacts.get(c);
          if (!contact) throw new Error("Contacto MuJoCo inaccesible");
          try {
            if (!hasCompleteSolvedContactRows(contact.efc_address, contact.dim, pyramidal, data.nefc, efcForce.length)) {
              continue;
            }
            const solved = decodeSolvedContactForce({
              solverRows: efcForce,
              efcAddress: contact.efc_address,
              dim: contact.dim,
              pyramidal,
              friction: contact.friction as Float64Array,
              adhesionN: contact.adhesion,
            });
            if (!(solved.normalN > MIN_CONTACT_NORMAL_N)) continue;
            const frame = contact.frame as Float64Array;
            const [t1, t2] = solved.tangentN;
            const force: Vec3 = [
              solved.normalN * frame[0]! + t1 * frame[3]! + t2 * frame[6]!,
              solved.normalN * frame[1]! + t1 * frame[4]! + t2 * frame[7]!,
              solved.normalN * frame[2]! + t1 * frame[5]! + t2 * frame[8]!,
            ];
            const pos = contact.pos as Float64Array;
            result.push({
              body_a: built.bodies[bodyOfGeom[contact.geom1]!]!,
              body_b: built.bodies[bodyOfGeom[contact.geom2]!]!,
              point_m: [pos[0]!, pos[1]!, pos[2]!],
              force_n: force,
            });
          } finally {
            contact.delete();
          }
        }
      } finally {
        contacts.delete();
      }
      return result;
    };

    // Estado inicial del escenario: ejes del actor en t = 0 (para `actor_initial`) y posición inicial.
    updateKinematics();
    // Humanoide: referencias del veredicto tras el asentamiento.
    let initialPelvisZ = 0;
    let initialComZ = 0;
    let settleComSpeed = 0;
    let minPelvisZ = Infinity;
    if (humanoid) {
      mj.mj_subtreeVel(model, data);
      initialPelvisZ = (data.xpos as Float64Array)[3 * actorBodyId + 2]!;
      initialComZ = (data.subtree_com as Float64Array)[3 * actorBodyId + 2]!;
      const lv = data.subtree_linvel as Float64Array;
      settleComSpeed = Math.hypot(lv[3 * actorBodyId]!, lv[3 * actorBodyId + 1]!, lv[3 * actorBodyId + 2]!);
    }
    const initialRotation = Array.from((data.xmat as Float64Array).subarray(9 * actorBodyId, 9 * actorBodyId + 9));
    const startXY: [number, number] = [(data.xpos as Float64Array)[3 * actorBodyId]!, (data.xpos as Float64Array)[3 * actorBodyId + 1]!];

    // Manos: el hombro, los desplazamientos y el eje vienen en `actor_initial` (origen y ejes del cuerpo principal en t = 0)
    // y se pasan al mundo aquí; `stepHand` trabaja todo en el mundo.
    const initialOrigin = triple(data.xpos as Float64Array, 3 * actorBodyId);
    for (const e of effectors) {
      if (e.spec.kind !== "hand") continue;
      const spec = e.spec;
      const params = resolveHandParams({
        ...spec,
        shoulder_m: add(initialOrigin, rotate(initialRotation, 0, spec.shoulder_m)),
        phases: spec.phases.map((p) => ({ ...p, offset_m: rotate(initialRotation, 0, p.offset_m) })),
        ...(spec.axis !== undefined ? { axis: rotate(initialRotation, 0, spec.axis) } : {}),
      });
      const handPeaks: HandPeaks = {
        max_power_w: 0, time_at_cap_s: 0, time_power_limited_s: 0, time_reach_limited_s: 0, events: [],
      };
      e.hand = { params, state: createHandState(), peaks: handPeaks };
      peaks[spec.id]!.hand = handPeaks;
    }

    // Humanoide: sitios de las suelas, partes que cuentan como «toca el suelo» (todas menos los pies) y frente inicial (+X del
    // cuerpo `pelvis`). El equilibrio se muestrea a `record_hz`; los primeros contactos con el suelo, a 500 Hz.
    const soleIds = humanoid ? { left: model.site("left_sole").id, right: model.site("right_sole").id } : undefined;
    const touchBodies = new Set<string>(humanoid ? ACTOR_BODIES.humanoid.filter((b) => !FOOT_BODIES.includes(b)) : []);
    const heading = Math.atan2(initialRotation[3]!, initialRotation[0]!);
    const frames: BalanceFrame[] = [];
    const touches: GroundTouch[] = [];
    const touched = new Set<string>();

    /** Velocidad (mundo) del punto `p` del cuerpo `bodyId` con el `cvel` actual. */
    const pointVelocityAt = (bodyId: number, p: Vec3): Vec3 => {
      const cvel = data.cvel as Float64Array;
      const ref = triple(data.subtree_com as Float64Array, 3 * rootOfBody[bodyId]!);
      return add(triple(cvel, 6 * bodyId + 3), cross(triple(cvel, 6 * bodyId), sub(p, ref)));
    };

    /**
     * Anota el primer contacto de cada parte del actor (no pies) con el suelo (`world`). Se llama justo después de `mj_step`: los
     * contactos, `xpos` y `cvel` que quedan son los del inicio de ese paso (antes de que su fuerza actúe), de modo que la
     * velocidad del punto de contacto es la de justo antes del impacto.
     */
    const scanTouches = (t: number): void => {
      const contacts = data.contact;
      try {
        for (let c = 0; c < contacts.size(); c++) {
          const contact = contacts.get(c);
          if (!contact) throw new Error("Contacto MuJoCo inaccesible");
          try {
            const b1 = bodyOfGeom[contact.geom1]!;
            const b2 = bodyOfGeom[contact.geom2]!;
            const other = b1 === 0 ? b2 : b2 === 0 ? b1 : -1;
            if (other <= 0) continue;
            const name = built.bodies[other]!;
            if (!touchBodies.has(name) || touched.has(name)) continue;
            touched.add(name);
            const pos = contact.pos as Float64Array;
            touches.push({ body: name, t_s: t, velocity_mps: pointVelocityAt(other, [pos[0]!, pos[1]!, pos[2]!]) });
          } finally {
            contact.delete();
          }
        }
      } finally {
        contacts.delete();
      }
    };

    // Detector de artefactos (#75 PR 5): medidas por paso físico a partir de t = 0 (no del asentamiento).
    const probe = new ArtifactProbe(mj, model, data, {
      bodies: built.bodies, joints: built.joints, actorBodies: ACTOR_BODIES[scenario.actors[0]!.kind], humanoid: Boolean(humanoid), dt,
    });
    const samples: RecordingSample[] = [];
    const toriBudget = scenario.tori
      ? new ToriBudget({ ...scenario.tori, friction: scenario.tori.friction ?? scenario.ground.friction })
      : undefined;
    let maxTilt = 0;
    let currentClass: Outcome | undefined;
    let classSinceS = 0;
    let finalTilt = 0;
    // Dirección de caída: la del eje en el primer paso en que la inclinación supera el umbral de caída; si nunca
    // lo supera, la del eje en la inclinación máxima. (La dirección del eje final no sirve: un cilindro tumbado
    // tiene el mismo aspecto con la tapa hacia +X o hacia -X.)
    let directionAtFall: number | null | undefined;
    let directionAtMax: number | null = null;
    let finalComHeight = 0;
    let finalXY: [number, number] = startXY;

    for (let i = 0; i <= totalSteps; i++) {
      const t = i * dt;
      if (i > 0) updateKinematics();
      probe.state(t);

      // Veredicto a 500 Hz sobre el estado actual.
      const xmat = data.xmat as Float64Array;
      const tilt = tiltDeg(xmat, 9 * tiltBodyId);
      const pelvisZ = (data.xpos as Float64Array)[3 * actorBodyId + 2]!;
      if (humanoid) minPelvisZ = Math.min(minPelvisZ, pelvisZ);
      if (tilt > maxTilt) {
        maxTilt = tilt;
        directionAtMax = fallDirectionDeg(xmat, 9 * tiltBodyId);
      }
      // El veredicto del humanoide sale de su trayectoria completa al final (`classifyHumanoidTrajectory`).
      if (!humanoid) {
        const cls = classifyTilt(tilt);
        if (directionAtFall === undefined && tilt > FALL_MIN_TILT_DEG) directionAtFall = fallDirectionDeg(xmat, 9 * tiltBodyId);
        if (cls !== currentClass) {
          currentClass = cls;
          classSinceS = t;
        }
      }

      // Efectores: fuerza durante el paso [t, t + dt], evaluada en el punto medio.
      xfrc.fill(0);
      const sampleEffectors: EffectorSample[] = [];
      let handsForce: Vec3 = [0, 0, 0];
      let handsPower = 0;
      effectors.forEach((e, k) => {
        const { point, velocity } = pointState(e.bodyId, e.local);
        const previous = pending[k];
        if (previous) {
          // Trabajo del paso anterior con la velocidad media (trapecio) del punto.
          const vMean: Vec3 = [
            (previous.velocity[0] + velocity[0]) / 2,
            (previous.velocity[1] + velocity[1]) / 2,
            (previous.velocity[2] + velocity[2]) / 2,
          ];
          peaks[e.spec.id]!.work_j += dot(previous.force, vMean) * dt;
        }
        let force: Vec3 = [0, 0, 0];
        let handSample: EffectorSample["hand"];
        if (e.spec.kind === "hand") {
          // Mano virtual: fuerza del muelle con sus topes, evaluada con el estado al comienzo del paso.
          const hand = e.hand!;
          const r = stepHand(hand.params, hand.state, { t, dt, grip_m: point, grip_velocity_mps: velocity });
          handSample = {
            held: r.sample.held, target_m: r.sample.target_m, force_limited: r.sample.force_limited,
            power_limited: r.sample.power_limited, reach_limited: r.sample.reach_limited,
          };
          if (i < totalSteps) {
            hand.state = r.state;
            force = r.force_n;
            const hp = hand.peaks;
            hp.max_power_w = Math.max(hp.max_power_w, r.sample.power_w);
            if (r.sample.force_limited) hp.time_at_cap_s += dt;
            if (r.sample.power_limited) hp.time_power_limited_s += dt;
            if (r.sample.reach_limited) hp.time_reach_limited_s += dt;
            for (const ev of r.events) hp.events.push({ t_s: ev.t, kind: ev.kind });
          }
        } else if (e.spec.kind === "pull") {
          // Tirón guiado: fuerza de la curva en el punto medio del paso, llevada al mundo y recortada por potencia.
          const demand = pullDemand(e.pull!.keys, t + dt / 2);
          const world: Vec3 = e.spec.frame === "world" ? demand
            : e.spec.frame === "actor_initial" ? rotate(initialRotation, 0, demand) : rotate(xmat, 9 * e.bodyId, demand);
          const limited = limitPullPower(e.pull!, world, velocity);
          force = limited.force_n;
          if (i < totalSteps) {
            const pp = peaks[e.spec.id]!.pull!;
            pp.max_power_w = Math.max(pp.max_power_w, limited.power_w);
            if (limited.power_limited) pp.time_power_limited_s += dt;
          }
        } else {
          const gain = waveformGain(e.spec, t + dt / 2);
          if (gain !== 0) {
            const f = e.spec.force_n;
            const scaled: Vec3 = [f[0] * gain, f[1] * gain, f[2] * gain];
            if (e.spec.frame === "world") force = scaled;
            else if (e.spec.frame === "actor_initial") force = rotate(initialRotation, 0, scaled);
            else force = rotate(xmat, 9 * e.bodyId, scaled);
          }
        }
        if (force[0] !== 0 || force[1] !== 0 || force[2] !== 0) {
          const com = triple(data.xipos as Float64Array, 3 * e.bodyId);
          const torque = cross(sub(point, com), force);
          const o = 6 * e.bodyId;
          xfrc[o] = xfrc[o]! + force[0];
          xfrc[o + 1] = xfrc[o + 1]! + force[1];
          xfrc[o + 2] = xfrc[o + 2]! + force[2];
          xfrc[o + 3] = xfrc[o + 3]! + torque[0];
          xfrc[o + 4] = xfrc[o + 4]! + torque[1];
          xfrc[o + 5] = xfrc[o + 5]! + torque[2];
        }
        if (i < totalSteps) {
          // Solo los pasos que se ejecutan de verdad (el muestreo final no da ningún paso).
          const peak = peaks[e.spec.id]!;
          let magnitude = Math.hypot(force[0], force[1], force[2]);
          // El recorte de la mano a su tope puede dejar el módulo un ulp por encima: eso no es superar el tope.
          if (e.hand && magnitude > e.hand.params.max_force_n && magnitude <= e.hand.params.max_force_n * (1 + 1e-9)) {
            magnitude = e.hand.params.max_force_n;
          }
          if (magnitude > peak.max_force_n) peak.max_force_n = magnitude;
          peak.impulse_ns = [
            peak.impulse_ns[0] + force[0] * dt,
            peak.impulse_ns[1] + force[1] * dt,
            peak.impulse_ns[2] + force[2] * dt,
          ];
        }
        pending[k] = i < totalSteps ? { force, velocity } : null;
        handsForce = add(handsForce, force);
        handsPower += dot(force, velocity);
        const effectorSample: EffectorSample = { id: e.spec.id, point_m: point, force_n: force };
        if (handSample) {
          effectorSample.kind = "hand";
          effectorSample.hand = handSample;
        }
        sampleEffectors.push(effectorSample);
      });

      // Obstáculos: orden del actuador para el paso [t, t + dt]. Se escribe ya (para el `mj_forward` de la muestra) y otra
      // vez tras el controlador de Uke, que pone `qfrc_applied` a cero.
      for (const o of obstacles) {
        const r = stepObstacle(o.params, o.state, {
          t, dt, ...readObstacleKinematics(o.params, o.handles, data as unknown as { qpos: Float64Array; qvel: Float64Array }),
          contact_force_on_uke_n: o.contact,
        });
        o.command = r;
        if (i < totalSteps) o.state = r.state;
      }
      applyObstacleCommands();
      if (toriBudget && i < totalSteps) {
        // El contacto de los obstáculos es el del paso anterior (un paso de retraso).
        let onUke: Vec3 = [0, 0, 0];
        for (const o of obstacles) onUke = add(onUke, o.contact);
        toriBudget.step({ t, dt, hands_force_n: handsForce, hands_power_w: handsPower, obstacle_force_on_uke_n: onUke });
      }

      if (i % stepsPerSample === 0) {
        mj.mj_forward(model, data); // contactos y fuerzas coherentes con el wrench ya aplicado
        probe.sample(t, Object.values(peaks).reduce((sum, p) => sum + p.work_j, 0));
        const sample: RecordingSample = {
          t,
          qpos: Array.from(data.qpos as Float64Array),
          qvel: Array.from(data.qvel as Float64Array),
          effectors: sampleEffectors,
          contacts: readContacts(),
        };
        if (humanoid && soleIds) {
          // Equilibrio: centro de masas y su velocidad (`subtree_com` / `subtree_linvel` de la pelvis, raíz del actor).
          mj.mj_subtreeVel(model, data);
          const com = triple(data.subtree_com as Float64Array, 3 * actorBodyId);
          const velocity = triple(data.subtree_linvel as Float64Array, 3 * actorBodyId);
          const xcom = capturePoint(com, velocity);
          const polygon = supportPolygon(sample.contacts, (b) => FOOT_BODIES.includes(b));
          const sites = data.site_xpos as Float64Array;
          sample.com = com;
          sample.xcom = [xcom[0], xcom[1]];
          sample.support_polygon = polygon.map((p): [number, number] => [p[0], p[1]]);
          sample.capture_margin_m = signedMargin(xcom, polygon);
          frames.push({
            t, com, com_velocity: velocity, xcom, support_polygon: polygon,
            pelvis_z: (data.xpos as Float64Array)[3 * actorBodyId + 2]!,
            feet: {
              left: [sites[3 * soleIds.left]!, sites[3 * soleIds.left + 1]!],
              right: [sites[3 * soleIds.right]!, sites[3 * soleIds.right + 1]!],
            },
            trunk_dir_deg: fallDirectionDeg(data.xmat as Float64Array, 9 * tiltBodyId),
          });
        }
        samples.push(sample);
        options.onSample?.(model, data, samples.length - 1);
      }

      if (i === totalSteps) {
        finalTilt = tilt;
        finalComHeight = humanoid
          ? (data.subtree_com as Float64Array)[3 * actorBodyId + 2]!
          : (data.xipos as Float64Array)[3 * actorBodyId + 2]!;
        finalXY = [(data.xpos as Float64Array)[3 * actorBodyId]!, (data.xpos as Float64Array)[3 * actorBodyId + 1]!];
        break;
      }
      if (uke) runUke(t);
      applyObstacleCommands();
      probe.beforeStep();
      mj.mj_step(model, data);
      probe.afterStep(t);
      for (const o of obstacles) {
        // Contacto resuelto del paso que acaba de ejecutarse, solo contra el actor (no contra props ni otros obstáculos).
        o.contact = readObstacleContactForce(
          model as unknown as ObstacleModelLike, o.handles, data as unknown as ObstacleContactData,
          { pyramidal, isOtherBody: (b) => actorBodyIds.has(b) },
        ).force_on_other_n;
      }
      if (humanoid) scanTouches(t);
    }

    const verdict = humanoid
      ? classifyHumanoidTrajectory({ frames, touches, initial_pelvis_z: initialPelvisZ, heading_rad: heading })
      : undefined;
    const outcome: Outcome = verdict ? verdict.outcome : currentClass!;
    const summary: RecordingSummary = {
      outcome,
      direction_deg: verdict
        ? (verdict.fall ? verdict.fall.direction_world_deg : directionAtMax)
        : directionAtFall !== undefined ? directionAtFall : directionAtMax,
      t_outcome_s: verdict ? verdict.t_outcome_s : classSinceS,
      final_tilt_deg: finalTilt,
      max_tilt_deg: maxTilt,
      final_com_height_m: finalComHeight,
      slide_m: Math.hypot(finalXY[0] - startXY[0], finalXY[1] - startXY[1]),
      expect: "none",
      expect_diffs: [],
    };
    if (humanoid && uke && verdict) {
      summary.uke = {
        max_trunk_tilt_deg: maxTilt,
        initial_pelvis_height_m: initialPelvisZ,
        min_pelvis_height_m: minPelvisZ,
        settle_com_speed_mps: settleComSpeed,
        initial_com_height_m: initialComZ,
        max_root_generalized_force: rootForceMax,
        controller: uke.status(),
        balance: verdict.balance,
        fall: verdict.fall,
        step: verdict.step,
        unbalance: verdict.unbalance,
      };
    }
    if (toriBudget) summary.tori_budget = toriBudget.summary();
    if (scenario.expect) {
      const result = compareExpect(summary, scenario.expect);
      summary.expect = result === "pass" ? "pass" : "fail";
      summary.expect_diffs = result === "pass" ? [] : result;
    }

    const obstaclePeaks: Record<string, ObstaclePeaks> = {};
    for (const o of obstacles) {
      const k = readObstacleKinematics(o.params, o.handles, data as unknown as { qpos: Float64Array; qvel: Float64Array });
      const totals = settleObstacleWork(o.state, k.velocity_mps).totals;
      obstaclePeaks[o.params.name] = {
        max_force_n: totals.max_force_n,
        max_horizontal_force_n: totals.max_horizontal_force_n,
        max_vertical_force_n: totals.max_vertical_force_n,
        limit_horizontal_n: o.params.max_horizontal_force_n,
        limit_vertical_n: o.params.max_vertical_force_n,
        max_contact_force_on_actor_n: totals.max_contact_force_on_uke_n,
        contact_impulse_on_actor_ns: totals.contact_impulse_on_uke_ns,
        max_deviation_m: totals.max_deviation_m,
        max_speed_mps: totals.max_speed_mps,
        time_saturated_s: totals.time_saturated_s,
        work_j: totals.work_j,
      };
    }

    const recording: Recording = {
      schema: RECORDING_SCHEMA_ID,
      meta: {
        scenario_id: scenario.id,
        scenario_sha256: scenarioSha256(scenario),
        engine_version: options.engineVersion ?? "unknown",
        mujoco_version: mj.mj_versionString(),
        physics_hz: PHYSICS_HZ,
        record_hz: scenario.record_hz,
        frame: "x_forward_y_left_z_up",
        quat_order: "wxyz",
      },
      model: { mjcf: built.mjcf, bodies: built.bodies, joints: built.joints },
      samples,
      peaks: obstacles.length > 0 ? { obstacles: obstaclePeaks, effectors: peaks } : { effectors: peaks },
      summary,
      diagnostics: probe.diagnostics(),
    };
    const report = detectArtifacts(recording);
    summary.artifacts = report.artifacts;
    summary.artifacts_not_evaluated = report.not_evaluated;
    return recording;
  } finally {
    data.delete();
    model.delete();
  }
}
