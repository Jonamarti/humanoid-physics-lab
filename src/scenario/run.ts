/**
 * Ejecutor de escenarios `hpl.scenario.v1` (solo Node): construye el MJCF, asienta 0,5 s sin efectores,
 * aplica los efectores antes de cada `mj_step` y muestrea a `record_hz`. Usa el módulo MuJoCo
 * directamente (sin `MujocoSimulation`). La matemática de la fuerza en un punto es la de
 * `src/physics/mujoco.ts`: fuerza en `xfrc_applied` y par `r × F` respecto al centro de masas del cuerpo.
 *
 * `Recording.samples` y `peaks` conservan la precisión completa; el redondeo a 6 cifras
 * se aplica al escribir (`src/recording/write.ts`).
 */
import { createHash } from "node:crypto";
import { FALL_MIN_TILT_DEG, classifyTilt, fallDirectionDeg, tiltDeg } from "../metrics/outcome";
import { RECORDING_SCHEMA_ID, type ContactSample, type EffectorPeaks, type EffectorSample, type Recording, type RecordingSample } from "../recording/schema";
import { decodeSolvedContactForce, hasCompleteSolvedContactRows } from "../physics/contactForces";
import { buildScenarioModel } from "./buildModel";
import { readEngineVersion } from "./engineVersion";
import { compareExpect } from "./expect";
import { getMujoco } from "./mujocoModule";
import type { EffectorSpec, Outcome, Scenario, Vec3 } from "./schema";
import { PHYSICS_HZ, PHYSICS_TIMESTEP_S } from "./worldOptions";

/** Asentamiento previo sin efectores, en s. El tiempo del escenario empieza después. */
export const SETTLE_S = 0.5;

/** Contactos con fuerza normal por debajo de este valor (N) no se graban. */
const MIN_CONTACT_NORMAL_N = 1e-6;

export interface RunOptions {
  /** Versión del motor para `meta.engine_version`; por defecto, la de `package.json` del directorio de trabajo. */
  engineVersion?: string;
  /**
   * Observador opcional, llamado tras grabar cada muestra con el modelo y los datos de MuJoCo (solo lectura).
   * Lo usa la prueba de reproducción para comparar `xpos` de la simulación real con `mj_forward`.
   */
  onSample?: (model: unknown, data: unknown, sampleIndex: number) => void;
}

/** Ganancia (0 a 1) de la forma de onda en el instante `t` (s del escenario) de un efector. */
export function waveformGain(effector: Pick<EffectorSpec, "start_s" | "duration_s" | "waveform">, t: number): number {
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
  return createHash("sha256").update(JSON.stringify(scenario)).digest("hex");
}

interface PendingWork {
  force: Vec3;
  velocity: Vec3;
}

export async function runScenario(scenario: Scenario, options: RunOptions = {}): Promise<Recording> {
  const mj = await getMujoco();
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

    const effectors = scenario.effectors.map((spec) => ({
      spec,
      bodyId: built.bodies.indexOf(spec.body),
      local: spec.local_point_m,
    }));
    const peaks: Record<string, EffectorPeaks> = {};
    for (const e of effectors) peaks[e.spec.id] = { max_force_n: 0, impulse_ns: [0, 0, 0], work_j: 0 };
    const pending: Array<PendingWork | null> = effectors.map(() => null);

    const xfrc = data.xfrc_applied as Float64Array;
    const bodyOfGeom = model.geom_bodyid as Int32Array;
    const rootOfBody = model.body_rootid as Int32Array;
    const pyramidal = Boolean(mj.mj_isPyramidal(model));

    // 1. Asentamiento sin efectores.
    for (let i = 0; i < settleSteps; i++) {
      xfrc.fill(0);
      mj.mj_step(model, data);
    }

    // 2. Escenario. `data.time` no se usa: el tiempo del escenario es `i * dt`.
    const updateKinematics = (): void => {
      mj.mj_kinematics(model, data);
      mj.mj_comPos(model, data);
      mj.mj_comVel(model, data);
    };

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
    const initialRotation = Array.from((data.xmat as Float64Array).subarray(9 * actorBodyId, 9 * actorBodyId + 9));
    const startXY: [number, number] = [(data.xpos as Float64Array)[3 * actorBodyId]!, (data.xpos as Float64Array)[3 * actorBodyId + 1]!];

    const samples: RecordingSample[] = [];
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

      // Veredicto a 500 Hz sobre el estado actual.
      const xmat = data.xmat as Float64Array;
      const tilt = tiltDeg(xmat, 9 * actorBodyId);
      if (tilt > maxTilt) {
        maxTilt = tilt;
        directionAtMax = fallDirectionDeg(xmat, 9 * actorBodyId);
      }
      if (directionAtFall === undefined && tilt > FALL_MIN_TILT_DEG) {
        directionAtFall = fallDirectionDeg(xmat, 9 * actorBodyId);
      }
      const cls = classifyTilt(tilt);
      if (cls !== currentClass) {
        currentClass = cls;
        classSinceS = t;
      }

      // Efectores: fuerza durante el paso [t, t + dt], evaluada en el punto medio.
      xfrc.fill(0);
      const sampleEffectors: EffectorSample[] = [];
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
        const gain = waveformGain(e.spec, t + dt / 2);
        let force: Vec3 = [0, 0, 0];
        if (gain !== 0) {
          const f = e.spec.force_n;
          const scaled: Vec3 = [f[0] * gain, f[1] * gain, f[2] * gain];
          if (e.spec.frame === "world") force = scaled;
          else if (e.spec.frame === "actor_initial") force = rotate(initialRotation, 0, scaled);
          else force = rotate(xmat, 9 * e.bodyId, scaled);
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
          const magnitude = Math.hypot(force[0], force[1], force[2]);
          if (magnitude > peak.max_force_n) peak.max_force_n = magnitude;
          peak.impulse_ns = [
            peak.impulse_ns[0] + force[0] * dt,
            peak.impulse_ns[1] + force[1] * dt,
            peak.impulse_ns[2] + force[2] * dt,
          ];
        }
        pending[k] = i < totalSteps ? { force, velocity } : null;
        sampleEffectors.push({ id: e.spec.id, point_m: point, force_n: force });
      });

      if (i % stepsPerSample === 0) {
        mj.mj_forward(model, data); // contactos y fuerzas coherentes con el wrench ya aplicado
        samples.push({
          t,
          qpos: Array.from(data.qpos as Float64Array),
          qvel: Array.from(data.qvel as Float64Array),
          effectors: sampleEffectors,
          contacts: readContacts(),
        });
        options.onSample?.(model, data, samples.length - 1);
      }

      if (i === totalSteps) {
        finalTilt = tilt;
        finalComHeight = (data.xipos as Float64Array)[3 * actorBodyId + 2]!;
        finalXY = [(data.xpos as Float64Array)[3 * actorBodyId]!, (data.xpos as Float64Array)[3 * actorBodyId + 1]!];
        break;
      }
      mj.mj_step(model, data);
    }

    const outcome = currentClass!;
    const summary = {
      outcome,
      direction_deg: directionAtFall !== undefined ? directionAtFall : directionAtMax,
      t_outcome_s: classSinceS,
      final_tilt_deg: finalTilt,
      max_tilt_deg: maxTilt,
      final_com_height_m: finalComHeight,
      slide_m: Math.hypot(finalXY[0] - startXY[0], finalXY[1] - startXY[1]),
      expect: "none" as "pass" | "fail" | "none",
      expect_diffs: [] as string[],
    };
    if (scenario.expect) {
      const result = compareExpect(summary, scenario.expect);
      summary.expect = result === "pass" ? "pass" : "fail";
      summary.expect_diffs = result === "pass" ? [] : result;
    }

    return {
      schema: RECORDING_SCHEMA_ID,
      meta: {
        scenario_id: scenario.id,
        scenario_sha256: scenarioSha256(scenario),
        engine_version: options.engineVersion ?? readEngineVersion(),
        mujoco_version: mj.mj_versionString(),
        physics_hz: PHYSICS_HZ,
        record_hz: scenario.record_hz,
        frame: "x_forward_y_left_z_up",
        quat_order: "wxyz",
      },
      model: { mjcf: built.mjcf, bodies: built.bodies, joints: built.joints },
      samples,
      peaks: { effectors: peaks },
      summary,
    };
  } finally {
    data.delete();
    model.delete();
  }
}
