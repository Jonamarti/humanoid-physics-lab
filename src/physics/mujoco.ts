import loadMujoco from "@mujoco/mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import { renderableGeomType, type PhysicsGeomPose, type PhysicsGeomSpec } from "./geometry";
import { buildBalanceObservation, type BalanceObservation, type FootGeom, type XYZ, type Matrix3 } from "../control/balanceSensors";
import { makeStandingController, stepActiveStanding, type StandingController } from "../control/standingBaseline";
import { ExperimentalStepSequence, applyStepJointTargets, type StepSnapshot } from "../control/experimentalStep";
import { applyCartesianFootImpedance } from "../control/cartesianFootImpedance";
import { rootTiltDegrees } from "./standingPush";
import type { FootSide } from "../control/recoveryPlanner";
import { capturePointXY, insideConvexHullXY, type XY } from "../control/balanceSensors";
import { summarizeGroundSupport, type GroundContactSample, type GroundSupportObservation } from "../control/footContacts";
import { decodeSolvedContactForce } from "./contactForces";

export type PhysicsState = Readonly<{
  time: number;
  qpos: number[];
  qvel: number[];
  bodies: number;
}>;

type Module = Awaited<ReturnType<typeof loadMujoco>>;

let modulePromise: Promise<Module> | undefined;

/** Load the official single-thread MuJoCo runtime; the asset URL is Vite-base aware. */
export function loadPhysicsModule(): Promise<Module> {
  // Cache the INITIALIZATION itself, not merely its promise after spawning it.
  // Starting extra Emscripten runtimes during a parameter sweep leaks memory.
  if (!modulePromise) {
    // In Node/Vitest the package locates mujoco.wasm itself. In browsers
    // Vite must supply a base-aware URL (including the GitHub Pages prefix).
    const load = typeof window === "undefined"
      ? loadMujoco()
      : loadMujoco({ locateFile: (path: string) => path.endsWith(".wasm") ? wasmUrl : path });
    modulePromise = load.catch((error: unknown) => {
      modulePromise = undefined; // allow retry after transient WASM fetch failure
      throw error;
    });
  }
  return modulePromise;
}

/** Real MuJoCo rigid-body simulation. All state is copied before returning. */
export class MujocoSimulation {
  private readonly module: Module;
  private model: InstanceType<Module["MjModel"]> | undefined;
  private data: InstanceType<Module["MjData"]> | undefined;
  private standingController: StandingController | undefined;
  private stepExperiment: ExperimentalStepSequence | undefined;
  private stepExperimentTicks = 0;
  private lastStepFootWorld: XYZ | undefined;
  private lastStepFootTimeS = 0;
  private footSpeedWorldMps: XYZ = [0,0,0];
  private cartesianStepScale = 0;

  private constructor(module: Module) {
    this.module = module;
  }

  static async create(xml: string): Promise<MujocoSimulation> {
    const module = await loadPhysicsModule();
    const sim = new MujocoSimulation(module);
    sim.loadModel(xml);
    return sim;
  }

  loadModel(xml: string): void {
    if (!xml.trim()) throw new Error("El modelo MJCF/XML está vacío");
    // Load new assets before releasing the prior working scene.
    const nextModel = this.module.MjModel.from_xml_string(xml);
    if (!nextModel) throw new Error("No se pudo compilar MJCF");
    let nextData: InstanceType<Module["MjData"]> | undefined;
    try {
      nextData = new this.module.MjData(nextModel);
      if (!nextData) throw new Error("No se pudo crear MjData");
      this.module.mj_forward(nextModel, nextData);
    } catch (error) {
      nextData?.delete();
      nextModel.delete();
      throw error;
    }
    this.data?.delete();
    this.model?.delete();
    this.model = nextModel;
    this.data = nextData;
    this.standingController = undefined;
    this.stepExperiment = undefined;
    this.stepExperimentTicks = 0;
    this.lastStepFootWorld = undefined;
    this.footSpeedWorldMps = [0,0,0];
  }

  private ready(): { model: NonNullable<MujocoSimulation["model"]>; data: NonNullable<MujocoSimulation["data"]> } {
    if (!this.model || !this.data) throw new Error("La simulación está cerrada");
    return { model: this.model, data: this.data };
  }

  reset(): void {
    const { model, data } = this.ready();
    const standingWasEnabled = this.standingController !== undefined;
    this.module.mj_resetData(model, data);
    this.module.mj_forward(model, data);
    this.standingController = standingWasEnabled ? makeStandingController(model, data) : undefined;
    this.stepExperiment = undefined;
    this.stepExperimentTicks = 0;
    this.lastStepFootWorld = undefined;
    this.footSpeedWorldMps = [0,0,0];
  }

  /** Enables bounded internal-joint torque control on the public humanoid.
   * This is a nominal-stance experiment, not a validated push recovery policy.
   */
  enableExperimentalStanding(): void {
    const { model, data } = this.ready();
    this.module.mj_forward(model, data);
    this.standingController = makeStandingController(model, data);
  }

  /** Opt-in voluntary recovery-step test. Not a validated walking controller.
   * Foot trajectories and contacts are all determined by real MuJoCo dynamics.
   */
  enableExperimentalStepTrial(
    swingFoot: FootSide,
    targetDeltaXY: XY = [0.10, 0],
    startTimeS = 0.8,
    cartesianStepScale = 0,
  ): void {
    if (!Number.isFinite(cartesianStepScale) || cartesianStepScale<0 || cartesianStepScale>1) {
      throw new RangeError("Escala de control de pie fuera de 0..1");
    }
    if (!this.standingController) throw new Error("Activa bipedestación antes de probar un paso");
    this.stepExperiment = new ExperimentalStepSequence(swingFoot, targetDeltaXY, startTimeS);
    this.stepExperimentTicks = 0;
    this.cartesianStepScale = cartesianStepScale;
    this.lastStepFootWorld = this.siteWorldPosition(swingFoot+"_sole");
    this.lastStepFootTimeS = this.ready().data.time;
    this.footSpeedWorldMps = [0,0,0];
  }

  experimentalStepStatus(): StepSnapshot | null {
    return this.stepExperiment?.snapshot() ?? null;
  }

  private sampleStepExperiment(): void {
    if (!this.stepExperiment) return;
    const { model, data } = this.ready();
    const side = this.stepExperiment.swingSide;
    this.module.mj_subtreeVel(model,data);
    const ground = this.humanoidGroundSupportObservation();
    const com = data.subtree_com as Float64Array;
    const vel = data.subtree_linvel as Float64Array;
    const comWorldM: XYZ = [com[3]!,com[4]!,com[5]!];
    const comVelocityWorldMps: XYZ = [vel[3]!,vel[4]!,vel[5]!];
    const loaded = ground.loadedSupportPolygonXY;
    const captureXY = capturePointXY([comWorldM[0],comWorldM[1]],
      [comVelocityWorldMps[0],comVelocityWorldMps[1]],
      Math.max(0.01,comWorldM[2]));
    const pelvis = data.qpos as Float64Array;
    this.stepExperiment.observe({
      timeS: data.time,
      comWorldM,
      comVelocityWorldMps,
      comProjectionInsideLoadedSupport: insideConvexHullXY(
        [comWorldM[0],comWorldM[1]],loaded),
      capturePointInsideLoadedSupport: insideConvexHullXY(captureXY,loaded),
      pelvisHeightM: pelvis[2]!,
      rootTiltDeg: rootTiltDegrees(Array.from(pelvis.subarray(0,7))),
      swingSoleWorldM: this.siteWorldPosition(side+"_sole"),
      supportSoleWorldM: this.siteWorldPosition((side==="left"?"right":"left")+"_sole"),
      swingLoadN: ground[side].verticalLoadN,
      supportLoadN: ground[side==="left" ? "right" : "left"].verticalLoadN,
    });
  }

  /** Advances count physical steps. Each optional callback runs immediately
   * before the step so force pulses respect physical time, not screen FPS.
   * External wrenches are cleared on EVERY step to avoid stale impulses.
   */
  step(count = 1, beforeStep?: (timeS: number) => void): PhysicsState {
    const { model, data } = this.ready();
    if (!Number.isSafeInteger(count) || count < 0 || count > 100_000) {
      throw new RangeError("Número de pasos inválido");
    }
    for (let i = 0; i < count; i++) {
      (data.xfrc_applied as Float64Array).fill(0);
      if (this.standingController) {
        this.module.mj_subtreeVel(model, data);
        const command = this.stepExperiment?.command(
          data.time, this.standingController.referenceComXY);
        const controller = command ? {
          ...this.standingController,
          referenceComXY: command.referenceComXY,
        } : this.standingController;
        stepActiveStanding(model, data, controller);
        if (command) {
          applyStepJointTargets(model,data,
            this.standingController.gains,command.targetsRad);
          if (command.soleTargetWorldM && this.stepExperiment &&
              this.cartesianStepScale > 0) {
            applyCartesianFootImpedance(model,data,this.standingController.gains,
              this.stepExperiment.swingSide,{
                targetWorldM:command.soleTargetWorldM,
                currentWorldM:this.siteWorldPosition(this.stepExperiment.swingSide+"_sole"),
                measuredVelocityWorldMps:this.footSpeedWorldMps,
                gainsNPerM:[780,780,900],
                dampingNsPerM:[90,90,80],
                maxFootForceN:150,
                blend:command.cartesianBlend*this.cartesianStepScale,
              });
          }
        }
      }
      beforeStep?.(data.time);
      this.module.mj_step(model, data);
      if (this.stepExperiment) {
        const point = this.siteWorldPosition(this.stepExperiment.swingSide+"_sole");
        const dt = data.time-this.lastStepFootTimeS;
        if (this.lastStepFootWorld && dt > 0) {
          const instant:XYZ = [
            (point[0]-this.lastStepFootWorld[0])/dt,
            (point[1]-this.lastStepFootWorld[1])/dt,
            (point[2]-this.lastStepFootWorld[2])/dt,
          ];
          const prev = this.footSpeedWorldMps;
          this.footSpeedWorldMps = [
            0.75*prev[0]+0.25*instant[0],
            0.75*prev[1]+0.25*instant[1],
            0.75*prev[2]+0.25*instant[2],
          ];
        }
        this.lastStepFootWorld = point;
        this.lastStepFootTimeS = data.time;
      }
      if (this.stepExperiment && ++this.stepExperimentTicks % 10 === 0) {
        // Read solved loads at 50 Hz, with the prior physical step's
        // contact solution (a <=2ms lag), not on render frames.
        this.sampleStepExperiment();
      }
    }
    return this.state();
  }

  /** Apply a world-force at a point given relative to the body's COM
   * in the current BODY-LOCAL axes. MuJoCo expects world force + world torque.
   * Call from step(..., beforeStep) so every interval is applied exactly once.
   */
  applyForceAtBodyLocalPoint(
    bodyIndex: number,
    localComOffsetM: readonly [number, number, number],
    worldForceN: readonly [number, number, number],
  ): void {
    const { model, data } = this.ready();
    if (!Number.isSafeInteger(bodyIndex) || bodyIndex <= 0 || bodyIndex >= model.nbody ||
        ![...localComOffsetM, ...worldForceN].every(Number.isFinite)) {
      throw new RangeError("Cuerpo o vector de fuerza inválido");
    }
    const m = data.xmat as Float64Array;
    const b = bodyIndex * 9;
    const [x, y, z] = localComOffsetM;
    const rx = m[b]! * x + m[b + 1]! * y + m[b + 2]! * z;
    const ry = m[b + 3]! * x + m[b + 4]! * y + m[b + 5]! * z;
    const rz = m[b + 6]! * x + m[b + 7]! * y + m[b + 8]! * z;
    const [fx, fy, fz] = worldForceN;
    const w = data.xfrc_applied as Float64Array;
    const offset = bodyIndex * 6;
    w[offset]! += fx;
    w[offset + 1]! += fy;
    w[offset + 2]! += fz;
    w[offset + 3]! += ry * fz - rz * fy;
    w[offset + 4]! += rz * fx - rx * fz;
    w[offset + 5]! += rx * fy - ry * fx;
  }

  /** Current physical world position of a named MuJoCo site (copied). */
  siteWorldPosition(siteName: string): readonly [number, number, number] {
    const { model, data } = this.ready();
    if (!siteName) throw new RangeError("Nombre de sitio vacío");
    const id = model.site(siteName).id;
    if (id < 0 || id >= model.nsite) throw new RangeError("Sitio físico inexistente");
    const p = data.site_xpos as Float64Array;
    const xyz: readonly [number, number, number] = [
      p[id * 3]!, p[id * 3 + 1]!, p[id * 3 + 2]!,
    ];
    if (!xyz.every(Number.isFinite)) throw new Error("Punto físico no finito");
    return xyz;
  }

  /** Apply an EXTERNAL world-space force at a real body-attached site.
   * Converts its world-space moment arm into a physical wrench about the COM.
   * Intended to be called only inside step(..., beforeStep), after clearing
   * previous xfrc_applied. This never actuates the unanchored pelvis root.
   */
  applyWorldForceAtSite(
    siteName: string,
    forceWorldN: readonly [number, number, number],
  ): void {
    const { model, data } = this.ready();
    if (!siteName || !forceWorldN.every(Number.isFinite)) {
      throw new RangeError("Sitio o fuerza inválidos");
    }
    const siteId = model.site(siteName).id;
    const bodyId = (model.site_bodyid as Int32Array)[siteId]!;
    if (!Number.isInteger(bodyId) || bodyId <= 0 || bodyId >= model.nbody) {
      throw new RangeError("El punto de impacto debe pertenecer a un cuerpo dinámico");
    }
    const positions = data.site_xpos as Float64Array;
    const centers = data.xipos as Float64Array;
    const wrench = data.xfrc_applied as Float64Array;
    const a = siteId * 3;
    const b = bodyId * 3;
    const rx = positions[a]! - centers[b]!;
    const ry = positions[a + 1]! - centers[b + 1]!;
    const rz = positions[a + 2]! - centers[b + 2]!;
    const [fx, fy, fz] = forceWorldN;
    const o = bodyId * 6;
    wrench[o]! += fx;
    wrench[o + 1]! += fy;
    wrench[o + 2]! += fz;
    wrench[o + 3]! += ry * fz - rz * fy;
    wrench[o + 4]! += rz * fx - rx * fz;
    wrench[o + 5]! += rx * fy - ry * fx;
  }

  state(): PhysicsState {
    const { model, data } = this.ready();
    const qpos = Array.from(data.qpos as Float64Array);
    const qvel = Array.from(data.qvel as Float64Array);
    if (![data.time, ...qpos, ...qvel].every(Number.isFinite)) {
      throw new Error("El integrador devolvió valores no finitos");
    }
    return { time: data.time, qpos, qvel, bodies: model.nbody };
  }

  /** Immutable descriptions of the currently loaded primitive collision geoms. */
  geometrySpecs(): PhysicsGeomSpec[] {
    const { model } = this.ready();
    const kinds = model.geom_type as Int32Array;
    const sizes = model.geom_size as Float64Array;
    const colors = model.geom_rgba as Float32Array;
    const specs: PhysicsGeomSpec[] = [];
    for (let id = 0; id < model.ngeom; id++) {
      const shape = renderableGeomType(kinds[id]!);
      if (!shape) continue; // floor plane is rendered independently
      const a = id * 3;
      const c = id * 4;
      specs.push({
        id,
        shape,
        size: [sizes[a]!, sizes[a + 1]!, sizes[a + 2]!],
        rgba: [colors[c]!, colors[c + 1]!, colors[c + 2]!, colors[c + 3]!],
      });
    }
    return specs;
  }

  /** Dynamic world-space transforms from MuJoCo; not animation guesses. */
  geometryPoses(): PhysicsGeomPose[] {
    const { model, data } = this.ready();
    const kinds = model.geom_type as Int32Array;
    const xyz = data.geom_xpos as Float64Array;
    const matrices = data.geom_xmat as Float64Array;
    const poses: PhysicsGeomPose[] = [];
    for (let id = 0; id < model.ngeom; id++) {
      if (!renderableGeomType(kinds[id]!)) continue;
      const p = id * 3;
      const m = id * 9;
      poses.push({
        id,
        position: [xyz[p]!, xyz[p + 1]!, xyz[p + 2]!],
        rotation: [
          matrices[m]!, matrices[m + 1]!, matrices[m + 2]!,
          matrices[m + 3]!, matrices[m + 4]!, matrices[m + 5]!,
          matrices[m + 6]!, matrices[m + 7]!, matrices[m + 8]!,
        ],
      });
    }
    return poses;
  }

  /** Observe physical centre of mass, COM velocity and the projected foot
   * geometry. This is a DIAGNOSTIC, not a ground-force/COP sensor.
   * Only valid for the public humanoid_v1.xml with one free pelvis root.
   */
  humanoidBalanceObservation(): BalanceObservation {
    const { model, data } = this.ready();
    this.module.mj_subtreeVel(model, data);
    const bodyRoot = 1; // public humanoid: world=0, free pelvis=1
    if (model.nbody < 15) throw new Error("El escenario actual no contiene humanoide_v1");
    const comArr = data.subtree_com as Float64Array;
    const velArr = data.subtree_linvel as Float64Array;
    const from3 = (arr: ArrayLike<number>, i: number): XYZ =>
      [arr[i]!, arr[i + 1]!, arr[i + 2]!];
    const foot = (geomId: number): FootGeom => {
      const xyz = data.geom_xpos as Float64Array;
      const rot = data.geom_xmat as Float64Array;
      const sizes = model.geom_size as Float64Array;
      const matIndex = geomId * 9;
      return {
        centerM: from3(xyz, geomId * 3),
        halfSizeM: from3(sizes, geomId * 3),
        rotationWorld: Array.from(rot.subarray(matIndex, matIndex + 9)) as unknown as Matrix3,
      };
    };
    const left = foot(model.geom("left_foot_geom").id);
    const right = foot(model.geom("right_foot_geom").id);
    const totalMassKg = Array.from(model.body_mass as Float64Array).reduce((sum, mass) => sum + mass, 0);
    return buildBalanceObservation({
      timeS: data.time,
      totalMassKg,
      comWorldM: from3(comArr, bodyRoot * 3),
      comVelocityWorldMps: from3(velArr, bodyRoot * 3),
      left,
      right,
      contactCount: data.ncon,
    });
  }

  /** Measured solved forces at actual floor-foot contacts. Unlike the
   * geometrical candidate hull this excludes airborne or unweighted feet.
   * Force is decoded from solved efc_force, NEVER estimated from ncon.
   * contact.frame[0..2] is MuJoCo's contact normal in WORLD axes; the
   * local normal force is its first contact-wrench component. Each Embind
   * proxy must be deleted, including the vector returned by data.contact.
   */
  humanoidGroundSupportObservation(): GroundSupportObservation {
    const { model, data } = this.ready();
    if (model.nbody < 15) throw new Error("El escenario actual no contiene humanoid_v1");
    const floorGeomId = model.geom("floor").id;
    const leftFootGeomId = model.geom("left_foot_geom").id;
    const rightFootGeomId = model.geom("right_foot_geom").id;
    const footGeom = (geomId: number): FootGeom => {
      const positions = data.geom_xpos as Float64Array;
      const rotations = data.geom_xmat as Float64Array;
      const sizes = model.geom_size as Float64Array;
      const p = geomId * 3;
      const r = geomId * 9;
      return {
        centerM: [positions[p]!, positions[p + 1]!, positions[p + 2]!],
        rotationWorld: [
          rotations[r]!, rotations[r + 1]!, rotations[r + 2]!,
          rotations[r + 3]!, rotations[r + 4]!, rotations[r + 5]!,
          rotations[r + 6]!, rotations[r + 7]!, rotations[r + 8]!,
        ],
        halfSizeM: [sizes[p]!, sizes[p + 1]!, sizes[p + 2]!],
      };
    };
    const samples: GroundContactSample[] = [];
    const contacts = data.contact;
    const constraintForce = data.efc_force as Float64Array;
    const pyramidal = this.module.mj_isPyramidal(model);
    try {
      for (let i = 0; i < contacts.size(); i++) {
        const contact = contacts.get(i);
        if (!contact) throw new Error("Contacto MuJoCo inaccesible");
        try {
          const first = contact.geom1;
          const second = contact.geom2;
          if ((first !== floorGeomId && second !== floorGeomId) ||
              ![leftFootGeomId, rightFootGeomId].includes(
                first === floorGeomId ? second : first
              )) continue;
          if (contact.efc_address < 0) continue; // excluded / unsolved
          // Decode the solved normal *and* tangents from efc_force with
          // the same pyramid formula as MuJoCo's mju_decodePyramid.
          // This avoids the WASM mj_contactForce output-buffer bug.
          const solved = decodeSolvedContactForce({
            solverRows: constraintForce,
            efcAddress: contact.efc_address,
            dim: contact.dim,
            pyramidal: Boolean(pyramidal),
            friction: contact.friction as Float64Array,
            adhesionN: contact.adhesion,
          });
          const point = contact.pos as Float64Array;
          const frame = contact.frame as Float64Array;
          const [u,v] = solved.tangentN;
          const tangentWorld: XYZ = [
            u*frame[3]!+v*frame[6]!,
            u*frame[4]!+v*frame[7]!,
            u*frame[5]!+v*frame[8]!,
          ];
          samples.push({
            geom1: first,
            geom2: second,
            positionWorldM: [point[0]!, point[1]!, point[2]!],
            contactNormalWorld: [frame[0]!, frame[1]!, frame[2]!],
            contactNormalForceN: Math.max(0, solved.normalN),
            tangentForceWorldN: tangentWorld,
            frictionUtilization: solved.frictionUtilization,
          });
        } finally {
          contact.delete();
        }
      }
    } finally {
      contacts.delete();
    }
    return summarizeGroundSupport({
      floorGeomId, leftFootGeomId, rightFootGeomId,
      leftFoot: footGeom(leftFootGeomId),
      rightFoot: footGeom(rightFootGeomId),
      contacts: samples,
    });
  }

  dispose(): void {
    this.stepExperiment = undefined;
    this.standingController = undefined;
    this.data?.delete();
    this.model?.delete();
    this.data = undefined;
    this.model = undefined;
  }
}
