import loadMujoco from "@mujoco/mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";
import { renderableGeomType, type PhysicsGeomPose, type PhysicsGeomSpec } from "./geometry";

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
  modulePromise ??= loadMujoco({
    locateFile: (path: string) => path.endsWith(".wasm") ? wasmUrl : path,
  }).catch((error: unknown) => {
    modulePromise = undefined; // allow retry after transient WASM fetch failure
    throw error;
  });
  return modulePromise;
}

/** Real MuJoCo rigid-body simulation. All state is copied before returning. */
export class MujocoSimulation {
  private readonly module: Module;
  private model: InstanceType<Module["MjModel"]> | undefined;
  private data: InstanceType<Module["MjData"]> | undefined;

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
  }

  private ready(): { model: NonNullable<MujocoSimulation["model"]>; data: NonNullable<MujocoSimulation["data"]> } {
    if (!this.model || !this.data) throw new Error("La simulación está cerrada");
    return { model: this.model, data: this.data };
  }

  reset(): void {
    const { model, data } = this.ready();
    this.module.mj_resetData(model, data);
    this.module.mj_forward(model, data);
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
      beforeStep?.(data.time);
      this.module.mj_step(model, data);
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

  dispose(): void {
    this.data?.delete();
    this.model?.delete();
    this.data = undefined;
    this.model = undefined;
  }
}
