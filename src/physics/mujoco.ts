import loadMujoco from "@mujoco/mujoco";
import wasmUrl from "@mujoco/mujoco/mujoco.wasm?url";

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

  /** Advances one fixed physics step from the MJCF option timestep. */
  step(count = 1): PhysicsState {
    const { model, data } = this.ready();
    if (!Number.isSafeInteger(count) || count < 0 || count > 100_000) {
      throw new RangeError("Número de pasos inválido");
    }
    for (let i = 0; i < count; i++) this.module.mj_step(model, data);
    return this.state();
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

  dispose(): void {
    this.data?.delete();
    this.model?.delete();
    this.data = undefined;
    this.model = undefined;
  }
}
