/**
 * Cinemática directa de una grabación: carga `model.mjcf` en MuJoCo SOLO para obtener geometrías y poses.
 * Escribe `qpos` y llama a `mj_forward`; NUNCA a `mj_step` (la reproducción no necesita física).
 * Sin DOM ni Three.js: sirve al visor y a las pruebas.
 */
import { renderableGeomType, type PhysicsGeomPose, type PhysicsGeomSpec } from "../physics/geometry";
import type { MujocoModule } from "../scenario/mujocoModule";

export class ReplayKinematics {
  private readonly model: InstanceType<MujocoModule["MjModel"]>;
  private readonly data: InstanceType<MujocoModule["MjData"]>;
  private deleted = false;

  constructor(private readonly mj: MujocoModule, mjcf: string) {
    this.model = mj.MjModel.from_xml_string(mjcf);
    this.data = new mj.MjData(this.model);
  }

  get nq(): number { return this.model.nq; }

  /** `jnt_type` de cada articulación (0 = libre, 1 = esférica, 2 = deslizante, 3 = bisagra). */
  jointTypes(): number[] {
    return Array.from(this.model.jnt_type as Int32Array);
  }

  /** Descripciones de las geometrías primitivas (el plano del suelo lo dibuja el visor aparte). */
  geomSpecs(): PhysicsGeomSpec[] {
    const kinds = this.model.geom_type as Int32Array;
    const sizes = this.model.geom_size as Float64Array;
    const colors = this.model.geom_rgba as Float32Array;
    const specs: PhysicsGeomSpec[] = [];
    for (let id = 0; id < this.model.ngeom; id++) {
      const shape = renderableGeomType(kinds[id]!);
      if (!shape) continue;
      const a = id * 3;
      const c = id * 4;
      specs.push({
        id, shape,
        size: [sizes[a]!, sizes[a + 1]!, sizes[a + 2]!],
        rgba: [colors[c]!, colors[c + 1]!, colors[c + 2]!, colors[c + 3]!],
      });
    }
    return specs;
  }

  /** Escribe `qpos`, anula `qvel` y calcula la cinemática directa (`mj_forward`). No avanza el tiempo. */
  setQpos(qpos: readonly number[]): void {
    if (qpos.length !== this.model.nq) throw new Error(`qpos tiene ${qpos.length} valores y el modelo ${this.model.nq}`);
    const target = this.data.qpos as Float64Array;
    for (let i = 0; i < qpos.length; i++) target[i] = qpos[i]!;
    (this.data.qvel as Float64Array).fill(0);
    this.mj.mj_forward(this.model, this.data);
  }

  /** Posiciones de los cuerpos (`xpos`, 3 por cuerpo) del último `setQpos`. */
  bodyPositions(): Float64Array {
    return Float64Array.from(this.data.xpos as Float64Array);
  }

  /** Poses de las geometrías primitivas del último `setQpos`. */
  geomPoses(): PhysicsGeomPose[] {
    const kinds = this.model.geom_type as Int32Array;
    const xyz = this.data.geom_xpos as Float64Array;
    const m = this.data.geom_xmat as Float64Array;
    const poses: PhysicsGeomPose[] = [];
    for (let id = 0; id < this.model.ngeom; id++) {
      if (!renderableGeomType(kinds[id]!)) continue;
      const p = 3 * id;
      const r = 9 * id;
      poses.push({
        id,
        position: [xyz[p]!, xyz[p + 1]!, xyz[p + 2]!],
        rotation: [m[r]!, m[r + 1]!, m[r + 2]!, m[r + 3]!, m[r + 4]!, m[r + 5]!, m[r + 6]!, m[r + 7]!, m[r + 8]!],
      });
    }
    return poses;
  }

  dispose(): void {
    if (this.deleted) return;
    this.deleted = true;
    this.data.delete();
    this.model.delete();
  }
}
