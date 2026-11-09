/** Geometry snapshots copied from MuJoCo's WASM memory.
 * All positions and rotation matrices are in MuJoCo's Z-up world frame.
 */
export type RenderableGeomType = "sphere" | "capsule" | "ellipsoid" | "cylinder" | "box";

export type PhysicsGeomSpec = Readonly<{
  id: number;
  shape: RenderableGeomType;
  size: readonly [number, number, number];
  rgba: readonly [number, number, number, number];
}>;

export type PhysicsGeomPose = Readonly<{
  id: number;
  position: readonly [number, number, number];
  rotation: readonly [number, number, number, number, number, number, number, number, number];
}>;

/** mjtGeom numeric values are defined in the MuJoCo public API. */
export function renderableGeomType(code: number): RenderableGeomType | undefined {
  switch (code) {
    case 2: return "sphere";
    case 3: return "capsule";
    case 4: return "ellipsoid";
    case 5: return "cylinder";
    case 6: return "box";
    default: return undefined; // plane, mesh etc. handled separately or unsupported
  }
}
