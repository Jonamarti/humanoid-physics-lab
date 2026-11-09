import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { PhysicsGeomPose, PhysicsGeomSpec } from "../physics/geometry";
import { mujocoToThreePosition } from "./coordinates";
import { forceArrowLengthM } from "./forceArrow";
import { projectBalanceToGround, type ProjectionStatus } from "./balanceProjection";
import type { XY, XYZ } from "../control/balanceSensors";

const MUJOCO_TO_THREE = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(1, 0, 0), -Math.PI / 2,
);

/** Three.js draws collision geometry using MuJoCo's own world transforms.
 * The simulation is never modified by the visualization.
 */
export class BallViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly sphere: THREE.Mesh;
  private readonly forceArrow: THREE.ArrowHelper;
  private readonly forceFlash: THREE.Mesh;
  private readonly forceFlashRing: THREE.Mesh;
  private readonly comMarker: THREE.Mesh;
  private readonly comGroundMarker: THREE.Mesh;
  private readonly comVerticalLine: THREE.Line;
  private readonly supportOutline: THREE.LineLoop;
  private readonly observer: ResizeObserver;
  private readonly articulated = new Map<number, THREE.Mesh>();
  private readonly rotationMatrix = new THREE.Matrix4();
  private readonly rotationQuaternion = new THREE.Quaternion();
  private disposed = false;

  constructor(private readonly container: HTMLElement) {
    this.scene.background = new THREE.Color("#102032");
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.02, 80);
    this.camera.position.set(2.7, 2.1, 3);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "low-power" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enablePan = true;
    this.controls.minDistance = 0.5;
    this.controls.maxDistance = 18;
    this.controls.target.set(0, 0.7, 0);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334a60, 2.4));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.7);
    keyLight.position.set(2, 4, 3);
    this.scene.add(keyLight);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(8, 8),
      new THREE.MeshStandardMaterial({ color: 0x243b51, roughness: 0.85 }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.004;
    this.scene.add(floor);

    const grid = new THREE.GridHelper(8, 24, 0x597b95, 0x36536b);
    grid.position.y = 0.003;
    this.scene.add(grid);

    this.sphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 32, 24),
      new THREE.MeshStandardMaterial({ color: 0x4fc6f1, roughness: 0.4, metalness: 0.08 }),
    );
    this.scene.add(this.sphere);
    this.forceArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0xffbd59, 0.18, 0.09);
    this.forceArrow.visible = false;
    this.scene.add(this.forceArrow);

    // Force highlight is visible only for physics steps which applied force:
    // emissive halo + camera-facing ring + vivid arrow, not merely color.
    this.forceFlash = new THREE.Mesh(
      new THREE.SphereGeometry(0.105, 18, 12),
      new THREE.MeshBasicMaterial({
        color: 0xff315f, transparent: true, opacity: 0.42,
        blending: THREE.AdditiveBlending, depthWrite: false,
      }),
    );
    this.forceFlashRing = new THREE.Mesh(
      new THREE.TorusGeometry(0.15, 0.022, 8, 40),
      new THREE.MeshBasicMaterial({
        color: 0xffe08b, transparent: true, opacity: 0.95,
        depthWrite: false, depthTest: false,
      }),
    );
    this.forceFlash.visible = false;
    this.forceFlashRing.visible = false;
    this.scene.add(this.forceFlash, this.forceFlashRing);

    this.comMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.051, 18, 12),
      new THREE.MeshBasicMaterial({ color: 0x50d9f9, depthTest: false, depthWrite: false }),
    );
    this.comGroundMarker = new THREE.Mesh(
      new THREE.CircleGeometry(0.075, 32),
      new THREE.MeshBasicMaterial({ color: 0x50d9f9, side: THREE.DoubleSide, depthTest: false, depthWrite: false }),
    );
    this.comGroundMarker.rotation.x = -Math.PI / 2;
    const dropGeometry = new THREE.BufferGeometry();
    dropGeometry.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(6), 3).setUsage(THREE.DynamicDrawUsage));
    this.comVerticalLine = new THREE.Line(dropGeometry,
      new THREE.LineDashedMaterial({ color: 0x5fe0f3, dashSize: 0.065, gapSize: 0.04, depthTest: false, depthWrite: false }));
    const hullGeometry = new THREE.BufferGeometry();
    hullGeometry.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(24), 3).setUsage(THREE.DynamicDrawUsage));
    hullGeometry.setDrawRange(0, 0);
    this.supportOutline = new THREE.LineLoop(hullGeometry,
      new THREE.LineBasicMaterial({ color: 0x74e0ae, depthTest: false, depthWrite: false }));
    this.comMarker.visible = false;
    this.comGroundMarker.visible = false;
    this.comVerticalLine.visible = false;
    this.supportOutline.visible = false;
    // Draw these diagnostics *after* the opaque humanoid and floor.
    // The COM often lies inside the pelvis mesh, so ordinary depth testing
    // hid it, defeating the purpose of the projection.
    for (const overlay of [this.comMarker, this.comGroundMarker,
      this.comVerticalLine, this.supportOutline]) {
      overlay.renderOrder = 90;
    }
    this.scene.add(this.comMarker, this.comGroundMarker,
      this.comVerticalLine, this.supportOutline);
    this.setSpherePosition([0, 0, 1.5]);

    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(this.container);
    this.resize();
    this.renderer.setAnimationLoop(() => {
      if (this.disposed) return;
      this.controls.update();
      if (this.forceFlashRing.visible) this.forceFlashRing.quaternion.copy(this.camera.quaternion);
      this.renderer.render(this.scene, this.camera);
    });
  }

  setSpherePosition(xyz: readonly [number, number, number]): void {
    const [x, y, z] = mujocoToThreePosition(xyz);
    this.sphere.position.set(x, y, z);
  }

  showSphere(): void {
    this.clearForceArrow();
    this.clearBalanceProjection();
    this.clearArticulated();
    this.sphere.visible = true;
  }

  /** Loads primitive shapes from the actual compiled MJCF model, not a fake pose. */
  showArticulated(specs: readonly PhysicsGeomSpec[]): void {
    this.clearForceArrow();
    this.clearBalanceProjection();
    this.clearArticulated();
    this.sphere.visible = false;
    for (const spec of specs) {
      const [sx, sy, sz] = spec.size;
      let geometry: THREE.BufferGeometry;
      switch (spec.shape) {
        case "sphere":
          geometry = new THREE.SphereGeometry(sx, 16, 12);
          break;
        case "ellipsoid":
          geometry = new THREE.SphereGeometry(1, 16, 12);
          geometry.scale(sx, sy, sz);
          break;
        case "capsule":
          geometry = new THREE.CapsuleGeometry(sx, 2 * sy, 6, 12);
          // MuJoCo's capsule axis is local Z; Three's is local Y.
          geometry.rotateX(Math.PI / 2);
          break;
        case "cylinder":
          geometry = new THREE.CylinderGeometry(sx, sx, 2 * sy, 32, 1);
          // MuJoCo cylinders use local Z, Three.js CylinderGeometry uses local Y.
          geometry.rotateX(Math.PI / 2);
          break;
        case "box":
          geometry = new THREE.BoxGeometry(2 * sx, 2 * sy, 2 * sz);
          break;
      }
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(spec.rgba[0], spec.rgba[1], spec.rgba[2]),
        roughness: 0.68,
        metalness: 0.03,
        opacity: spec.rgba[3],
        transparent: spec.rgba[3] < 1,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.name = "mujoco_geom_" + spec.id;
      this.scene.add(mesh);
      this.articulated.set(spec.id, mesh);
    }
  }

  /** Each frame comes from data.geom_xpos and data.geom_xmat in the simulator. */
  setArticulatedPoses(poses: readonly PhysicsGeomPose[]): void {
    for (const pose of poses) {
      const mesh = this.articulated.get(pose.id);
      if (!mesh) continue;
      const [x, y, z] = mujocoToThreePosition(pose.position);
      mesh.position.set(x, y, z);
      const m = pose.rotation;
      this.rotationMatrix.set(
        m[0], m[1], m[2], 0,
        m[3], m[4], m[5], 0,
        m[6], m[7], m[8], 0,
        0, 0, 0, 1,
      );
      this.rotationQuaternion.setFromRotationMatrix(this.rotationMatrix);
      mesh.quaternion.copy(MUJOCO_TO_THREE).multiply(this.rotationQuaternion);
    }
  }

  /** Display the push vector at its physical application site. The longer
   * arrow is a schematic (nonlinear length); its direction is exact.
   * Preview stays visible before the pulse; active force is colored red.
   */
  setForceArrow(
    pointWorldM: readonly [number, number, number],
    forceWorldN: readonly [number, number, number],
    active: boolean,
  ): void {
    if (![...pointWorldM, ...forceWorldN].every(Number.isFinite)) {
      throw new RangeError("Flecha de fuerza con valores no finitos");
    }
    const magnitude = Math.hypot(...forceWorldN);
    if (magnitude <= 0) {
      this.clearForceArrow();
      return;
    }
    const [px, py, pz] = mujocoToThreePosition(pointWorldM);
    const [fx, fy, fz] = mujocoToThreePosition(forceWorldN);
    this.forceArrow.position.set(px, py, pz);
    this.forceArrow.setDirection(new THREE.Vector3(fx, fy, fz).normalize());
    const length = forceArrowLengthM(magnitude);
    this.forceArrow.setLength(length, Math.min(0.18, length * 0.25), Math.min(0.09, length * 0.13));
    this.forceArrow.setColor(active ? 0xff3355 : 0xffbd59);
    this.forceArrow.visible = true;
    this.forceFlash.position.copy(this.forceArrow.position);
    this.forceFlashRing.position.copy(this.forceArrow.position);
    this.forceFlash.visible = active;
    this.forceFlashRing.visible = active;
  }

  clearForceArrow(): void {
    this.forceArrow.visible = false;
    this.forceFlash.visible = false;
    this.forceFlashRing.visible = false;
  }

  /** The marker is at the actual physical body COM, with dashed vertical
   * projection to z=0. Support contour is an approximate hull of *loaded*
   * feet, never a measured CoP or a stability certificate.
   */
  setBalanceProjection(
    comWorldM: XYZ,
    loadedSupportPolygonXY: readonly XY[],
  ): ProjectionStatus {
    const projection = projectBalanceToGround(comWorldM, loadedSupportPolygonXY);
    const [x, y, z] = mujocoToThreePosition(comWorldM);
    const groundY = 0.008;
    this.comMarker.position.set(x, y, z);
    this.comGroundMarker.position.set(x, groundY, z);
    const color = projection.status === "outside" ? 0xff5975 :
      projection.status === "inside" ? 0x5df0b3 : 0xffd27c;
    (this.comMarker.material as THREE.MeshBasicMaterial).color.setHex(color);
    (this.comGroundMarker.material as THREE.MeshBasicMaterial).color.setHex(color);
    (this.comVerticalLine.material as THREE.LineDashedMaterial).color.setHex(color);
    const drop = this.comVerticalLine.geometry.getAttribute("position") as THREE.BufferAttribute;
    drop.setXYZ(0, x, y, z);
    drop.setXYZ(1, x, groundY, z);
    drop.needsUpdate = true;
    this.comVerticalLine.computeLineDistances();
    const attr = this.supportOutline.geometry.getAttribute("position") as THREE.BufferAttribute;
    const count = Math.min(loadedSupportPolygonXY.length, 8);
    for (let i = 0; i < count; i++) {
      const [hx, hy] = loadedSupportPolygonXY[i]!;
      // MuJoCo +Y -> Three -Z
      attr.setXYZ(i, hx, groundY + 0.002, -hy);
    }
    attr.needsUpdate = true;
    this.supportOutline.geometry.setDrawRange(0, count);
    this.supportOutline.visible = count >= 3;
    this.comMarker.visible = true;
    this.comGroundMarker.visible = true;
    this.comVerticalLine.visible = true;
    return projection.status;
  }

  clearBalanceProjection(): void {
    this.comMarker.visible = false;
    this.comGroundMarker.visible = false;
    this.comVerticalLine.visible = false;
    this.supportOutline.visible = false;
  }

  private clearArticulated(): void {
    for (const mesh of this.articulated.values()) {
      this.scene.remove(mesh);
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    }
    this.articulated.clear();
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.renderer.setAnimationLoop(null);
    this.observer.disconnect();
    this.controls.dispose();
    this.clearArticulated();
    this.scene.remove(this.forceFlash, this.forceFlashRing,
      this.comMarker, this.comGroundMarker, this.comVerticalLine, this.supportOutline);
    for (const marker of [this.forceFlash, this.forceFlashRing,
      this.comMarker, this.comGroundMarker]) {
      marker.geometry.dispose();
      (marker.material as THREE.Material).dispose();
    }
    for (const line of [this.comVerticalLine, this.supportOutline]) {
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
    }
    this.forceArrow.line.geometry.dispose();
    (this.forceArrow.line.material as THREE.Material).dispose();
    this.forceArrow.cone.geometry.dispose();
    (this.forceArrow.cone.material as THREE.Material).dispose();
    this.sphere.geometry.dispose();
    (this.sphere.material as THREE.Material).dispose();
    for (const child of this.scene.children) {
      if (child instanceof THREE.Mesh && child !== this.sphere) {
        child.geometry.dispose();
        if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
        else child.material.dispose();
      }
    }
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
