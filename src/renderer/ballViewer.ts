import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { PhysicsGeomPose, PhysicsGeomSpec } from "../physics/geometry";
import { mujocoToThreePosition } from "./coordinates";

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
    this.setSpherePosition([0, 0, 1.5]);

    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(this.container);
    this.resize();
    this.renderer.setAnimationLoop(() => {
      if (this.disposed) return;
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  setSpherePosition(xyz: readonly [number, number, number]): void {
    const [x, y, z] = mujocoToThreePosition(xyz);
    this.sphere.position.set(x, y, z);
  }

  showSphere(): void {
    this.clearArticulated();
    this.sphere.visible = true;
  }

  /** Loads primitive shapes from the actual compiled MJCF model, not a fake pose. */
  showArticulated(specs: readonly PhysicsGeomSpec[]): void {
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
