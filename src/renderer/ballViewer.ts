import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { mujocoToThreePosition } from "./coordinates";

/** Lightweight Three.js view of the *real* MuJoCo gravity test state.
 * The model contains exactly one dynamic sphere; future humanoid visualization
 * must render articulated geoms from MjData.xpos/xquat, not this placeholder.
 */
export class BallViewer {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly sphere: THREE.Mesh;
  private readonly observer: ResizeObserver;
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
