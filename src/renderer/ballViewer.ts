import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { PhysicsGeomPose, PhysicsGeomSpec } from "../physics/geometry";
import { mujocoToThreePosition } from "./coordinates";
import { forceArrowLengthM } from "./forceArrow";
import { classifyForceGlyph } from "./forceGlyph";
import type { ForceVector } from "../physics/forceSequence";
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
  private readonly multiForceArrows = new Map<string, THREE.ArrowHelper>();
  private readonly editableHeads = new Map<string, THREE.Mesh>();
  private readonly multiForceHalos = new Map<string, THREE.Mesh>();
  private readonly glyphState = new Map<string, string>();
  private selectedForceId: string | undefined;
  private pickBodyForForceId: string | undefined;
  private oncePick: ((geomId: number, point: ForceVector) => void) | undefined;
  private forceInteraction?: {
    onPicked(id: string, geomId: number, point: ForceVector): void;
    onDragged(id: string, direction: ForceVector): void;
  };
  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private draggingForceId: string | undefined;
  private readonly dragPlane = new THREE.Plane();
  private readonly dragOrigin = new THREE.Vector3();
  private readonly multiForceGlyphs = new Map<string, { sprite: THREE.Sprite; texture: THREE.CanvasTexture }>();
  private multiForces: readonly { id: string; point: ForceVector; force: ForceVector; active: boolean }[] = [];
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
    this.raycaster.params.Line = { threshold: 0.12 };
    this.renderer.domElement.addEventListener("pointerdown", this.handleForcePointerDown);
    this.renderer.domElement.addEventListener("pointermove", this.handleForcePointerMove);
    this.renderer.domElement.addEventListener("pointerup", this.handleForcePointerUp);
    this.renderer.domElement.addEventListener("pointercancel", this.handleForcePointerUp);
    this.renderer.setAnimationLoop(() => {
      if (this.disposed) return;
      this.controls.update();
      if (this.forceFlashRing.visible) this.forceFlashRing.quaternion.copy(this.camera.quaternion);
      if (this.multiForces.length) this.refreshMultiForceVisuals();
      this.renderer.render(this.scene, this.camera);
    });
  }

  setSpherePosition(xyz: readonly [number, number, number]): void {
    const [x, y, z] = mujocoToThreePosition(xyz);
    this.sphere.position.set(x, y, z);
  }

  showSphere(): void {
    this.clearForceArrow();
    this.clearMultiForces();
    this.clearBalanceProjection();
    this.clearArticulated();
    this.sphere.visible = true;
  }

  /** Loads primitive shapes from the actual compiled MJCF model, not a fake pose. */
  showArticulated(specs: readonly PhysicsGeomSpec[]): void {
    this.clearForceArrow();
    this.clearMultiForces();
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


  /** Render view-relative ⊗ (away) and ⊙ (toward camera), or an ordinary arrow.
   * This is presentation only: MuJoCo forces remain in world coordinates.
   */
  setMultiForces(forces: readonly { id: string; point: ForceVector; force: ForceVector; active: boolean }[]): void {
    this.multiForces = forces.map(f => ({ ...f }));
    const visible = new Set(forces.map(f => f.id));
    for (const [id, arrow] of this.multiForceArrows) {
      if (visible.has(id)) continue;
      this.scene.remove(arrow);
      const halo = this.multiForceHalos.get(id);
      if (halo) {
        this.scene.remove(halo);
        halo.geometry.dispose();
        (halo.material as THREE.Material).dispose();
        this.multiForceHalos.delete(id);
      }
      this.glyphState.delete(id);
      const head = this.editableHeads.get(id);
      if (head) {
        this.scene.remove(head);
        head.geometry.dispose();
        (head.material as THREE.Material).dispose();
        this.editableHeads.delete(id);
      }
      arrow.line.geometry.dispose();
      (arrow.line.material as THREE.Material).dispose();
      arrow.cone.geometry.dispose();
      (arrow.cone.material as THREE.Material).dispose();
      this.multiForceArrows.delete(id);
    }
    for (const [id, glyph] of this.multiForceGlyphs) {
      if (visible.has(id)) continue;
      this.scene.remove(glyph.sprite);
      glyph.texture.dispose();
      glyph.sprite.material.dispose();
      this.multiForceGlyphs.delete(id);
    }
    for (const f of forces) {
      if (this.multiForceArrows.has(f.id)) continue;
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(), 0.3, 0xffbd59);
      const canvas = document.createElement("canvas");
      canvas.width = canvas.height = 128;
      const texture = new THREE.CanvasTexture(canvas);
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
      sprite.scale.set(0.3, 0.3, 1);
      const head = new THREE.Mesh(new THREE.SphereGeometry(0.075, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0xf9f1ad, transparent: true, opacity: 0.85, depthTest: false }));
      head.visible = false;
      const halo = new THREE.Mesh(new THREE.SphereGeometry(0.13, 16, 12),
        new THREE.MeshBasicMaterial({
          color: 0xff365a, transparent: true, opacity: 0.48,
          depthWrite: false, depthTest: false, blending: THREE.AdditiveBlending,
        }));
      halo.visible = false;
      this.scene.add(arrow, sprite, head, halo);
      this.multiForceHalos.set(f.id, halo);
      this.editableHeads.set(f.id, head);
      this.multiForceArrows.set(f.id, arrow);
      this.multiForceGlyphs.set(f.id, { sprite, texture });
    }
    this.refreshMultiForceVisuals();
  }

  private refreshMultiForceVisuals(): void {
    for (const f of this.multiForces) {
      const arrow = this.multiForceArrows.get(f.id);
      const marker = this.multiForceGlyphs.get(f.id);
      if (!arrow || !marker) continue;
      const world = new THREE.Vector3(...mujocoToThreePosition(f.point));
      const vec = new THREE.Vector3(...mujocoToThreePosition(f.force));
      const camera = this.camera.position.clone().sub(world).normalize();
      const glyph = classifyForceGlyph(f.force, [camera.x, -camera.z, camera.y]);
      arrow.position.copy(world);
      arrow.visible = glyph === "arrow";
      const halo = this.multiForceHalos.get(f.id);
      if (halo) {
        halo.position.copy(world);
        halo.visible = f.active;
      }
      const head = this.editableHeads.get(f.id);
      if (head) {
        head.visible = f.id === this.selectedForceId && !!this.forceInteraction && vec.length() > 0;
        // Visible even for ⊗/⊙: allows direct dragging of a view-normal force.
        head.position.copy(world).add(vec.clone().normalize().multiplyScalar(forceArrowLengthM(vec.length())));
      }
      if (glyph === "arrow") {
        arrow.setDirection(vec.normalize());
        const size = forceArrowLengthM(vec.length());
        arrow.setLength(size, Math.min(0.18, size*0.25), Math.min(0.09, size*0.13));
        arrow.setColor(f.active ? 0xff3355 : 0xffbd59);
      }
      marker.sprite.position.copy(world);
      marker.sprite.visible = glyph === "into-screen" || glyph === "out-of-screen";
      if (marker.sprite.visible) {
        const state = glyph + ":" + (f.active ? "active" : "reference");
        if (this.glyphState.get(f.id) === state) continue;
        this.glyphState.set(f.id, state);
        const ctx = (marker.texture.image as HTMLCanvasElement).getContext("2d");
        if (ctx) {
          ctx.clearRect(0,0,128,128);
          ctx.strokeStyle = f.active ? "#ff3355" : "#ffbd59";
          ctx.fillStyle = ctx.strokeStyle;
          ctx.lineWidth = 10;
          ctx.beginPath(); ctx.arc(64,64,45,0,Math.PI*2); ctx.stroke();
          if (glyph === "into-screen") {
            ctx.beginPath(); ctx.moveTo(38,38); ctx.lineTo(90,90);
            ctx.moveTo(90,38); ctx.lineTo(38,90); ctx.stroke();
          } else {
            ctx.beginPath(); ctx.arc(64,64,12,0,Math.PI*2); ctx.fill();
          }
          marker.texture.needsUpdate = true;
        }
      }
    }
  }


  /** The editor configures interactions. Geometry picks are converted to
   * physical body-local anchors by MujocoSimulation, never by Three.js.
   */
  configureForceEditing(
    selectedId: string | undefined,
    onPicked: ((id: string, geomId: number, point: ForceVector) => void) | undefined,
    onDragged: ((id: string, direction: ForceVector) => void) | undefined,
  ): void {
    this.selectedForceId = selectedId;
    this.forceInteraction = onPicked && onDragged ? { onPicked, onDragged } : undefined;
    if (!this.forceInteraction) {
      this.pickBodyForForceId = undefined;
      this.renderer.domElement.style.cursor = "";
    }
  }

  /**
   * Espera un clic sobre una geometría articulada y devuelve su id y el punto (coordenadas de MuJoCo). Un solo uso:
   * `undefined` lo cancela. Lo usa el editor de escenarios para elegir dónde se aplica una fuerza.
   */
  pickArticulatedOnce(onPicked: ((geomId: number, point: ForceVector) => void) | undefined): void {
    this.oncePick = onPicked;
    this.renderer.domElement.style.cursor = onPicked ? "crosshair" : "";
  }

  armForcePointPick(id: string): void {
    if (!this.forceInteraction) return;
    this.selectedForceId = id;
    this.pickBodyForForceId = id;
    this.renderer.domElement.style.cursor = "crosshair";
  }

  private updateForceRay(event: PointerEvent): void {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.set(
      ((event.clientX - rect.left) / Math.max(rect.width, 1)) * 2 - 1,
      -((event.clientY - rect.top) / Math.max(rect.height, 1)) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.pointer, this.camera);
  }

  private readonly handleForcePointerDown = (event: PointerEvent): void => {
    if (this.oncePick && event.button === 0) {
      this.updateForceRay(event);
      const hit = this.raycaster.intersectObjects(Array.from(this.articulated.values()), false)[0];
      const geomId = hit ? Number(hit.object.name.replace("mujoco_geom_", "")) : NaN;
      if (!hit || !Number.isSafeInteger(geomId)) return;
      const done = this.oncePick;
      this.pickArticulatedOnce(undefined);
      done(geomId, [hit.point.x, -hit.point.z, hit.point.y]);
      event.preventDefault();
      event.stopImmediatePropagation();
      return;
    }
    if (!this.forceInteraction || event.button !== 0) return;
    this.updateForceRay(event);
    if (this.pickBodyForForceId) {
      const meshes = Array.from(this.articulated.values());
      const picked = this.raycaster.intersectObjects(meshes, false)[0];
      if (!picked) return;
      const geomId = Number(picked.object.name.replace("mujoco_geom_", ""));
      if (!Number.isSafeInteger(geomId)) return;
      const target = this.pickBodyForForceId;
      this.pickBodyForForceId = undefined;
      this.renderer.domElement.style.cursor = "";
      // The picked point belongs to a *physical* dynamic body, validated in the callback.
      this.forceInteraction.onPicked(target, geomId,
        [picked.point.x, -picked.point.z, picked.point.y]);
      event.preventDefault();
      event.stopImmediatePropagation();
      return;
    }
    const selected = this.selectedForceId;
    if (!selected) return;
    const head = this.editableHeads.get(selected);
    const arrow = this.multiForceArrows.get(selected);
    if (!head || !arrow) return;
    const hits = this.raycaster.intersectObjects([head, arrow], true);
    if (!hits.length) return;
    const spec = this.multiForces.find(f => f.id === selected);
    if (!spec) return;
    this.dragOrigin.set(...mujocoToThreePosition(spec.point));
    this.dragPlane.setFromNormalAndCoplanarPoint(
      this.camera.getWorldDirection(new THREE.Vector3()), head.position,
    );
    this.draggingForceId = selected;
    this.controls.enabled = false;
    this.renderer.domElement.setPointerCapture(event.pointerId);
    event.preventDefault();
    event.stopImmediatePropagation();
  };

  private readonly handleForcePointerMove = (event: PointerEvent): void => {
    if (!this.draggingForceId || !this.forceInteraction) return;
    this.updateForceRay(event);
    const point = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.dragPlane, point)) return;
    const vec = point.sub(this.dragOrigin);
    if (vec.lengthSq() < 0.001) return;
    vec.normalize();
    // Preserve the edited magnitude: dragging the end changes direction, not newtons.
    this.forceInteraction.onDragged(this.draggingForceId, [vec.x, -vec.z, vec.y]);
    event.preventDefault();
  };

  private readonly handleForcePointerUp = (event: PointerEvent): void => {
    if (!this.draggingForceId) return;
    this.draggingForceId = undefined;
    this.controls.enabled = true;
    if (this.renderer.domElement.hasPointerCapture(event.pointerId)) {
      this.renderer.domElement.releasePointerCapture(event.pointerId);
    }
  };

  clearMultiForces(): void { this.setMultiForces([]); }

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
    this.renderer.domElement.removeEventListener("pointerdown", this.handleForcePointerDown);
    this.renderer.domElement.removeEventListener("pointermove", this.handleForcePointerMove);
    this.renderer.domElement.removeEventListener("pointerup", this.handleForcePointerUp);
    this.renderer.domElement.removeEventListener("pointercancel", this.handleForcePointerUp);
    this.controls.dispose();
    this.clearArticulated();
    this.clearMultiForces();
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
