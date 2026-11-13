/**
 * Construye el MJCF de un escenario `hpl.scenario.v1`. Modelos NUEVOS: usan `worldOptionTag()`
 * (cono elíptico, ver `docs/CONTACT_CALIBRATION.md`) y los parámetros de contacto de `humanoid_step_v2.xml`.
 * No depende de `src/physics/`.
 */
import { appendToWorldbody, buildObstacleMjcf, resolveObstacleParams, type ObstacleParams } from "../effectors/obstacle";
import { buildToriBodyMjcf, resolveToriBodyParams, toriBodyNames, type ToriBodyParams } from "../effectors/toriBody";
import { generateHumanoidMJCF } from "../uke/generate";
import { attitudeJointDamping } from "../uke/damping";
import { ACTOR_BODIES, type HumanoidActor, type Scenario } from "./schema";
import { obstacleConfig } from "./validate";
import { worldOptionTag } from "./worldOptions";

/** Contacto de las geometrías (los de `humanoid_step_v2.xml`). */
const GEOM_CONTACT = `solref="0.015 1" solimp="0.9 0.95 0.001"`;
/** Rozamiento torsional y de rodadura de todas las geometrías. */
const TORSIONAL = 0.005;
const ROLLING = 0.0001;
/**
 * Precisión de la colisión convexa (caja-cilindro). Con los valores por defecto (`ccd_tolerance` 1e-6) el resultado
 * deja de ser simétrico respecto a un reflejo (error 1e-4 en qpos tras 1 s de contacto con un obstáculo);
 * con estos valores baja de 1e-8. Medido en `tests/unit/scenarioRun.test.ts`.
 */
const COLLISION_ATTRS = `ccd_tolerance="1e-14" ccd_iterations="100"`;
/** Holgura inicial del cilindro sobre el suelo, en m (cae 1 mm en el asentamiento). */
const SPAWN_CLEARANCE_M = 0.001;

export interface ScenarioModel {
  mjcf: string;
  /** Nombres de los cuerpos en el orden de índices de MuJoCo (`world` primero). */
  bodies: string[];
  /** Nombres de las articulaciones en el orden de MuJoCo. */
  joints: string[];
  /** Cuerpo principal del actor (el que se mide para el veredicto). */
  actorBody: string;
  /** Presente si el actor es un humanoide (ver `src/uke/`). */
  humanoid?: HumanoidActor;
  /** Obstáculos del escenario, resueltos, en el orden de `scenario.obstacles`. Sus cuerpos van al final del modelo. */
  obstacles: ObstacleParams[];
  /** Cuerpo de Tori (si el escenario lo trae y está activo). Sus cuerpos van al final del modelo, tras los obstáculos. */
  toriBody?: ToriBodyParams;
}

/** Parámetros del cuerpo de Tori del escenario, o `undefined` si no hay o está desactivado. */
export function scenarioToriBody(scenario: Scenario): ToriBodyParams | undefined {
  const body = scenario.tori?.body;
  if (!body || body.enabled === false) return undefined;
  return resolveToriBodyParams({
    mass_kg: scenario.tori!.mass_kg, height_m: body.height_m, keys: body.keys,
    ...(body.friction !== undefined ? { friction: body.friction } : {}),
  });
}

/**
 * Añade los obstáculos al final de `<worldbody>` (después del actor: la pelvis debe seguir siendo el cuerpo 1 y la raíz
 * libre los seis primeros grados de libertad; ver `appendToWorldbody`).
 */
function withObstacles(scenario: Scenario, model: Omit<ScenarioModel, "obstacles">): ScenarioModel {
  const obstacles = (scenario.obstacles ?? []).map((spec) => resolveObstacleParams(obstacleConfig(spec)));
  let mjcf = model.mjcf;
  const taken = new Set<string>(["floor", ...model.bodies]);
  for (const o of obstacles) {
    if (taken.has(o.name) || taken.has(`${o.name}_geom`)) {
      throw new Error(`obstacles[].id "${o.name}" colisiona con un nombre reservado del modelo`);
    }
    taken.add(o.name);
    mjcf = appendToWorldbody(mjcf, buildObstacleMjcf(o));
  }
  const toriBody = scenarioToriBody(scenario);
  const toriNames = toriBody ? toriBodyNames(toriBody) : { bodies: [], joints: [] };
  if (toriBody) {
    for (const name of toriNames.bodies) {
      if (taken.has(name)) throw new Error(`"${name}" está reservado para el cuerpo de Tori`);
    }
    mjcf = appendToWorldbody(mjcf, buildToriBodyMjcf(toriBody));
  }
  return {
    ...model,
    mjcf,
    bodies: [...model.bodies, ...obstacles.map((o) => o.name), ...toriNames.bodies],
    joints: [...model.joints, ...obstacles.flatMap((o) => [`${o.name}_x`, `${o.name}_y`, `${o.name}_z`]), ...toriNames.joints],
    obstacles,
    ...(toriBody ? { toriBody } : {}),
  };
}

function num(x: number): string {
  return String(x);
}

/** Real oriented capsule, no special solver behaviour: ordinary static MuJoCo contact. */
function propGeometryXml(prop: Scenario["props"][number], friction:string):string {
  if(prop.kind==="fixed_bar"){
    const [ax,ay,az]=prop.from_m,[bx,by,bz]=prop.to_m;
    return `    <body name="${prop.id}">
      <geom name="${prop.id}_geom" type="capsule"
        fromto="${num(ax)} ${num(ay)} ${num(az)} ${num(bx)} ${num(by)} ${num(bz)}"
        size="${num(prop.radius_m)}" rgba="0.38 0.82 0.64 1"
        friction="${friction}" ${GEOM_CONTACT}/>
    </body>`;
  }
  const [sx,sy,sz]=prop.size_m,[px,py,pz]=prop.pos_m;
  return `    <body name="${prop.id}" pos="${num(px)} ${num(py)} ${num(pz)}">
      <geom name="${prop.id}_geom" type="box" size="${num(sx/2)} ${num(sy/2)} ${num(sz/2)}"
        rgba="0.68 0.43 0.24 1" friction="${friction}" ${GEOM_CONTACT}/>
    </body>`;
}

/**
 * MJCF del escenario. En esta fase solo se admite un actor (`rigid_cylinder`, cuerpo `cylinder`);
 * los props son cajas fijas, cada una un cuerpo estático llamado como su `id`.
 */
export function buildScenarioModel(scenario: Scenario): ScenarioModel {
  if (scenario.actors.length !== 1) {
    throw new Error(`Esta fase admite exactamente un actor, no ${scenario.actors.length}`);
  }
  const actor = scenario.actors[0]!;
  const actorBody = ACTOR_BODIES[actor.kind][0]!;
  const friction = `${num(scenario.ground.friction)} ${TORSIONAL} ${ROLLING}`;
  if (actor.kind === "humanoid") return withObstacles(scenario, buildHumanoidModel(scenario, actor, friction));

  const bodies = ["world"];
  const joints: string[] = [];
  const taken = new Set<string>(["world", "floor", actorBody]);

  const propXml = scenario.props.map((prop) => {
    if (taken.has(prop.id)) throw new Error(`props[].id "${prop.id}" colisiona con un nombre reservado del modelo`);
    taken.add(prop.id);
    return { id: prop.id, prop };
  });

  const propBlocks = propXml.map(({ id, prop }) => {
    bodies.push(id);
    return propGeometryXml(prop,friction);
  });

  bodies.push(actorBody);
  const jointName = `${actorBody}_free`;
  joints.push(jointName);

  const mjcf = `<mujoco model="${scenario.id}">
  <compiler angle="radian"/>
  ${worldOptionTag(COLLISION_ATTRS)}
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.23 0.28 0.32 1"
          friction="${friction}" ${GEOM_CONTACT}/>
${propBlocks.join("\n")}
    <body name="${actorBody}" pos="0 0 ${num(actor.height_m / 2 + SPAWN_CLEARANCE_M)}">
      <freejoint name="${jointName}"/>
      <geom name="${actorBody}_geom" type="cylinder" size="${num(actor.radius_m)} ${num(actor.height_m / 2)}"
            mass="${num(actor.mass_kg)}" rgba="0.74 0.83 0.88 1"
            friction="${friction}" ${GEOM_CONTACT}/>
    </body>
  </worldbody>
</mujoco>
`;
  return withObstacles(scenario, { mjcf, bodies, joints, actorBody });
}

/** Nombres en el orden de aparición en el MJCF (el de MuJoCo): cuerpos tras `world` y articulaciones. */
function namesInOrder(mjcf: string, tags: readonly string[]): string[] {
  const re = new RegExp(`<(?:${tags.join("|")})\\s+name="([^"]+)"`, "g");
  return Array.from(mjcf.matchAll(re), (m) => m[1]!);
}

/** Actor humanoide: el modelo lo genera `src/uke/generate.ts`; los props se insertan tras el suelo. */
function buildHumanoidModel(scenario: Scenario, actor: HumanoidActor, friction: string): Omit<ScenarioModel, "obstacles"> {
  const reserved = new Set<string>(["world", "floor", ...ACTOR_BODIES.humanoid]);
  const propBlocks = scenario.props.map((prop) => {
    if (reserved.has(prop.id) || reserved.has(`${prop.id}_geom`)) {
      throw new Error(`props[].id "${prop.id}" colisiona con un nombre reservado del modelo`);
    }
    reserved.add(prop.id);
    return propGeometryXml(prop,friction);
  });
  const mjcf = generateHumanoidMJCF({
    mass_kg: actor.mass_kg,
    height_m: actor.height_m,
    ...(actor.build !== undefined ? { build: actor.build } : {}),
    ...(actor.segment_lengths_m !== undefined ? { segment_lengths_m: actor.segment_lengths_m } : {}),
  }, {
    friction: scenario.ground.friction,
    extraWorldXml: propBlocks.join("\n"),
    optionAttrs: COLLISION_ATTRS,
    modelName: scenario.id,
    jointDampingByName: attitudeJointDamping(actor.attitude, actor.mass_kg, actor.height_m, actor.strength),
  });
  return {
    mjcf,
    bodies: ["world", ...namesInOrder(mjcf, ["body"])],
    joints: namesInOrder(mjcf, ["joint", "freejoint"]),
    actorBody: ACTOR_BODIES.humanoid[0]!,
    humanoid: actor,
  };
}
