/**
 * Construye el MJCF de un escenario `hpl.scenario.v1`. Modelos NUEVOS: usan `worldOptionTag()`
 * (cono elíptico, ver `docs/CONTACT_CALIBRATION.md`) y los parámetros de contacto de `humanoid_step_v2.xml`.
 * No depende de `src/physics/`.
 */
import { ACTOR_BODIES, type Scenario } from "./schema";
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
}

function num(x: number): string {
  return String(x);
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
    const [sx, sy, sz] = prop.size_m;
    const [px, py, pz] = prop.pos_m;
    return `    <body name="${id}" pos="${num(px)} ${num(py)} ${num(pz)}">
      <geom name="${id}_geom" type="box" size="${num(sx / 2)} ${num(sy / 2)} ${num(sz / 2)}"
            rgba="0.68 0.43 0.24 1" friction="${friction}" ${GEOM_CONTACT}/>
    </body>`;
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
  return { mjcf, bodies, joints, actorBody };
}
