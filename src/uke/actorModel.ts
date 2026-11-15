/** Compose real articulated humanoids in one MuJoCo world; no mocap bodies or world actuators. */
import type { HumanoidActor } from "../scenario/schema";
import { generateHumanoidMJCF } from "./generate";
import { attitudeJointDamping } from "./damping";
import { extractBodyTree } from "./modelNamespace";
export interface HumanoidPlacement {
  readonly xy_m: readonly [
    number,
    number
  ];
  readonly heading_deg: number;
}
export function appendArticulatedHumanoid(world: string, actor: HumanoidActor, prefix: string, placement: HumanoidPlacement, friction: number): string {
  if (!prefix)
    throw new RangeError("El segundo humanoide necesita un prefijo");
  if (![...placement.xy_m, placement.heading_deg, friction].every(Number.isFinite) || friction < 0)
    throw new RangeError("Colocación o fricción inválida");
  const generated = generateHumanoidMJCF(actor, {
    namePrefix: prefix, friction,
    jointDampingByName: attitudeJointDamping(actor.attitude, actor.mass_kg, actor.height_m, actor.strength),
    ...(actor.joint_limit_timeconst_s !== undefined ? { jointLimitTimeconstS: actor.joint_limit_timeconst_s } : {}),
  });
  let body = extractBodyTree(generated, prefix + "pelvis").xml;
  const existing = new Set(Array.from(world.matchAll(/\bname="([^"]+)"/g), m => m[1]));
  for (const m of body.matchAll(/\bname="([^"]+)"/g))
    if (existing.has(m[1]))
      throw new RangeError(`Nombre duplicado: ${m[1]}`);
  const yaw = (placement.heading_deg % 360) * Math.PI / 180;
  body = body.replace(/^(<body\b[^>]*\bpos=")([^"]+)"/, (_all, start: string, xyz: string) => `${start}${placement.xy_m[0]} ${placement.xy_m[1]} ${xyz.split(/\s+/)[2]}" quat="${Math.cos(yaw / 2)} 0 0 ${Math.sin(yaw / 2)}"`);
  // Explicit per-actor defaults prevent the first actor's size/attitude leaking into the second.
  body = body.replace(/<joint /g, `<joint limited="true" armature="0.015" solreflimit="${actor.joint_limit_timeconst_s ?? 0.02} 1" `)
    .replace(/<geom /g, `<geom contype="1" conaffinity="1" friction="${friction} 0.025 0.002" condim="3" solref="0.015 1" solimp="0.9 0.95 0.001" `)
    .replace(/(<geom\b[^>]*\brgba=)"[^"]+"/g, '$1"0.95 0.93 0.86 1"');
  const contact = /<contact>([\s\S]*?)<\/contact>/.exec(generated)?.[1] ?? "";
  if (!world.includes("</worldbody>") || !world.includes("</contact>"))
    throw new Error("El mundo no contiene las secciones del generador");
  return world.replace("</worldbody>", body + "\n</worldbody>").replace("</contact>", contact + "\n</contact>");
}
