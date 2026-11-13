/**
 * Utilidades puras del editor de escenarios de la web (`src/renderer/scenarioEditor.ts`, `docs/SCENARIO_EDITOR.md`):
 * fuerza en módulo y ángulos, escenario dentro de un enlace e informe de un ensayo. Sin DOM ni MuJoCo.
 */
import type { Recording } from "../recording/schema";
import type { Vec3 } from "./schema";

/**
 * Fuerza en la forma en que se edita: módulo en el plano sagital, elevación y componente lateral.
 * Elevación en grados en el plano X–Z del `frame` del efector: 0 = hacia delante (+X), 90 = arriba, −90 = abajo,
 * ±180 = hacia atrás. Con `frame: "actor_initial"`, «delante» es hacia donde mira Uke al empezar.
 */
export interface PolarForce {
  magnitude_n: number;
  elevation_deg: number;
  lateral_n: number;
}

const round = (x: number, digits: number): number => Number(x.toFixed(digits)) + 0;

export function forceToPolar(force: Vec3): PolarForce {
  const magnitude = Math.hypot(force[0], force[2]);
  return {
    magnitude_n: round(magnitude, 1),
    elevation_deg: magnitude < 1e-9 ? 0 : round((Math.atan2(force[2], force[0]) * 180) / Math.PI, 1),
    lateral_n: round(force[1], 1),
  };
}

export function polarToForce(polar: PolarForce): Vec3 {
  const a = (polar.elevation_deg * Math.PI) / 180;
  return [round(polar.magnitude_n * Math.cos(a), 1), round(polar.lateral_n, 1), round(polar.magnitude_n * Math.sin(a), 1)];
}

/** Clave del fragmento de la URL que lleva un escenario editado: `#scn=<base64url del JSON en UTF-8>`. */
export const SCENARIO_HASH_KEY = "scn";

function toBase64Url(bytes: Uint8Array): string {
  let binary = "";
  for (const b of bytes) binary += String.fromCharCode(b);
  return btoa(binary).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

function fromBase64Url(text: string): Uint8Array {
  const binary = atob(text.replace(/-/g, "+").replace(/_/g, "/"));
  return Uint8Array.from(binary, (c) => c.charCodeAt(0));
}

/** Fragmento (`#scn=...`) con el escenario dentro. El JSON se guarda compacto. */
export function encodeScenarioHash(scenario: unknown): string {
  return `#${SCENARIO_HASH_KEY}=${toBase64Url(new TextEncoder().encode(JSON.stringify(scenario)))}`;
}

/** Escenario (JSON sin validar) de un fragmento `#scn=...`, o `undefined` si no lo lleva. Lanza si está corrupto. */
export function decodeScenarioHash(hash: string): unknown {
  const value = new URLSearchParams(hash.replace(/^#/, "")).get(SCENARIO_HASH_KEY);
  if (value === null || value === "") return undefined;
  try {
    return JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(fromBase64Url(value))) as unknown;
  } catch {
    throw new Error("El escenario del enlace está dañado o incompleto");
  }
}

interface ContactSpan {
  body: string;
  from_s: number;
  to_s: number;
  max_n: number;
}

/** Tramos de contacto entre `owner` y cada otro cuerpo (se unen huecos de menos de 0,05 s). */
function contactSpans(recording: Recording, owner: string): ContactSpan[] {
  const spans: ContactSpan[] = [];
  for (const sample of recording.samples) {
    for (const c of sample.contacts) {
      if (c.body_a !== owner && c.body_b !== owner) continue;
      const body = c.body_a === owner ? c.body_b : c.body_a;
      const force = Math.hypot(c.force_n[0], c.force_n[1], c.force_n[2]);
      let last: ContactSpan | undefined;
      for (let i = spans.length - 1; i >= 0 && !last; i--) if (spans[i]!.body === body) last = spans[i];
      if (last && sample.t - last.to_s <= 0.05) {
        last.to_s = sample.t;
        last.max_n = Math.max(last.max_n, force);
      } else spans.push({ body, from_s: sample.t, to_s: sample.t, max_n: force });
    }
  }
  return spans;
}

const kN = (n: number): string => `${(n / 1000).toFixed(1).replace(".", ",")} kN`;
const sec = (t: number): string => `${t.toFixed(2).replace(".", ",")} s`;

/**
 * Informe en texto de un ensayo, una línea por dato: resultado, primer contacto con el suelo, golpe en la cabeza, qué toca
 * cada obstáculo, picos de las manos, presupuesto de Tori y artefactos. Es lo que el editor enseña tras simular.
 */
export function describeTrial(recording: Recording): string[] {
  const s = recording.summary;
  const lines: string[] = [];
  const fall = s.uke?.fall;
  lines.push(
    `Resultado: ${s.outcome}` + (fall ? ` a los ${sec(fall.t_s)}, primera parte en el suelo: ${fall.first_part ?? "-"}` : "") +
      `; inclinación máxima ${s.max_tilt_deg.toFixed(0)}°` + (s.expect === "none" ? "" : `; expect ${s.expect}`),
  );

  // Golpes contra el suelo de las partes que no son pies.
  const ground = contactSpans(recording, "world").filter((x) => !x.body.includes("foot"));
  const firstByBody = new Map<string, ContactSpan>();
  for (const g of ground) {
    const known = firstByBody.get(g.body);
    if (!known) firstByBody.set(g.body, { ...g });
    else known.max_n = Math.max(known.max_n, g.max_n);
  }
  const order = [...firstByBody.values()].filter((g) => !recording.peaks.obstacles?.[g.body]).sort((a, b) => a.from_s - b.from_s);
  if (order.length > 0) {
    lines.push("Suelo, por orden de llegada: " + order.slice(0, 6).map((g) => `${g.body} a ${sec(g.from_s)} (${kN(g.max_n)})`).join(", "));
    const head = firstByBody.get("head");
    // Los contactos se graban a `record_hz`; el pico real (a 500 Hz) puede ser mayor y, si pasa de 10 kN, sale en «Artefactos».
    lines.push(head ? `Cabeza: golpea el suelo a ${sec(head.from_s)}, al menos ${kN(head.max_n)}` : "Cabeza: no toca el suelo");
  }

  for (const [id, o] of Object.entries(recording.peaks.obstacles ?? {})) {
    const touches = contactSpans(recording, id).filter((x) => x.body !== "world" && x.to_s - x.from_s >= 0.02);
    lines.push(
      `Obstáculo ${id}: ` + (touches.length === 0 ? "no llega a cargar a Uke" :
        "toca " + touches.map((x) => `${x.body} de ${sec(x.from_s)} a ${sec(x.to_s)} (${x.max_n.toFixed(0)} N)`).join(", ")) +
        `; cede ${(o.max_deviation_m * 100).toFixed(1)} cm; saturado ${sec(o.time_saturated_s)}`,
    );
  }

  const body = recording.peaks.tori_body;
  if (body) {
    const loaded = Object.entries(body.segments).filter(([, x]) => x.contact_time_s >= 0.02)
      .sort((x, y) => y[1].max_contact_force_on_actor_n - x[1].max_contact_force_on_actor_n);
    lines.push(
      "Cuerpo de Tori: " + (loaded.length === 0 ? "no llega a cargar a Uke" :
        "cargan " + loaded.map(([name, x]) => `${name} ${sec(x.contact_time_s)} (${x.max_contact_force_on_actor_n.toFixed(0)} N)`).join(", ")) +
        `; contacto total máximo ${body.max_contact_force_on_actor_n.toFixed(0)} N`,
    );
    const yielded = Object.entries(body.segments).filter(([, x]) => x.max_deviation_m >= 0.05);
    if (yielded.length > 0) lines.push("  Ceden: " + yielded.map(([name, x]) => `${name} ${(x.max_deviation_m * 100).toFixed(0)} cm`).join(", "));
  }

  for (const [id, e] of Object.entries(recording.peaks.effectors)) {
    let extra = "";
    if (e.hand) extra = `; en el tope ${sec(e.hand.time_at_cap_s)}; ${e.hand.events.map((x) => `${x.kind} a ${sec(x.t_s)}`).join(", ")}`;
    if (e.pull) extra = `; potencia máxima ${e.pull.max_power_w.toFixed(0)} W, recortada ${sec(e.pull.time_power_limited_s)}`;
    lines.push(`Fuerza ${id}: máximo ${e.max_force_n.toFixed(0)} N` + (e.limit_n === undefined ? "" : ` (tope ${e.limit_n.toFixed(0)} N)`) + extra);
  }

  const b = s.tori_budget;
  if (b) {
    lines.push(
      `Tori (${b.mass_kg} kg): ${b.feasible ? "viable" : "INVIABLE"}; usa el ${(100 * b.friction_use.value).toFixed(0)} % del rozamiento de sus pies; ` +
        `carga en los pies de ${b.min_feet_load_n.value.toFixed(0)} a ${b.max_feet_load_n.value.toFixed(0)} N; tiro total máximo ${b.max_total_pull_n.value.toFixed(0)} N`,
    );
    for (const v of b.violations) {
      lines.push(`  Tori incumple ${v.kind} desde ${sec(v.t_s)} durante ${sec(v.duration_s)} (medido ${v.value.toFixed(0)}, tope ${v.limit.toFixed(0)})`);
    }
  }

  const artifacts = s.artifacts ?? [];
  lines.push(artifacts.length === 0 ? "Artefactos: ninguno" : "Artefactos: " + artifacts.map((a) => a.detail).join("; "));
  return lines;
}
