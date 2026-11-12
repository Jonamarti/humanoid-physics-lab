/**
 * Detector de artefactos (sección 12.5 del plan, issue #75 PR 5). Función pura sobre una grabación ya hecha: no simula ni
 * necesita MuJoCo. Lo que exige datos por paso físico (500 Hz) lo mide el ejecutor (`src/scenario/artifactProbe.ts`) y lo deja
 * en `recording.diagnostics`; los umbrales viven aquí, no en la grabación, para poder reevaluar sin volver a simular.
 *
 * Reglas:
 * - Los umbrales numéricos de la sección 12.5 del plan no se relajan para que algo pase.
 * - Un detector que no puede evaluarse (faltan datos o no aplica al actor) se informa en `not_evaluated`, nunca como limpio.
 * - Lo que el plan no numera (amplitud mínima del temblor, tolerancia de energía, qué es «cargado») es una elección de esta PR,
 *   documentada y medida en `docs/ARTIFACTS.md`.
 */
import type {
  ArtifactFinding, ArtifactNotEvaluated, ArtifactType, PeakAt, Recording,
} from "../recording/schema";

/** Un cuerpo del actor penetra el suelo más de esto (m). Plan 12.5. */
export const GROUND_PENETRATION_LIMIT_M = 0.01;
/** Dos segmentos del actor se penetran más de esto (m). Plan 12.5. */
export const SEGMENT_PENETRATION_LIMIT_M = 0.02;
/** Velocidad angular articular máxima (rad/s). Plan 12.5. */
export const JOINT_SPEED_LIMIT_RAD_S = 25;
/** Fuerza normal de contacto máxima (N). Plan 12.5. */
export const CONTACT_FORCE_LIMIT_N = 10_000;
/** Deslizamiento neto máximo de un pie cargado (m). Plan 12.5. */
export const FOOT_SLIP_LIMIT_M = 0.02;
/** Cambios de sentido de una articulación por segundo; se supera con MÁS de este número. Plan 12.5. */
export const TREMOR_REVERSALS_PER_S_LIMIT = 10;
/** Ventana (s) en que se cuentan los cambios de sentido del temblor. */
export const TREMOR_WINDOW_S = 1;
/**
 * Amplitud mínima (rad) de un recorrido para que cuente como cambio de sentido (histéresis). Elección de esta PR: el plan no la
 * fija. Por debajo es ruido numérico. Sensibilidad medida en `docs/ARTIFACTS.md`.
 */
export const TREMOR_MIN_AMPLITUDE_RAD = 0.005;
/** Un pie está «cargado» si la suma de las fuerzas normales de sus contactos con el suelo supera esta fracción del peso del actor. */
export const FOOT_LOADED_WEIGHT_FRACTION = 0.2;
/**
 * El rozamiento de un contacto cuenta como «saturado» (el deslizamiento lo provoca la fuerza, no es un artefacto) si su
 * utilización (|F_t| / (μ·F_n), 1 = límite del cono) supera esto.
 */
export const FRICTION_SATURATED_UTILIZATION = 0.9;
/**
 * Tolerancia del balance de energía: fracción de la energía potencial inicial del centro de masas (`m·g·z_cm`) que puede
 * superar la energía mecánica a la aportada. Elección de esta PR (misma regla del 1 % que la prueba de energía de `relaxed`);
 * cubre la energía elástica de los contactos blandos al liberarse y el error de integración de un paso de 2 ms.
 */
export const ENERGY_GAIN_TOLERANCE_FRACTION = 0.01;
/** Fuerza generalizada tolerada en los 6 grados de la raíz (debe ser exactamente 0). */
export const ROOT_FORCE_LIMIT = 1e-9;

export interface ArtifactReport {
  /** Artefactos detectados, por instante; vacío = limpio (salvo lo que figure en `not_evaluated`). */
  artifacts: ArtifactFinding[];
  /** Detectores que no pudieron evaluarse y por qué. */
  not_evaluated: ArtifactNotEvaluated[];
}

function fromPeak(type: ArtifactType, p: PeakAt, threshold: number, detail: string): ArtifactFinding {
  return { type, t_s: p.t_s, where: p.where, value: p.value, threshold, detail };
}

function cm(m: number): string {
  return `${(m * 100).toFixed(2)} cm`;
}

/** Instante de la muestra en que el efector `id` aplica más fuerza (0 si ninguna tiene fuerza). */
function timeOfMaxForce(recording: Recording, id: string): number {
  let best = 0;
  let t = 0;
  for (const s of recording.samples) {
    for (const e of s.effectors) {
      if (e.id !== id) continue;
      const m = Math.hypot(e.force_n[0], e.force_n[1], e.force_n[2]);
      if (m > best) { best = m; t = s.t; }
    }
  }
  return t;
}

/** Evalúa la grabación. `artifacts` vacío = limpia en todo lo evaluado; mira siempre `not_evaluated`. */
export function detectArtifacts(recording: Recording): ArtifactReport {
  const found: ArtifactFinding[] = [];
  const notEvaluated: ArtifactNotEvaluated[] = [];
  const skip = (type: ArtifactType, reason: string): void => { notEvaluated.push({ type, reason }); };
  const d = recording.diagnostics;
  const lastT = recording.samples[recording.samples.length - 1]?.t ?? 0;
  const NO_DIAG = "la grabación no trae `diagnostics` (anterior a la PR 5 de #75)";

  if (!d) {
    for (const type of ["ground_penetration", "segment_penetration", "joint_speed", "contact_force", "foot_slip", "tremor", "energy_gain"] as const) {
      skip(type, NO_DIAG);
    }
  } else {
    const gp = d.ground_penetration;
    if (gp.value > GROUND_PENETRATION_LIMIT_M) {
      found.push(fromPeak("ground_penetration", gp, GROUND_PENETRATION_LIMIT_M,
        `penetración en el suelo de ${cm(gp.value)} en ${gp.where}`));
    }
    if (d.segment_penetration) {
      const sp = d.segment_penetration;
      if (sp.value > SEGMENT_PENETRATION_LIMIT_M) {
        found.push(fromPeak("segment_penetration", sp, SEGMENT_PENETRATION_LIMIT_M,
          `penetración entre segmentos de ${cm(sp.value)} en ${sp.where}`));
      }
    } else skip("segment_penetration", "el actor tiene un solo cuerpo: no hay segmentos que se atraviesen");
    if (d.joint_speed) {
      const js = d.joint_speed;
      if (js.value > JOINT_SPEED_LIMIT_RAD_S) {
        found.push(fromPeak("joint_speed", js, JOINT_SPEED_LIMIT_RAD_S,
          `velocidad articular de ${js.value.toFixed(1)} rad/s en ${js.where}`));
      }
    } else skip("joint_speed", "el actor no tiene articulaciones internas");
    const cf = d.contact_force;
    if (cf.value > CONTACT_FORCE_LIMIT_N) {
      found.push(fromPeak("contact_force", cf, CONTACT_FORCE_LIMIT_N,
        `fuerza de contacto de ${(cf.value / 1000).toFixed(1)} kN en ${cf.where}`));
    }
    if (d.foot_slip) {
      const fs = d.foot_slip;
      if (fs.value > FOOT_SLIP_LIMIT_M) {
        found.push(fromPeak("foot_slip", fs, FOOT_SLIP_LIMIT_M,
          `${fs.where} cargado patina ${cm(fs.value)} sin saturar el rozamiento`));
      }
    } else skip("foot_slip", "el actor no tiene pies");
    if (d.tremor) {
      const tr = d.tremor;
      if (lastT < TREMOR_WINDOW_S) skip("tremor", `la grabación dura ${lastT.toFixed(2)} s, menos que la ventana de ${TREMOR_WINDOW_S} s`);
      else if (tr.value > TREMOR_REVERSALS_PER_S_LIMIT) {
        found.push(fromPeak("tremor", tr, TREMOR_REVERSALS_PER_S_LIMIT,
          `${tr.value} cambios de sentido en ${TREMOR_WINDOW_S} s (amplitud >= ${tr.amplitude_rad} rad) en ${tr.where}`));
      }
    } else skip("tremor", "el actor no tiene articulaciones internas");
    if (d.energy) {
      const limit = ENERGY_GAIN_TOLERANCE_FRACTION * d.energy.scale_j;
      if (d.energy.max_excess_j > limit) {
        found.push({
          type: "energy_gain", t_s: d.energy.t_s, where: "",
          value: d.energy.max_excess_j, threshold: limit,
          detail: `la energía mecánica supera en ${d.energy.max_excess_j.toFixed(2)} J la aportada por pares internos y efectores`,
        });
      }
    } else skip("energy_gain", "no se midió la energía");
  }

  // Topes de efectores: solo los que declaran uno.
  const noCap: string[] = [];
  for (const [id, peak] of Object.entries(recording.peaks.effectors)) {
    if (peak.limit_n === undefined) { noCap.push(id); continue; }
    if (peak.max_force_n > peak.limit_n) {
      found.push({
        type: "effector_over_limit", t_s: timeOfMaxForce(recording, id), where: id,
        value: peak.max_force_n, threshold: peak.limit_n,
        detail: `el efector "${id}" aplica ${peak.max_force_n.toFixed(0)} N, por encima de su tope de ${peak.limit_n.toFixed(0)} N`,
      });
    }
  }
  if (noCap.length > 0) skip("effector_over_limit", `sin tope declarado: ${noCap.join(", ")}`);

  // Fuerza sobre la raíz de Uke: la medida por paso del ejecutor y, en su defecto, la del resumen del humanoide.
  const root = d?.root_generalized_force ?? recording.summary.uke?.max_root_generalized_force;
  if (root === undefined) skip("root_force", "el actor no es un humanoide");
  else if (root > ROOT_FORCE_LIMIT) {
    found.push({
      type: "root_force", t_s: 0, where: "raíz (6 grados de libertad)", value: root, threshold: ROOT_FORCE_LIMIT,
      detail: `fuerza generalizada de ${root} sobre los 6 grados de la raíz de Uke`,
    });
  }

  return { artifacts: found.sort((a, b) => a.t_s - b.t_s), not_evaluated: notEvaluated };
}

/** Una línea por artefacto (para el comando `sim`). */
export function describeArtifact(a: ArtifactFinding): string {
  return `${a.type} a ${a.t_s.toFixed(3)} s: ${a.detail} (medido ${Number(a.value.toPrecision(4))}, umbral ${Number(a.threshold.toPrecision(4))})`;
}
