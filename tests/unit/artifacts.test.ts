import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import {
  CONTACT_FORCE_LIMIT_N, FOOT_SLIP_LIMIT_M, GROUND_PENETRATION_LIMIT_M, JOINT_SPEED_LIMIT_RAD_S, SEGMENT_PENETRATION_LIMIT_M,
  TREMOR_REVERSALS_PER_S_LIMIT, describeArtifact, detectArtifacts,
} from "../../src/metrics/artifacts";
import { RecordingPlayer } from "../../src/recording/player";
import { parseRecording } from "../../src/recording/read";
import type { ArtifactType, PeakAt, Recording, RecordingDiagnostics } from "../../src/recording/schema";

/**
 * Detector de artefactos sobre grabaciones SINTÉTICAS (sin simulación): por cada detector, una grabación que lo dispara y
 * otra que no. Las medidas por paso las mide `ArtifactProbe` (`artifactProbeMujoco.test.ts`); aquí solo se prueba la evaluación.
 */
const FIXTURE = new URL("../fixtures/cylinder_obstacle_low.recording.json.gz", import.meta.url);

const none = (): PeakAt => ({ value: 0, t_s: 0, where: "" });

/** Diagnósticos de un humanoide limpio (todo medido y a cero). */
function cleanDiagnostics(over: Partial<RecordingDiagnostics> = {}): RecordingDiagnostics {
  return {
    actor_mass_kg: 75,
    ground_penetration: none(),
    segment_penetration: none(),
    joint_speed: none(),
    contact_force: none(),
    foot_slip: none(),
    tremor: { ...none(), amplitude_rad: 0.005 },
    energy: { max_excess_j: 0, t_s: 0, scale_j: 750 },
    root_generalized_force: 0,
    ...over,
  };
}

function recording(diagnostics: RecordingDiagnostics | undefined, extra: Partial<Recording> = {}): Recording {
  return {
    schema: "hpl.recording.v1",
    meta: {
      scenario_id: "sintetico", scenario_sha256: "", engine_version: "test", mujoco_version: "test", physics_hz: 500, record_hz: 125,
      frame: "x_forward_y_left_z_up", quat_order: "wxyz",
    },
    model: { mjcf: "<mujoco/>", bodies: ["world"], joints: [] },
    samples: [0, 1, 2, 3].map((t) => ({ t, qpos: [0], qvel: [0], effectors: [], contacts: [] })),
    peaks: { effectors: {} },
    summary: {
      outcome: "standing", direction_deg: null, t_outcome_s: 0, final_tilt_deg: 0, max_tilt_deg: 0, final_com_height_m: 1, slide_m: 0,
      expect: "none", expect_diffs: [],
    },
    ...(diagnostics ? { diagnostics } : {}),
    ...extra,
  };
}

const types = (r: Recording): ArtifactType[] => detectArtifacts(r).artifacts.map((a) => a.type);

describe("detectArtifacts: una grabación limpia", () => {
  it("sin ningún artefacto y con todos los detectores evaluados (salvo efectores sin efectores: nada que comprobar)", () => {
    const report = detectArtifacts(recording(cleanDiagnostics()));
    expect(report.artifacts).toEqual([]);
    expect(report.not_evaluated).toEqual([]);
  });
});

describe("detectArtifacts: un caso que dispara y otro que no por detector", () => {
  const table: Array<{ type: ArtifactType; key: keyof RecordingDiagnostics; at: number; below: number; where: string }> = [
    { type: "ground_penetration", key: "ground_penetration", at: 0.0125, below: 0.0099, where: "chest" },
    { type: "segment_penetration", key: "segment_penetration", at: 0.0225, below: 0.0199, where: "left_forearm|chest" },
    { type: "joint_speed", key: "joint_speed", at: 30, below: 24.9, where: "left_knee" },
    { type: "contact_force", key: "contact_force", at: 12_000, below: 9_900, where: "chest|world" },
    { type: "foot_slip", key: "foot_slip", at: 0.03, below: 0.0199, where: "left_foot" },
  ];
  for (const row of table) {
    it(`${row.type}: dispara con ${row.at} y no con ${row.below}`, () => {
      const hit = recording(cleanDiagnostics({ [row.key]: { value: row.at, t_s: 1.5, where: row.where } }));
      const report = detectArtifacts(hit);
      expect(report.artifacts).toHaveLength(1);
      expect(report.artifacts[0]).toMatchObject({ type: row.type, t_s: 1.5, where: row.where, value: row.at });
      expect(report.artifacts[0]!.threshold).toBeGreaterThan(0);
      expect(report.artifacts[0]!.detail.length).toBeGreaterThan(10);
      expect(types(recording(cleanDiagnostics({ [row.key]: { value: row.below, t_s: 1.5, where: row.where } })))).toEqual([]);
    });
  }

  it("los umbrales son los de la sección 12.5 y se supera con estrictamente más", () => {
    expect([GROUND_PENETRATION_LIMIT_M, SEGMENT_PENETRATION_LIMIT_M, JOINT_SPEED_LIMIT_RAD_S, CONTACT_FORCE_LIMIT_N, FOOT_SLIP_LIMIT_M, TREMOR_REVERSALS_PER_S_LIMIT])
      .toEqual([0.01, 0.02, 25, 10_000, 0.02, 10]);
    expect(types(recording(cleanDiagnostics({ ground_penetration: { value: 0.01, t_s: 1, where: "chest" } })))).toEqual([]);
    expect(types(recording(cleanDiagnostics({ contact_force: { value: 10_000, t_s: 1, where: "a|b" } })))).toEqual([]);
    expect(types(recording(cleanDiagnostics({ tremor: { value: 10, t_s: 1, where: "left_knee", amplitude_rad: 0.005 } })))).toEqual([]);
    expect(types(recording(cleanDiagnostics({ tremor: { value: 11, t_s: 1, where: "left_knee", amplitude_rad: 0.005 } })))).toEqual(["tremor"]);
  });

  it("tremor: 11 cambios de sentido en 1 s dispara y 6 no", () => {
    const hit = detectArtifacts(recording(cleanDiagnostics({ tremor: { value: 14, t_s: 2.2, where: "left_knee", amplitude_rad: 0.005 } })));
    expect(hit.artifacts).toHaveLength(1);
    expect(hit.artifacts[0]).toMatchObject({ type: "tremor", where: "left_knee", value: 14, threshold: 10 });
    expect(types(recording(cleanDiagnostics({ tremor: { value: 6, t_s: 2.2, where: "left_knee", amplitude_rad: 0.005 } })))).toEqual([]);
  });

  it("energy_gain: el exceso de 20 J sobre una escala de 750 J (tolerancia 7,5 J) dispara; 5 J no", () => {
    const hit = detectArtifacts(recording(cleanDiagnostics({ energy: { max_excess_j: 20, t_s: 0.8, scale_j: 750 } })));
    expect(hit.artifacts).toHaveLength(1);
    expect(hit.artifacts[0]).toMatchObject({ type: "energy_gain", t_s: 0.8, value: 20, threshold: 7.5 });
    expect(types(recording(cleanDiagnostics({ energy: { max_excess_j: 5, t_s: 0.8, scale_j: 750 } })))).toEqual([]);
    // La tolerancia depende de la escala: el mismo exceso de 5 J sobre un actor ligero sí es una fuente oculta.
    expect(types(recording(cleanDiagnostics({ energy: { max_excess_j: 5, t_s: 0.8, scale_j: 300 } })))).toEqual(["energy_gain"]);
  });

  it("effector_over_limit: solo con tope declarado; 612 N sobre 600 N dispara, 500 N sobre 600 N no", () => {
    const withForce = (max: number): Recording => recording(cleanDiagnostics(), {
      peaks: { effectors: { hand: { max_force_n: max, impulse_ns: [0, 0, 0], work_j: 0, limit_n: 600 } } },
      samples: [0, 1, 2].map((t) => ({
        t, qpos: [0], qvel: [0], contacts: [],
        effectors: [{ id: "hand", point_m: [0, 0, 1] as const, force_n: [t === 1 ? max : 0, 0, 0] as const }],
      })),
    });
    const hit = detectArtifacts(withForce(612));
    expect(hit.artifacts).toHaveLength(1);
    expect(hit.artifacts[0]).toMatchObject({ type: "effector_over_limit", where: "hand", t_s: 1, value: 612, threshold: 600 });
    expect(detectArtifacts(withForce(500)).artifacts).toEqual([]);
    expect(detectArtifacts(withForce(500)).not_evaluated).toEqual([]);
  });

  it("root_force: cualquier fuerza generalizada en la raíz dispara; 0 no", () => {
    const hit = detectArtifacts(recording(cleanDiagnostics({ root_generalized_force: 0.4 })));
    expect(hit.artifacts).toHaveLength(1);
    expect(hit.artifacts[0]).toMatchObject({ type: "root_force", value: 0.4 });
    expect(types(recording(cleanDiagnostics({ root_generalized_force: 0 })))).toEqual([]);
    // Sin medida del paso, vale el máximo del resumen del humanoide.
    const viaSummary = recording(undefined, { summary: { ...recording(undefined).summary, uke: { max_root_generalized_force: 2 } as never } });
    expect(types(viaSummary)).toEqual(["root_force"]);
  });

  it("varios artefactos salen ordenados por instante y describeArtifact los resume", () => {
    const report = detectArtifacts(recording(cleanDiagnostics({
      contact_force: { value: 15_000, t_s: 1.9, where: "chest|world" },
      ground_penetration: { value: 0.02, t_s: 0.7, where: "chest" },
    })));
    expect(report.artifacts.map((a) => a.type)).toEqual(["ground_penetration", "contact_force"]);
    expect(describeArtifact(report.artifacts[1]!)).toContain("contact_force a 1.900 s");
  });
});

describe("detectArtifacts: lo que no se puede evaluar se dice, no se da por limpio", () => {
  const reasonOf = (r: Recording, type: ArtifactType): string | undefined =>
    detectArtifacts(r).not_evaluated.find((n) => n.type === type)?.reason;

  it("una grabación antigua (sin `diagnostics`) no lanza y no figura como limpia: todo queda no evaluado", async () => {
    const old = await parseRecording(new Uint8Array(readFileSync(FIXTURE)));
    expect(old.diagnostics).toBeUndefined();
    expect(old.summary.artifacts).toBeUndefined();
    const report = detectArtifacts(old);
    expect(report.artifacts).toEqual([]);
    const skipped = report.not_evaluated.map((n) => n.type);
    for (const t of ["ground_penetration", "contact_force", "energy_gain", "tremor", "joint_speed", "foot_slip", "segment_penetration"] as const) {
      expect(skipped).toContain(t);
    }
    expect(reasonOf(old, "contact_force")).toContain("diagnostics");
    // Sigue leyéndose y reproduciéndose.
    expect(new RecordingPlayer(old, []).sampleAt(0).t).toBe(0);
  });

  it("un actor sin articulaciones, sin pies ni segmentos (cilindro) marca esos detectores como no evaluados", () => {
    const cylinder = recording({
      actor_mass_kg: 75, ground_penetration: none(), contact_force: none(), energy: { max_excess_j: 0, t_s: 0, scale_j: 640 },
    });
    const skipped = detectArtifacts(cylinder).not_evaluated.map((n) => n.type).sort();
    expect(skipped).toEqual(["foot_slip", "joint_speed", "root_force", "segment_penetration", "tremor"]);
  });

  it("un efector sin tope declarado queda no evaluado (con su id), aunque aplique 5000 N", () => {
    const r = recording(cleanDiagnostics(), { peaks: { effectors: { pull: { max_force_n: 5000, impulse_ns: [0, 0, 0], work_j: 0 } } } });
    expect(detectArtifacts(r).artifacts).toEqual([]);
    expect(reasonOf(r, "effector_over_limit")).toContain("pull");
  });

  it("el temblor no se evalúa en una grabación de menos de 1 s", () => {
    const short = recording(cleanDiagnostics({ tremor: { value: 40, t_s: 0.3, where: "left_knee", amplitude_rad: 0.005 } }), {
      samples: [0, 0.4].map((t) => ({ t, qpos: [0], qvel: [0], effectors: [], contacts: [] })),
    });
    expect(types(short)).toEqual([]);
    expect(reasonOf(short, "tremor")).toContain("ventana");
  });

  it("sin medida de la energía, energy_gain queda no evaluado", () => {
    const d = cleanDiagnostics();
    delete d.energy;
    expect(reasonOf(recording(d), "energy_gain")).toBeDefined();
  });
});
