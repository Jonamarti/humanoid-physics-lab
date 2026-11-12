/**
 * Comando sin navegador (issue #74, PR 5):
 *
 *   npm run sim -- <escenario.json> --out <carpeta>
 *   npm run sim -- --batch <carpeta_de_escenarios> --out <carpeta>
 *
 * Escribe `recording.json.gz` y `summary.json` (y, con `--batch`, `batch.csv`) e imprime el resumen.
 * Código de salida: 0 si `expect` pasa o no existe, 1 si falla, 2 si el escenario es inválido o el uso es incorrecto, 3 si el
 * ejecutor lanza una excepción (lo decide `scripts/sim.ts`) y 4 con `--strict` si el detector de artefactos encuentra alguno.
 * Por defecto los artefactos solo se imprimen y se guardan en `summary.artifacts`.
 * Solo Node. La lógica está aquí (y no en `scripts/sim.ts`) para poder probarla con Vitest.
 */
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { basename, join } from "node:path";
import { describeArtifact } from "../metrics/artifacts";
import type { Recording } from "../recording/schema";
import { gzipRecording, roundDeep } from "../recording/write";
import { readEngineVersion } from "./engineVersion";
import { runScenario } from "./run";
import { ScenarioValidationError, validateScenario } from "./validate";

export const EXIT_OK = 0;
export const EXIT_EXPECT_FAILED = 1;
export const EXIT_INVALID = 2;
/** Excepción del ejecutor: lo asigna `scripts/sim.ts` (no `runCli`). Ya existía; la PR 5 de #75 no lo cambia. */
export const EXIT_RUNTIME_ERROR = 3;
/** `--strict` y el detector de artefactos encontró alguno. (El 3 ya significaba excepción del ejecutor.) */
export const EXIT_ARTIFACTS = 4;

export interface CliIo {
  out: (line: string) => void;
  err: (line: string) => void;
}

const defaultIo: CliIo = {
  out: (line) => process.stdout.write(`${line}\n`),
  err: (line) => process.stderr.write(`${line}\n`),
};

export const USAGE = [
  "Uso:",
  "  npm run sim -- <escenario.json> [--out <carpeta>] [--strict]",
  "  npm run sim -- --batch <carpeta_de_escenarios> [--out <carpeta>] [--strict]",
  "Escribe recording.json.gz y summary.json en <carpeta> (por defecto out/<id>; con --batch, out/batch y una subcarpeta por",
  "escenario más batch.csv). Salida: 0 expect correcto o ausente, 1 expect incumplido, 2 escenario inválido o uso incorrecto,",
  "3 excepción del ejecutor, 4 (solo con --strict) el detector de artefactos encontró alguno.",
].join("\n");

interface ParsedArgs {
  scenarioPath?: string;
  batchDir?: string;
  outDir?: string;
  strict: boolean;
}

function parseArgs(argv: readonly string[]): ParsedArgs | string {
  const parsed: ParsedArgs = { strict: false };
  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i]!;
    if (arg === "--out" || arg === "--batch") {
      const value = argv[++i];
      if (value === undefined || value.startsWith("--")) return `falta el valor de ${arg}`;
      if (arg === "--out") parsed.outDir = value;
      else parsed.batchDir = value;
    } else if (arg === "--strict") {
      parsed.strict = true;
    } else if (arg.startsWith("--")) {
      return `opción desconocida ${arg}`;
    } else if (parsed.scenarioPath === undefined) {
      parsed.scenarioPath = arg;
    } else {
      return `sobra el argumento ${arg}`;
    }
  }
  if (parsed.batchDir !== undefined && parsed.scenarioPath !== undefined) return "use un escenario o --batch, no los dos";
  if (parsed.batchDir === undefined && parsed.scenarioPath === undefined) return "falta el escenario";
  return parsed;
}

interface Outcome {
  /** `invalid` si el escenario no se pudo leer o validar. */
  kind: "ok" | "invalid";
  scenarioId: string;
  message?: string;
  recording?: Recording;
  recordingBytes?: number;
}

/** Lee, valida y ejecuta un escenario; escribe sus ficheros en `outDir`. Nunca lanza por un escenario inválido. */
async function runFile(path: string, outDir: string): Promise<Outcome> {
  const fileName = basename(path);
  let json: unknown;
  try {
    // PowerShell 5.1 escribe UTF-8 con BOM: se ignora.
    const text = readFileSync(path, "utf8");
    json = JSON.parse(text.charCodeAt(0) === 0xfeff ? text.slice(1) : text) as unknown;
  } catch (error) {
    return { kind: "invalid", scenarioId: fileName.replace(/\.json$/i, ""), message: `${path}: no se pudo leer el JSON (${(error as Error).message})` };
  }
  let scenario;
  try {
    scenario = validateScenario(json, { fileName });
  } catch (error) {
    if (error instanceof ScenarioValidationError) {
      return { kind: "invalid", scenarioId: fileName.replace(/\.json$/i, ""), message: `${path}: escenario inválido: ${error.message}` };
    }
    throw error;
  }
  const recording = await runScenario(scenario, { engineVersion: readEngineVersion() });
  const gz = gzipRecording(recording);
  mkdirSync(outDir, { recursive: true });
  writeFileSync(join(outDir, "recording.json.gz"), gz);
  const summaryFile = {
    scenario_id: recording.meta.scenario_id,
    scenario_sha256: recording.meta.scenario_sha256,
    engine_version: recording.meta.engine_version,
    mujoco_version: recording.meta.mujoco_version,
    summary: recording.summary,
    peaks: recording.peaks,
    recording: { file: "recording.json.gz", bytes: gz.length, samples: recording.samples.length },
  };
  writeFileSync(join(outDir, "summary.json"), `${JSON.stringify(roundDeep(summaryFile), null, 2)}\n`);
  return { kind: "ok", scenarioId: scenario.id, recording, recordingBytes: gz.length };
}

function describeSummary(recording: Recording): string[] {
  const s = recording.summary;
  // `+ 0` evita escribir «-0.0».
  const direction = s.direction_deg === null ? "-" : `${(Number(s.direction_deg.toFixed(1)) + 0).toFixed(1)}°`;
  const lines = [
    `${recording.meta.scenario_id}: ${s.outcome}, dirección ${direction}, t_outcome ${s.t_outcome_s.toFixed(3)} s, ` +
      `inclinación máx ${s.max_tilt_deg.toFixed(1)}°, final ${s.final_tilt_deg.toFixed(1)}°, ` +
      `altura CM ${s.final_com_height_m.toFixed(3)} m, deslizamiento ${s.slide_m.toFixed(3)} m, expect ${s.expect}`,
  ];
  const u = s.uke;
  if (u?.unbalance) {
    lines.push(`  desequilibrio: ${u.unbalance.direction} a los ${u.unbalance.t_s.toFixed(3)} s (margen mín ${u.unbalance.min_margin_m.toFixed(3)} m, ${u.unbalance.recovered ? "recuperado" : "no recuperado"})`);
  }
  if (u?.fall) {
    const f = u.fall;
    lines.push(`  caída: ${f.t_s.toFixed(3)} s (${f.trigger}), dirección ${f.direction_name ?? "-"}, primera parte ${f.first_part ?? "-"}`);
  }
  const b = s.tori_budget;
  if (b) {
    lines.push(
      `  Tori (${b.mass_kg} kg): ${b.feasible ? "viable" : "INVIABLE"}; rozamiento usado ${(100 * b.friction_use.value).toFixed(0)} % a los ${b.friction_use.t_s.toFixed(2)} s, ` +
        `carga en los pies ${b.min_feet_load_n.value.toFixed(0)} a ${b.max_feet_load_n.value.toFixed(0)} N, tiro total máx ${b.max_total_pull_n.value.toFixed(0)} N, ` +
        `potencia máx ${b.max_total_power_w.value.toFixed(0)} W`,
    );
    for (const v of b.violations) {
      lines.push(`  Tori inviable: ${v.kind} desde ${v.t_s.toFixed(3)} s durante ${v.duration_s.toFixed(3)} s (medido ${v.value.toFixed(0)}, tope ${v.limit.toFixed(0)})`);
    }
  }
  for (const diff of s.expect_diffs) lines.push(`  diferencia: ${diff}`);
  if (s.artifacts !== undefined) {
    if (s.artifacts.length === 0) lines.push("  artefactos: ninguno");
    else for (const a of s.artifacts) lines.push(`  artefacto: ${describeArtifact(a)}`);
  }
  const skipped = s.artifacts_not_evaluated ?? [];
  if (skipped.length > 0) lines.push(`  no evaluado: ${skipped.map((n) => `${n.type} (${n.reason})`).join("; ")}`);
  return lines;
}

function csvCell(value: string | number | null): string {
  if (value === null) return "";
  const text = typeof value === "number" ? String(value) : value;
  return /[",\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
}

export const BATCH_CSV_HEADER = [
  "scenario_id", "status", "outcome", "direction_deg", "t_outcome_s", "final_tilt_deg", "max_tilt_deg",
  "final_com_height_m", "slide_m", "expect", "expect_diffs", "recording_bytes", "artifacts",
].join(",");

function csvRow(o: Outcome): string {
  if (o.kind === "invalid" || !o.recording) {
    return [o.scenarioId, "invalid", "", "", "", "", "", "", "", "invalid", o.message ?? "", "", ""].map((c) => csvCell(c)).join(",");
  }
  const s = o.recording.summary;
  const round = (x: number): number => Number(x.toPrecision(6));
  return [
    o.scenarioId, "ok", s.outcome, s.direction_deg === null ? null : round(s.direction_deg), round(s.t_outcome_s),
    round(s.final_tilt_deg), round(s.max_tilt_deg), round(s.final_com_height_m), round(s.slide_m), s.expect,
    s.expect_diffs.join("; "), o.recordingBytes ?? null, (s.artifacts ?? []).map((a) => a.type).join(";"),
  ].map((c) => csvCell(c)).join(",");
}

function exitCodeOf(outcomes: readonly Outcome[], strict: boolean): number {
  if (outcomes.some((o) => o.kind === "invalid")) return EXIT_INVALID;
  if (outcomes.some((o) => o.recording?.summary.expect === "fail")) return EXIT_EXPECT_FAILED;
  if (strict && outcomes.some((o) => (o.recording?.summary.artifacts?.length ?? 0) > 0)) return EXIT_ARTIFACTS;
  return EXIT_OK;
}

/** Ejecuta el comando con los argumentos (sin `node` ni el script) y devuelve el código de salida. */
export async function runCli(argv: readonly string[], io: CliIo = defaultIo): Promise<number> {
  const parsed = parseArgs(argv);
  if (typeof parsed === "string") {
    io.err(`${parsed}\n${USAGE}`);
    return EXIT_INVALID;
  }

  if (parsed.batchDir !== undefined) {
    let files: string[];
    try {
      files = readdirSync(parsed.batchDir).filter((f) => f.toLowerCase().endsWith(".json")).sort();
    } catch (error) {
      io.err(`no se pudo leer la carpeta ${parsed.batchDir}: ${(error as Error).message}`);
      return EXIT_INVALID;
    }
    if (files.length === 0) {
      io.err(`no hay escenarios .json en ${parsed.batchDir}`);
      return EXIT_INVALID;
    }
    const outRoot = parsed.outDir ?? join("out", "batch");
    const outcomes: Outcome[] = [];
    for (const file of files) {
      const id = file.replace(/\.json$/i, "");
      const outcome = await runFile(join(parsed.batchDir, file), join(outRoot, id));
      outcomes.push(outcome);
      if (outcome.recording) for (const line of describeSummary(outcome.recording)) io.out(line);
      else io.err(outcome.message ?? `${file}: escenario inválido`);
    }
    mkdirSync(outRoot, { recursive: true });
    writeFileSync(join(outRoot, "batch.csv"), `${BATCH_CSV_HEADER}\n${outcomes.map(csvRow).join("\n")}\n`);
    const code = exitCodeOf(outcomes, parsed.strict);
    const passed = outcomes.filter((o) => o.recording && o.recording.summary.expect !== "fail").length;
    const withArtifacts = outcomes.filter((o) => (o.recording?.summary.artifacts?.length ?? 0) > 0).length;
    io.out(`batch: ${passed}/${outcomes.length} sin incumplir expect, ${withArtifacts} con artefactos; tabla en ${join(outRoot, "batch.csv")}`);
    return code;
  }

  const scenarioPath = parsed.scenarioPath!;
  const outcome = await runFile(scenarioPath, parsed.outDir ?? join("out", basename(scenarioPath).replace(/\.json$/i, "")));
  if (!outcome.recording) {
    io.err(outcome.message ?? "escenario inválido");
    return EXIT_INVALID;
  }
  for (const line of describeSummary(outcome.recording)) io.out(line);
  return exitCodeOf([outcome], parsed.strict);
}
