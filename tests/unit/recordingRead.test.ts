import { readFileSync } from "node:fs";
import { gzipSync } from "node:zlib";
import { describe, expect, it } from "vitest";
import { parseRecording } from "../../src/recording/read";

const FIXTURE = new URL("../fixtures/cylinder_obstacle_low.recording.json.gz", import.meta.url);

function minimal(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    schema: "hpl.recording.v1",
    meta: { scenario_id: "x", record_hz: 125 },
    model: { mjcf: "<mujoco/>", bodies: ["world"], joints: [] },
    samples: [{ t: 0, qpos: [0], qvel: [], effectors: [], contacts: [] }],
    peaks: { effectors: {} },
    summary: { outcome: "standing" },
    ...overrides,
  };
}

describe("parseRecording", () => {
  it("lee la grabación de referencia .json.gz (bytes)", async () => {
    const bytes = new Uint8Array(readFileSync(FIXTURE));
    expect(bytes[0]).toBe(0x1f);
    const r = await parseRecording(bytes);
    expect(r.schema).toBe("hpl.recording.v1");
    expect(r.meta.scenario_id).toBe("cylinder_obstacle_low");
    expect(r.samples.length).toBeGreaterThan(100);
    expect(r.model.mjcf).toContain("<mujoco");
    expect(r.summary.outcome).toBe("fall");
  });

  it("lee el mismo contenido como .json (texto y bytes) y da lo mismo que el gz", async () => {
    const gz = await parseRecording(new Uint8Array(readFileSync(FIXTURE)));
    const text = JSON.stringify(gz);
    expect(await parseRecording(text)).toEqual(gz);
    expect(await parseRecording(new TextEncoder().encode(text))).toEqual(gz);
    const buf = gzipSync(Buffer.from(text));
    expect(await parseRecording(Uint8Array.from(buf).buffer as ArrayBuffer)).toEqual(gz);
  });

  it("acepta un BOM inicial", async () => {
    expect((await parseRecording("﻿" + JSON.stringify(minimal()))).meta.scenario_id).toBe("x");
  });

  it("versión desconocida → error claro", async () => {
    await expect(parseRecording(JSON.stringify(minimal({ schema: "hpl.recording.v2" }))))
      .rejects.toThrow(/Versión de grabación desconocida.*hpl\.recording\.v2/);
    await expect(parseRecording(JSON.stringify({ foo: 1 }))).rejects.toThrow(/desconocida/);
  });

  it("campos mínimos ausentes o incoherentes → error con la ruta", async () => {
    await expect(parseRecording("no es json")).rejects.toThrow(/JSON válido/);
    await expect(parseRecording(JSON.stringify(minimal({ samples: [] })))).rejects.toThrow(/samples/);
    await expect(parseRecording(JSON.stringify(minimal({ model: { bodies: [], joints: [] } })))).rejects.toThrow(/model\.mjcf/);
    await expect(parseRecording(JSON.stringify(minimal({ summary: {} })))).rejects.toThrow(/summary\.outcome/);
    const two = [
      { t: 0, qpos: [0], qvel: [], effectors: [], contacts: [] },
      { t: 0.1, qpos: [0, 1], qvel: [], effectors: [], contacts: [] },
    ];
    await expect(parseRecording(JSON.stringify(minimal({ samples: two })))).rejects.toThrow(/samples\[1\]\.qpos/);
    const backwards = [
      { t: 0.1, qpos: [0], qvel: [], effectors: [], contacts: [] },
      { t: 0.05, qpos: [0], qvel: [], effectors: [], contacts: [] },
    ];
    await expect(parseRecording(JSON.stringify(minimal({ samples: backwards })))).rejects.toThrow(/crecientes/);
  });
});
