import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { expect, it } from "vitest";
import { runScenario } from "../../src/scenario/run";
import { validateScenario } from "../../src/scenario/validate";
import { gzipRecording } from "../../src/recording/write";
import type { HumanoidActor } from "../../src/scenario/schema";
it.skipIf(!process.env.HPL_FALL_PHASE)("compara entradas congeladas del reflejo de caída", async () => {
  const phase = process.env.HPL_FALL_PHASE!, out = resolve("out/fall-response/" + phase);
  mkdirSync(out, { recursive: true });
  const rows = [];
  for (const input of JSON.parse(readFileSync("tests/fixtures/fallResponseCases.json", "utf8"))) {
    for (const response of ["none", "instinctive"] as const) {
      const raw = structuredClone(input.scenario);
      const actor = raw.actors.find((a: HumanoidActor) => a.id === input.actor);
      actor.fall = response;
      const scenario = validateScenario(raw), trace: unknown[] = [];
      let previous: {
        tau: number[];
        vel: number[];
      } | undefined;
      let positiveWork = 0, negativeWork = 0;
      const dofs: number[] = [];
      let body = 0;
      let maxJointPower = 0;
      const r = await runScenario(scenario, { onSample: (rawM, rawD, i) => {
          const m = rawM as {
            njnt: number;
            jnt_type: Int32Array;
            jnt_bodyid: Int32Array;
            jnt_dofadr: Int32Array;
            body: (n: string) => {
              id: number;
            };
            body_rootid: Int32Array;
          }, d = rawD as {
            qfrc_applied: Float64Array;
            qvel: Float64Array;
            energy: Float64Array;
          };
          if (i === 0) {
            body = m.body(input.prefix + "pelvis").id;
            for (let j = 0; j < m.njnt; j++)
              if (m.jnt_type[j] === 3 && m.body_rootid[m.jnt_bodyid[j]!] === body) {
                dofs.push(m.jnt_dofadr[j]!);
              }
          }
          const tau = dofs.map(v => d.qfrc_applied[v]!), vel = dofs.map(v => d.qvel[v]!);
          if (previous)
            for (let j = 0; j < dofs.length; j++) {
              const work = previous.tau[j]! * (previous.vel[j]! + vel[j]!) * .001;
              positiveWork += Math.max(0, work);
              negativeWork += Math.min(0, work);
            }
          maxJointPower = Math.max(maxJointPower, ...tau.map((t, j) => Math.abs(t * vel[j]!)));
          previous = { tau, vel };
          trace.push({ t: i * .002, kinetic_j: d.energy[1], positive_work_j: positiveWork, negative_work_j: negativeWork });
        } });
      const a = r.summary.actors?.[input.actor] ?? { ...r.summary.uke!, outcome: r.summary.outcome, diagnostics: r.diagnostics, artifacts: r.summary.artifacts, artifacts_not_evaluated: r.summary.artifacts_not_evaluated };
      expect(a.max_root_generalized_force).toBe(0);
      expect(r.samples.every(s => [...s.qpos, ...s.qvel].every(Number.isFinite))).toBe(true);
      const row = { id: scenario.id, response, sha256: r.meta.scenario_sha256, actor: input.actor, summary: a, positive_work_j: positiveWork, negative_work_j: negativeWork, max_joint_power_w: maxJointPower, final_com_speed_mps: a.balance?.final_com_speed_mps };
      rows.push(row);
      const stem = scenario.id + "_" + response;
      writeFileSync(resolve(out, stem + ".json.gz"), gzipRecording(r));
      writeFileSync(resolve(out, stem + ".trace.json"), JSON.stringify(trace));
      console.log(JSON.stringify({ id: row.id, response, outcome: a.outcome, contact_n: a.diagnostics?.contact_force.value, penetration_m: a.diagnostics?.ground_penetration.value, speed: a.diagnostics?.joint_speed?.value, work_j: positiveWork, reflex: a.controller.fall_response }));
    }
  }
  writeFileSync(resolve(out, "metrics.json"), JSON.stringify({ phase, rows }, null, 2));
}, 180000);
