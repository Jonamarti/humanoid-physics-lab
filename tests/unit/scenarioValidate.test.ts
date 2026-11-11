import { describe, expect, it } from "vitest";
import { validateScenario, ScenarioValidationError } from "../../src/scenario/validate";

const VALID = {
  schema: "hpl.scenario.v1",
  id: "cylinder_obstacle_low",
  duration_s: 3.0,
  record_hz: 125,
  ground: { friction: 0.6 },
  actors: [
    { id: "uke", kind: "rigid_cylinder", mass_kg: 75, height_m: 1.75, radius_m: 0.18 },
  ],
  props: [
    { id: "obstacle", kind: "fixed_box", size_m: [0.06, 0.4, 0.05], pos_m: [0.26, 0, 0.05] },
  ],
  effectors: [
    {
      id: "pull", kind: "prescribed_force", actor: "uke",
      body: "cylinder", local_point_m: [0, 0, 0.525],
      frame: "world", force_n: [600, 0, 180],
      start_s: 0.3, duration_s: 0.15, waveform: "half_sine",
    },
  ],
  expect: { outcome: "fall", direction_deg: 0, tolerance_deg: 30 },
};

/** Copia profunda del caso válido con una modificación. */
// eslint-disable-next-line @typescript-eslint/no-explicit-any
function mutated(edit: (s: any) => void): unknown {
  const copy = structuredClone(VALID);
  edit(copy);
  return copy;
}

function messageOf(json: unknown, fileName?: string): string {
  try {
    validateScenario(json, fileName === undefined ? {} : { fileName });
  } catch (e) {
    expect(e).toBeInstanceOf(ScenarioValidationError);
    return (e as Error).message;
  }
  throw new Error("se esperaba un error de validación");
}

describe("validateScenario", () => {
  it("acepta el escenario de ejemplo de la issue #74 (con record_hz 125, ver abajo) y lo devuelve igual", () => {
    const scenario = validateScenario(structuredClone(VALID), { fileName: "scenarios/cylinder_obstacle_low.json" });
    expect(scenario).toEqual(VALID);
  });

  it("acepta un escenario sin expect", () => {
    const withoutExpect = structuredClone(VALID) as Record<string, unknown>;
    delete withoutExpect.expect;
    expect(validateScenario(withoutExpect).expect).toBeUndefined();
  });

  const invalid: Array<[string, unknown, string]> = [
    ["raíz que no es un objeto", [], "(raíz): debe ser un objeto"],
    ["schema distinto de hpl.scenario.v1", mutated((s) => { s.schema = "hpl.scenario.v2"; }),
      `schema: debe ser "hpl.scenario.v1", no "hpl.scenario.v2"`],
    ["campo desconocido en la raíz", mutated((s) => { s.gravity = 9.81; }), "gravity: campo desconocido"],
    ["campo desconocido en un efector", mutated((s) => { s.effectors[0].max_force_n = 1; }),
      "effectors[0].max_force_n: campo desconocido"],
    ["campo obligatorio ausente", mutated((s) => { delete s.ground; }), "ground: campo obligatorio ausente"],
    ["número no finito (NaN)", mutated((s) => { s.effectors[0].force_n[1] = NaN; }),
      "effectors[0].force_n[1]: debe ser un número finito"],
    ["número no finito (Infinity)", mutated((s) => { s.actors[0].mass_kg = Infinity; }),
      "actors[0].mass_kg: debe ser un número finito"],
    ["número dado como cadena", mutated((s) => { s.ground.friction = "0.6"; }),
      "ground.friction: debe ser un número"],
    ["id con caracteres no permitidos", mutated((s) => { s.id = "Cylinder-Low"; }),
      "id: solo admite [a-z0-9_]"],
    ["id distinto del nombre del fichero", VALID, `id: debe ser igual al nombre del fichero "otro_nombre"`],
    ["duration_s cero", mutated((s) => { s.duration_s = 0; }), "duration_s: debe ser mayor que 0"],
    ["duration_s mayor de 30 s", mutated((s) => { s.duration_s = 30.5; }), "duration_s: no puede superar 30 s"],
    ["duración de efector mayor de 30 s", mutated((s) => { s.effectors[0].duration_s = 31; }),
      "effectors[0].duration_s: no puede superar 30 s"],
    ["record_hz por debajo de 30", mutated((s) => { s.record_hz = 25; }), "record_hz: debe estar entre 30 y 500 Hz"],
    ["record_hz por encima de 500", mutated((s) => { s.record_hz = 1000; }), "record_hz: debe estar entre 30 y 500 Hz"],
    ["record_hz que no divide a 500", mutated((s) => { s.record_hz = 125.5; }),
      "record_hz: debe ser divisor exacto de la frecuencia física (500 Hz)"],
    ["record_hz 300 no divide a 500", mutated((s) => { s.record_hz = 300; }),
      "record_hz: debe ser divisor exacto de la frecuencia física (500 Hz)"],
    // El ejemplo de la issue #74 usa 120 Hz, que contradice su propia regla (500 / 120 no es entero).
    ["record_hz 120 (valor del ejemplo de la issue) no divide a 500", mutated((s) => { s.record_hz = 120; }),
      "record_hz: debe ser divisor exacto de la frecuencia física (500 Hz), no 120"],
    ["efector con actor inexistente", mutated((s) => { s.effectors[0].actor = "tori"; }),
      `effectors[0].actor: referencia a un actor inexistente "tori"`],
    ["efector con cuerpo inexistente", mutated((s) => { s.effectors[0].body = "pelvis"; }),
      `effectors[0].body: el actor "uke" no tiene el cuerpo "pelvis"`],
    ["marco desconocido", mutated((s) => { s.effectors[0].frame = "local"; }),
      "effectors[0].frame: debe ser uno de world, actor_initial, actor_current"],
    ["terna con longitud incorrecta", mutated((s) => { s.props[0].pos_m = [0, 0]; }),
      "props[0].pos_m: debe tener exactamente 3 números"],
    ["arista no positiva", mutated((s) => { s.props[0].size_m[2] = 0; }), "props[0].size_m[2]: debe ser mayor que 0"],
    ["masa negativa", mutated((s) => { s.actors[0].mass_kg = -75; }), "actors[0].mass_kg: debe ser mayor que 0"],
    ["id de efector duplicado", mutated((s) => { s.effectors.push(structuredClone(s.effectors[0])); }),
      `effectors[1].id: id duplicado "pull"`],
    ["start_s negativo en el segundo efector", mutated((s) => {
      const second = structuredClone(s.effectors[0]);
      second.id = "push";
      second.start_s = -0.1;
      s.effectors.push(second);
    }), "effectors[1].start_s: no puede ser negativo"],
    ["outcome desconocido", mutated((s) => { s.expect.outcome = "roll"; }),
      "expect.outcome: debe ser uno de standing, tilted, fall"],
    ["tolerancia mayor de 180 grados", mutated((s) => { s.expect.tolerance_deg = 200; }),
      "expect.tolerance_deg: no puede superar 180 grados"],
    ["direction_deg con outcome distinto de fall", mutated((s) => { s.expect.outcome = "standing"; }),
      `expect.direction_deg: solo se admite con outcome "fall"`],
  ];

  it.each(invalid)("rechaza: %s", (name, json, fragment) => {
    const fileName = name.startsWith("id distinto") ? "otro_nombre.json" : undefined;
    expect(messageOf(json, fileName)).toContain(fragment);
  });

  it("incluye la ruta y el motivo como propiedades del error", () => {
    try {
      validateScenario(mutated((s) => { s.effectors[0].duration_s = 0; }));
      throw new Error("no lanzó");
    } catch (e) {
      expect(e).toBeInstanceOf(ScenarioValidationError);
      expect((e as ScenarioValidationError).path).toBe("effectors[0].duration_s");
      expect((e as ScenarioValidationError).reason).toContain("mayor que 0");
    }
  });
});
