import { describe, expect, it } from "vitest";
import {
  STABLE_HOLD_S, classifyHumanoidTrajectory,
  type BalanceFrame, type GroundTouch, type HumanoidTrajectory,
} from "../../src/metrics/outcome";
import type { XY } from "../../src/metrics/balance";

/**
 * Trayectorias sintéticas (sin simulación) para cada rama del veredicto del humanoide. Pies de 0,268 m a ±0,09 m; apoyo
 * de pie: rectángulo `x` de -0,07 a 0,198 e `y` de -0,16 a 0,16; centro de masas a 1 m sobre el origen.
 */
const DT = 0.05;
const STAND_POLY: XY[] = [[-0.07, -0.16], [0.198, -0.16], [0.198, 0.16], [-0.07, 0.16]];
const PELVIS_Z0 = 0.95;

interface FrameOptions {
  xcom?: XY;
  polygon?: readonly XY[];
  speed?: number;
  pelvisZ?: number;
  left?: XY;
  right?: XY;
  trunkDir?: number | null;
}

function frame(t: number, o: FrameOptions = {}): BalanceFrame {
  const xcom = o.xcom ?? [0, 0];
  return {
    t,
    com: [xcom[0], xcom[1], 1],
    com_velocity: [o.speed ?? 0, 0, 0],
    xcom,
    support_polygon: o.polygon ?? STAND_POLY,
    pelvis_z: o.pelvisZ ?? PELVIS_Z0,
    feet: { left: o.left ?? [0, 0.09], right: o.right ?? [0, -0.09] },
    trunk_dir_deg: o.trunkDir ?? null,
  };
}

/** Frames de 0 a `durationS` con la función `f(t)`. */
function series(durationS: number, f: (t: number) => FrameOptions): BalanceFrame[] {
  const n = Math.round(durationS / DT);
  return Array.from({ length: n + 1 }, (_, i) => frame(i * DT, f(i * DT)));
}

function traj(frames: readonly BalanceFrame[], touches: readonly GroundTouch[] = [], heading = 0): HumanoidTrajectory {
  return { frames, touches, initial_pelvis_z: PELVIS_Z0, heading_rad: heading };
}

const touch = (body: string, t_s: number, v: [number, number, number] = [0, 0, -2]): GroundTouch => ({ body, t_s, velocity_mps: v });

describe("humanoide: standing", () => {
  it("quieto 3 s sin mover los pies: standing, sin desequilibrio ni paso ni caída", () => {
    const v = classifyHumanoidTrajectory(traj(series(3, () => ({}))));
    expect(v.outcome).toBe("standing");
    expect(v.t_outcome_s).toBe(0);
    expect(v.fall).toBeNull();
    expect(v.step).toBeNull();
    expect(v.unbalance).toBeNull();
    expect(v.balance.stable_since_s).toBe(0);
    expect(v.balance.min_capture_margin_m).toBeCloseTo(0.07, 12); // el borde trasero está a 7 cm del origen
    expect(v.balance.max_foot_displacement_m).toEqual({ left: 0, right: 0 });
  });

  it("un empujón que saca el punto de captura del apoyo y vuelve: standing desde que se estabiliza, con desequilibrio hacia atrás", () => {
    // Entre 0,5 y 1,0 s el punto de captura está 10 cm detrás del borde trasero y con velocidad alta.
    const v = classifyHumanoidTrajectory(traj(series(4, (t) =>
      t >= 0.5 && t < 1.0 ? { xcom: [-0.17, 0], speed: 0.4 } : t >= 1.0 && t < 1.5 ? { speed: 0.3 } : {})));
    expect(v.outcome).toBe("standing");
    expect(v.t_outcome_s).toBeCloseTo(1.5, 9);
    expect(v.unbalance).toMatchObject({ t_s: 0.5, direction: "back", direction_deg: 180, recovered: true });
    expect(v.unbalance!.min_margin_m).toBeCloseTo(-0.1, 9);
  });

  it("dura menos de 2 s: no se puede decidir, unresolved", () => {
    const v = classifyHumanoidTrajectory(traj(series(1.5, () => ({}))));
    expect(v.outcome).toBe("unresolved");
    expect(v.t_outcome_s).toBeCloseTo(1.5, 9);
    // Con exactamente 2 s ya vale.
    expect(classifyHumanoidTrajectory(traj(series(STABLE_HOLD_S, () => ({})))).outcome).toBe("standing");
  });

  it("velocidad del centro de masas >= 0,1 m/s al final: unresolved; con 0,099 m/s, standing", () => {
    expect(classifyHumanoidTrajectory(traj(series(4, (t) => ({ speed: t > 3.9 ? 0.1 : 0 })))).outcome).toBe("unresolved");
    expect(classifyHumanoidTrajectory(traj(series(4, () => ({ speed: 0.099 })))).outcome).toBe("standing");
  });

  it("punto de captura fuera del apoyo al final: unresolved, con el margen negativo", () => {
    const v = classifyHumanoidTrajectory(traj(series(4, (t) => (t > 3 ? { xcom: [0.3, 0] } : {}))));
    expect(v.outcome).toBe("unresolved");
    expect(v.balance.final_capture_margin_m).toBeCloseTo(-0.102, 9);
    expect(v.balance.stable_since_s).toBeNull();
    expect(v.unbalance).toMatchObject({ direction: "front", recovered: false });
  });

  it("estable solo durante 1,9 s al final: unresolved; durante 2,0 s: standing", () => {
    const unstable = (): FrameOptions => ({ xcom: [0.5, 0] });
    const a = classifyHumanoidTrajectory(traj(series(4, (t) => (t < 2.1 - 1e-9 ? unstable() : {}))));
    expect(a.outcome).toBe("unresolved");
    const b = classifyHumanoidTrajectory(traj(series(4, (t) => (t < 2 - 1e-9 ? unstable() : {}))));
    expect(b.outcome).toBe("standing");
    expect(b.t_outcome_s).toBeCloseTo(2, 9);
  });
});

describe("humanoide: stepped", () => {
  /** Pie izquierdo desplazado `d` m en X a partir de `t0`. */
  const moved = (t0: number, d: number) => (t: number): FrameOptions =>
    (t >= t0 ? { left: [d, 0.09], right: [0, -0.09] } : {});

  it("un pie se desplaza 8 cm y 2 s después sigue estable: stepped, con el pie y el instante del paso", () => {
    const v = classifyHumanoidTrajectory(traj(series(4, moved(1, 0.08))));
    expect(v.outcome).toBe("stepped");
    expect(v.step).toEqual({ t_s: 1, foot: "left" });
    expect(v.t_outcome_s).toBeCloseTo(1, 9);
    expect(v.balance.max_foot_displacement_m.left).toBeCloseTo(0.08, 12);
    expect(v.fall).toBeNull();
  });

  it("el umbral es 5 cm: 4,9 cm es standing, 5,1 cm es stepped (el pie derecho también cuenta)", () => {
    expect(classifyHumanoidTrajectory(traj(series(4, moved(1, 0.049)))).outcome).toBe("standing");
    expect(classifyHumanoidTrajectory(traj(series(4, moved(1, 0.051)))).outcome).toBe("stepped");
    const right = classifyHumanoidTrajectory(traj(series(4, (t) => (t >= 1 ? { right: [-0.06, -0.09] } : {}))));
    expect(right.outcome).toBe("stepped");
    expect(right.step!.foot).toBe("right");
  });

  it("el desplazamiento cuenta aunque el pie vuelva a su sitio", () => {
    const v = classifyHumanoidTrajectory(traj(series(4, (t) => (t >= 1 && t < 1.5 ? { left: [0.1, 0.09] } : {}))));
    expect(v.outcome).toBe("stepped");
    expect(v.balance.max_foot_displacement_m.left).toBeCloseTo(0.1, 12);
  });

  it("la inestabilidad durante el paso se tolera si después se recupera; t_outcome es cuando se estabiliza", () => {
    const v = classifyHumanoidTrajectory(traj(series(5, (t) =>
      t < 1.5 ? { xcom: [0.4, 0], left: t >= 1 ? [0.1, 0.09] : [0, 0.09] } : { left: [0.1, 0.09] })));
    expect(v.outcome).toBe("stepped");
    expect(v.t_outcome_s).toBeCloseTo(1.5, 9);
    expect(v.unbalance!.direction).toBe("front");
  });

  it("el paso ocurre menos de 2 s antes del final: unresolved", () => {
    expect(classifyHumanoidTrajectory(traj(series(3, moved(1.5, 0.08)))).outcome).toBe("unresolved");
    expect(classifyHumanoidTrajectory(traj(series(3, moved(1, 0.08)))).outcome).toBe("stepped");
  });

  it("paso pero sin estabilidad al final: unresolved", () => {
    const v = classifyHumanoidTrajectory(traj(series(5, (t) => ({ left: t >= 1 ? [0.1, 0.09] : [0, 0.09], speed: t > 4.5 ? 0.5 : 0 }))));
    expect(v.outcome).toBe("unresolved");
    expect(v.step).not.toBeNull();
  });

  it("sin pie cargado (polígono vacío) no hay estabilidad: unresolved, desequilibrio con margen nulo", () => {
    const v = classifyHumanoidTrajectory(traj(series(4, (t) => (t >= 1 ? { polygon: [] } : {}))));
    expect(v.outcome).toBe("unresolved");
    expect(v.unbalance!.t_s).toBeCloseTo(1, 9);
    expect(v.balance.final_capture_margin_m).toBeNull();
  });
});

describe("humanoide: fall", () => {
  const standing = series(4, () => ({}));

  it.each(["head", "chest", "abdomen", "pelvis"])("el contacto de «%s» con el suelo es una caída", (body) => {
    const v = classifyHumanoidTrajectory(traj(standing, [touch(body, 2.5)]));
    expect(v.outcome).toBe("fall");
    expect(v.t_outcome_s).toBe(2.5);
    expect(v.fall).toMatchObject({ t_s: 2.5, trigger: `contact:${body}`, first_part: body });
  });

  it("manos, muslos, espinillas y pies que tocan el suelo no son una caída", () => {
    const parts = ["left_forearm", "right_forearm", "left_upperarm", "left_thigh", "right_shin", "left_foot"];
    const v = classifyHumanoidTrajectory(traj(standing, parts.map((p, i) => touch(p, 1 + 0.1 * i))));
    expect(v.outcome).toBe("standing");
    expect(v.fall).toBeNull();
  });

  it("la pelvis por debajo del 50 % de su altura inicial es una caída; justo por encima, no", () => {
    const low = (z: number) => series(4, (t) => ({ pelvisZ: t >= 2 ? z : PELVIS_Z0 }));
    const fall = classifyHumanoidTrajectory(traj(low(0.49 * PELVIS_Z0)));
    expect(fall.outcome).toBe("fall");
    expect(fall.fall).toMatchObject({ t_s: 2, trigger: "pelvis_height" });
    expect(classifyHumanoidTrajectory(traj(low(0.51 * PELVIS_Z0))).outcome).toBe("standing");
  });

  it("la caída es irreversible: una pelvis que baja y se recupera sigue siendo caída", () => {
    const v = classifyHumanoidTrajectory(traj(series(4, (t) => ({ pelvisZ: t >= 1 && t < 1.5 ? 0.3 : PELVIS_Z0 }))));
    expect(v.outcome).toBe("fall");
    expect(v.t_outcome_s).toBeCloseTo(1, 9);
  });

  it("gana lo que ocurre antes: contacto antes que la altura de la pelvis, o al revés", () => {
    const lowFrom = (t0: number) => series(4, (t) => ({ pelvisZ: t >= t0 ? 0.2 : PELVIS_Z0 }));
    const a = classifyHumanoidTrajectory(traj(lowFrom(2), [touch("chest", 1.5)]));
    expect(a.fall).toMatchObject({ t_s: 1.5, trigger: "contact:chest" });
    const b = classifyHumanoidTrajectory(traj(lowFrom(1), [touch("chest", 1.5)]));
    expect(b.fall).toMatchObject({ t_s: 1, trigger: "pelvis_height" });
  });

  it("primera parte que toca y orden de contactos: ordenados por tiempo, sin pies", () => {
    const v = classifyHumanoidTrajectory(traj(standing, [
      touch("pelvis", 2.2), touch("left_foot", 0.1), touch("right_forearm", 2.0), touch("head", 2.6), touch("chest", 2.3),
    ]));
    expect(v.fall!.first_part).toBe("right_forearm");
    expect(v.fall!.first_part_t_s).toBe(2);
    expect(v.fall!.contact_order.map((c) => c.body)).toEqual(["right_forearm", "pelvis", "chest", "head"]);
    // La caída es la del primer contacto de cabeza, pecho, abdomen o pelvis (no el de la mano).
    expect(v.fall!.t_s).toBe(2.2);
  });

  it("velocidad de impacto de cabeza, tronco y pelvis: módulo y componente hacia el suelo; null si no tocan", () => {
    const v = classifyHumanoidTrajectory(traj(standing, [
      touch("head", 2.6, [-3, 0, -4]), touch("pelvis", 2.2, [0, 0, -1.5]), touch("chest", 2.3, [0, 0.5, 0.5]),
    ]));
    expect(v.fall!.impact.head).toEqual({ t_s: 2.6, speed_mps: 5, normal_speed_mps: 4 });
    expect(v.fall!.impact.pelvis).toEqual({ t_s: 2.2, speed_mps: 1.5, normal_speed_mps: 1.5 });
    expect(v.fall!.impact.chest!.normal_speed_mps).toBe(0); // se aleja del suelo: componente normal 0, no negativa
    const noHead = classifyHumanoidTrajectory(traj(standing, [touch("pelvis", 2.2)]));
    expect(noHead.fall!.impact.head).toBeNull();
    expect(noHead.fall!.impact.chest).toBeNull();
  });

  it("dirección de la caída: la del tronco desde el instante de caída, relativa al frente inicial de Uke", () => {
    const frames = series(4, (t) => ({ trunkDir: t >= 2 ? 90 : null }));
    const v = classifyHumanoidTrajectory(traj(frames, [touch("pelvis", 2.2)]));
    expect(v.fall!.direction_world_deg).toBe(90);
    expect(v.fall!.direction_deg).toBe(90);
    expect(v.fall!.direction_name).toBe("left");
    // Uke mira a +Y (frente a 90° del mundo): caer hacia +Y es caer hacia delante.
    const facingY = classifyHumanoidTrajectory(traj(frames, [touch("pelvis", 2.2)], Math.PI / 2));
    expect(facingY.fall!.direction_deg).toBeCloseTo(0, 9);
    expect(facingY.fall!.direction_name).toBe("front");
    // Hacia atrás (mundo -X con frente +X) en la otra rama del ángulo.
    const back = classifyHumanoidTrajectory(traj(series(4, () => ({ trunkDir: -170 })), [touch("pelvis", 2.2)]));
    expect(back.fall!.direction_deg).toBe(-170);
    expect(back.fall!.direction_name).toBe("back");
  });

  it("si en el instante de caída el tronco sigue recto, se toma la primera muestra posterior con dirección", () => {
    const frames = series(4, (t) => ({ trunkDir: t >= 2.5 ? 180 : null }));
    const v = classifyHumanoidTrajectory(traj(frames, [touch("pelvis", 2.0)]));
    expect(v.fall!.direction_world_deg).toBe(180);
    // Direcciones anteriores a la caída no valen.
    const before = series(4, (t) => ({ trunkDir: t < 1 ? 45 : null }));
    expect(classifyHumanoidTrajectory(traj(before, [touch("pelvis", 2.0)])).fall!.direction_deg).toBeNull();
  });

  it("una caída tiene prioridad sobre un paso y un desequilibrio, que se siguen informando", () => {
    const frames = series(4, (t) => ({
      left: t >= 1 ? [0.2, 0.09] : [0, 0.09], xcom: t >= 0.5 ? [0.5, 0] : [0, 0], trunkDir: 0, pelvisZ: t >= 3 ? 0.2 : PELVIS_Z0,
    }));
    const v = classifyHumanoidTrajectory(traj(frames));
    expect(v.outcome).toBe("fall");
    expect(v.step).toEqual({ t_s: 1, foot: "left" });
    expect(v.unbalance!.t_s).toBeCloseTo(0.5, 9);
  });
});

describe("humanoide: desequilibrio (el punto de captura sale del apoyo)", () => {
  const exitAt = (xcom: XY, heading = 0) => classifyHumanoidTrajectory(traj(
    series(3, (t) => (t >= 1 ? { xcom } : {})), [], heading)).unbalance!;

  // Centro del apoyo de pie: (0,064, 0). Cada punto sale por un lado distinto.
  it.each<[string, XY, number, string]>([
    ["delante", [0.5, 0], 0, "front"],
    ["delante-izquierda", [0.5, 0.5], 45, "front_left"],
    ["izquierda", [0.064, 0.5], 90, "left"],
    ["atrás-izquierda", [-0.4, 0.5], 135, "back_left"],
    ["atrás", [-0.5, 0], 180, "back"],
    ["atrás-derecha", [-0.4, -0.5], -135, "back_right"],
    ["derecha", [0.064, -0.5], -90, "right"],
    ["delante-derecha", [0.5, -0.5], -45, "front_right"],
  ])("sale por %s", (_name, xcom, deg, name) => {
    const u = exitAt(xcom);
    expect(u.t_s).toBeCloseTo(1, 9);
    expect(u.direction_deg).toBe(deg);
    expect(u.direction).toBe(name);
  });

  it("la dirección es relativa al frente inicial de Uke", () => {
    // Uke mira a -X (180°): salir por +X es salir por detrás.
    expect(exitAt([0.5, 0], Math.PI).direction).toBe("back");
  });

  it("el instante es la primera muestra con margen negativo, no el máximo ni el final", () => {
    const v = classifyHumanoidTrajectory(traj(series(4, (t) => (t >= 1.2 && t < 1.6 ? { xcom: [-0.2, 0] } : t >= 2 ? { xcom: [0.3, 0] } : {}))));
    expect(v.unbalance!.t_s).toBeCloseTo(1.2, 9);
    expect(v.unbalance!.direction).toBe("back");
    expect(v.unbalance!.min_margin_m).toBeCloseTo(-0.13, 9);
  });

  it("al levantarse un talón el apoyo se reduce a una línea: la dirección se mide desde el apoyo anterior, no desde la línea", () => {
    // El punto de captura avanza a x = 0,15 (dentro del pie entero) y, en el mismo instante, el apoyo queda en el borde delantero (x = 0,198).
    const toeLine: XY[] = [[0.198, -0.16], [0.198, 0.16]];
    const v = classifyHumanoidTrajectory(traj(series(3, (t) => (t >= 1 ? { xcom: [0.15, 0], polygon: toeLine } : {}))));
    expect(v.unbalance!.t_s).toBeCloseTo(1, 9);
    expect(v.unbalance!.direction).toBe("front");
    expect(v.unbalance!.min_margin_m).toBeCloseTo(-0.048, 9);
  });

  it("sin ningún pie cargado cuenta como fuera del apoyo", () => {
    const v = classifyHumanoidTrajectory(traj(series(3, (t) => (t >= 1 ? { polygon: [], xcom: [0.2, 0] } : {}))));
    expect(v.unbalance!.t_s).toBeCloseTo(1, 9);
  });
});

describe("humanoide: entradas", () => {
  it("sin muestras es un error", () => {
    expect(() => classifyHumanoidTrajectory(traj([]))).toThrow(RangeError);
  });

  it("una sola muestra no puede resolver nada: unresolved", () => {
    expect(classifyHumanoidTrajectory(traj([frame(0)])).outcome).toBe("unresolved");
  });
});
