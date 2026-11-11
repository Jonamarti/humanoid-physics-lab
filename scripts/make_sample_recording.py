#!/usr/bin/env python3
"""Genera una grabación de ejemplo `hpl.recording.v1` con MuJoCo de Python.

Es un dato de prueba para `render_recording.py` mientras el ejecutor TypeScript
(`npm run sim`) produce las grabaciones reales. Cumple el formato de la issue #74
(PR 3): `meta`, `model.mjcf`, `samples[].t/qpos/qvel/effectors/contacts`,
`peaks` y `summary`.

Escenario: cilindro rígido de 75 kg, 1,75 m y radio 0,18 m, con un obstáculo
fijo en forma de caja y una fuerza de medio seno aplicada a 1,40 m de altura.

Uso:
    uv run --quiet --python 3.12 --with mujoco --with numpy \
        python scripts/make_sample_recording.py out/sample.json.gz
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

PHYSICS_HZ = 500
TIMESTEP_S = 1.0 / PHYSICS_HZ
SETTLE_S = 0.5

MASS_KG = 75.0
HEIGHT_M = 1.75
RADIUS_M = 0.18
GRIP_HEIGHT_M = 1.40
FRICTION = 0.6


def sig6(x):
    """Redondea a 6 cifras significativas (recursivo sobre listas)."""
    if isinstance(x, (list, tuple, np.ndarray)):
        return [sig6(v) for v in x]
    x = float(x)
    if x == 0.0 or not math.isfinite(x):
        return 0.0 if not math.isfinite(x) else x
    return float(f"{x:.6g}")


def build_mjcf(obstacle_height_m: float) -> str:
    half_h = obstacle_height_m / 2
    return f"""<mujoco model="cylinder_obstacle_sample">
  <compiler angle="radian"/>
  <option timestep="{TIMESTEP_S}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.23 0.28 0.32 1" friction="{FRICTION} 0.005 0.0001"/>
    <body name="obstacle" pos="0.26 0 {half_h}">
      <geom name="obstacle_box" type="box" size="0.06 0.4 {half_h}" rgba="0.68 0.43 0.24 1" friction="{FRICTION} 0.005 0.0001"/>
    </body>
    <body name="cylinder" pos="0 0 {HEIGHT_M / 2 + 0.001}">
      <freejoint name="free_cylinder"/>
      <geom name="cylinder_geom" type="cylinder" size="{RADIUS_M} {HEIGHT_M / 2}" mass="{MASS_KG}" rgba="0.74 0.83 0.88 1" friction="{FRICTION} 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>"""


def make_recording(obstacle_height_m=0.10, force_n=(1100.0, 0.0, 330.0),
                   start_s=0.3, pulse_s=0.15, duration_s=3.0, record_hz=125) -> dict:
    mjcf = build_mjcf(obstacle_height_m)
    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cylinder")
    local_point = np.array([RADIUS_M, 0.0, GRIP_HEIGHT_M - HEIGHT_M / 2])
    force_peak = np.array(force_n, dtype=float)

    def body_name(geom_id):
        b = model.geom_bodyid[geom_id]
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "world"

    # Asentamiento sin efectores: el tiempo del escenario empieza después.
    for _ in range(int(round(SETTLE_S * PHYSICS_HZ))):
        mujoco.mj_step(model, data)

    every = PHYSICS_HZ // record_hz
    n_steps = int(round(duration_s * PHYSICS_HZ))
    samples = []
    peak_f = 0.0
    impulse = np.zeros(3)
    work = 0.0
    tilt_final = 0.0
    vel6 = np.zeros(6)

    for k in range(n_steps + 1):
        t = k * TIMESTEP_S
        # Fuerza de medio seno
        tau = t - start_s
        f = force_peak * math.sin(math.pi * tau / pulse_s) if 0.0 <= tau <= pulse_s else np.zeros(3)

        data.xfrc_applied[:] = 0.0
        point = data.xpos[body] + data.xmat[body].reshape(3, 3) @ local_point
        if np.any(f):
            r = point - data.xipos[body]
            data.xfrc_applied[body, :3] = f
            data.xfrc_applied[body, 3:] = np.cross(r, f)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, vel6, 0)
            # mj_objectVelocity: [ang, lin] en el centro del cuerpo; v_punto = v + w x r
            v_point = vel6[3:] + np.cross(vel6[:3], point - data.xipos[body])
            peak_f = max(peak_f, float(np.linalg.norm(f)))
            impulse += f * TIMESTEP_S
            work += float(f @ v_point) * TIMESTEP_S

        if k % every == 0:
            contacts = []
            fc = np.zeros(6)
            for i in range(data.ncon):
                c = data.contact[i]
                mujoco.mj_contactForce(model, data, i, fc)
                fw = c.frame.reshape(3, 3).T @ fc[:3]  # marco del contacto -> mundo
                if np.linalg.norm(fw) < 1e-6:
                    continue
                contacts.append({
                    "body_a": body_name(c.geom1), "body_b": body_name(c.geom2),
                    "point_m": sig6(c.pos), "force_n": sig6(fw),
                })
            samples.append({
                "t": sig6(t),
                "qpos": sig6(data.qpos), "qvel": sig6(data.qvel),
                "effectors": [{"id": "pull", "kind": "prescribed_force",
                               "point_m": sig6(point), "force_n": sig6(f)}],
                "contacts": contacts,
            })
        if k < n_steps:
            mujoco.mj_step(model, data)

    # Veredicto mínimo (inclinación del eje del cilindro respecto a la vertical)
    axis_z = data.xmat[body].reshape(3, 3)[2, 2]
    tilt_final = math.degrees(math.acos(max(-1.0, min(1.0, axis_z))))
    outcome = "standing" if tilt_final < 15 else ("fall" if tilt_final > 60 else "tilted")
    fx, fy = data.xpos[body][0], data.xpos[body][1]
    direction = math.degrees(math.atan2(fy, fx))

    rec = {
        "schema": "hpl.recording.v1",
        "meta": {
            "scenario_id": "sample_cylinder_obstacle",
            "scenario_sha256": hashlib.sha256(mjcf.encode()).hexdigest(),
            "engine_version": "sample-python",
            "mujoco_version": mujoco.__version__,
            "physics_hz": PHYSICS_HZ, "record_hz": record_hz,
            "frame": "x_forward_y_left_z_up", "quat_order": "wxyz",
            "generator": "scripts/make_sample_recording.py (dato de prueba, no es una salida de npm run sim)",
        },
        "model": {
            "mjcf": mjcf,
            "bodies": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)],
            "joints": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)],
        },
        "samples": samples,
        "peaks": {"effectors": {"pull": {
            "max_force_n": sig6(peak_f), "impulse_ns": sig6(impulse), "work_j": sig6(work)}}},
        "summary": {"outcome": outcome, "tilt_deg": sig6(tilt_final),
                    "direction_deg": sig6(direction), "expect": "none"},
    }
    return rec


def write_recording(rec: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(rec, separators=(",", ":"))
    if str(path).endswith(".gz"):
        with gzip.open(path, "wt", encoding="utf-8") as fh:
            fh.write(text)
    else:
        path.write_text(text, encoding="utf-8")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("out", type=Path, help="Salida .json o .json.gz")
    p.add_argument("--obstacle-height", type=float, default=0.10, help="Altura del obstáculo en m (0,10)")
    p.add_argument("--force", type=float, default=1100.0, help="Pico de la fuerza horizontal en N")
    p.add_argument("--duration", type=float, default=3.0, help="Duración en s")
    p.add_argument("--record-hz", type=int, default=125)
    a = p.parse_args(argv)
    rec = make_recording(a.obstacle_height, (a.force, 0.0, 0.3 * a.force),
                         duration_s=a.duration, record_hz=a.record_hz)
    write_recording(rec, a.out)
    s = rec["summary"]
    print(f"{a.out}: {len(rec['samples'])} muestras, resultado={s['outcome']}, "
          f"inclinación final={s['tilt_deg']}°, impulso={rec['peaks']['effectors']['pull']['impulse_ns']} N·s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
