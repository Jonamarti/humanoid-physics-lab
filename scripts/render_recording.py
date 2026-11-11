#!/usr/bin/env python3
"""Convierte una grabación `hpl.recording.v1` en un vídeo MP4 y una hoja de fotogramas.

El script NUNCA simula: carga `model.mjcf` de la grabación, pone `qpos` de cada
muestra, llama a `mj_forward` y renderiza sin ventana con `mujoco.Renderer`.

Uso (esta máquina):
    uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow \
        python scripts/render_recording.py recording.json.gz out.mp4 --view side --sheet hoja.png
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import sys
from pathlib import Path
from typing import Iterator

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SUPPORTED_SCHEMA = "hpl.recording.v1"
NEWTONS_PER_METER = 1000.0          # 1 m de flecha = 1000 N
MIN_ARROW_N = 2.0                   # no se dibujan flechas por debajo de este valor
CONTACT_RGBA = (1.0, 0.82, 0.1, 1.0)
EFFECTOR_COLORS = {                 # color fijo por tipo de efector
    "prescribed_force": (0.95, 0.15, 0.15, 1.0),
    "hand": (0.95, 0.35, 0.65, 1.0),
    "ram": (0.95, 0.55, 0.10, 1.0),
    "sweep": (0.65, 0.30, 0.95, 1.0),
}
FALLBACK_EFFECTOR_COLORS = [(0.95, 0.15, 0.15, 1.0), (0.2, 0.8, 0.9, 1.0), (0.6, 0.9, 0.2, 1.0)]
VIEWS = {   # azimut y elevación de la cámara libre de MuJoCo (grados)
    "front": (180.0, -8.0),         # mira hacia -X: ve al actor de frente (el empuje va hacia la cámara)
    "side": (90.0, -8.0),           # mira hacia +Y: +X va hacia la derecha de la imagen
    "top": (90.0, -89.0),
    "three_quarter": (125.0, -20.0),
}


class RecordingError(ValueError):
    """Error de formato de la grabación, con mensaje claro."""


# ---------------------------------------------------------------- carga

def load_recording(path: str | Path) -> dict:
    path = Path(path)
    raw = path.read_bytes()
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    try:
        rec = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RecordingError(f"{path}: no es un JSON válido ({exc})") from exc
    return validate_recording(rec)


def validate_recording(rec: dict) -> dict:
    if not isinstance(rec, dict):
        raise RecordingError("la grabación debe ser un objeto JSON")
    schema = rec.get("schema")
    if schema != SUPPORTED_SCHEMA:
        raise RecordingError(
            f"versión de esquema no soportada: {schema!r} (este script solo entiende {SUPPORTED_SCHEMA!r})")
    mjcf = (rec.get("model") or {}).get("mjcf")
    if not isinstance(mjcf, str) or not mjcf.strip():
        raise RecordingError("falta model.mjcf")
    samples = rec.get("samples")
    if not isinstance(samples, list) or not samples:
        raise RecordingError("samples está vacío o ausente")
    for i, s in enumerate(samples):
        if not isinstance(s, dict) or "t" not in s or "qpos" not in s:
            raise RecordingError(f"samples[{i}] necesita t y qpos")
    return rec


# ---------------------------------------------------------------- cámara y overlay

def find_actor_body(model: mujoco.MjModel) -> int:
    """Primer cuerpo con articulación libre; si no hay, el primer cuerpo no estático."""
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            return int(model.jnt_bodyid[j])
    return 1 if model.nbody > 1 else 0


def actor_extent(model: mujoco.MjModel, body: int) -> float:
    """Dimensión mayor aproximada del actor (altura o diámetro), para encuadrar."""
    best = 0.5
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == body:
            best = max(best, 2.0 * float(np.max(model.geom_size[g])))
    return best


def load_font(size: int):
    for name in ("arial.ttf", "DejaVuSans.ttf", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size)
    except TypeError:
        return ImageFont.load_default()


def project(point, cam, width, height, fovy_deg):
    """Proyecta un punto del mundo a píxeles con el modelo de cámara de la escena."""
    pos = np.asarray(cam.pos, dtype=float)
    fwd = np.asarray(cam.forward, dtype=float)
    up = np.asarray(cam.up, dtype=float)
    right = np.cross(fwd, up)
    d = np.asarray(point, dtype=float) - pos
    zc = float(d @ fwd)
    if zc <= 1e-6:
        return None
    f = (height / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return (width / 2.0 + f * float(d @ right) / zc, height / 2.0 - f * float(d @ up) / zc)


def add_arrow(scene, start, vec_m, rgba, width=0.02):
    """Añade una flecha de usuario a la escena (mjv_initGeom + mjv_connector)."""
    if scene.ngeom >= scene.maxgeom:
        return False
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.asarray(rgba, dtype=np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width,
                         np.asarray(start, dtype=float), np.asarray(start, dtype=float) + np.asarray(vec_m, dtype=float))
    scene.ngeom += 1
    return True


def add_grid(scene, center_xy, step=0.5, half=3.0):
    """Líneas finas en el suelo, para que se perciba el movimiento aunque la cámara siga al actor."""
    cx = round(center_xy[0] / step) * step
    cy = round(center_xy[1] / step) * step
    rgba = np.array([0.55, 0.62, 0.68, 1.0], dtype=np.float32)
    n = int(half / step)
    for i in range(-n, n + 1):
        for horizontal in (True, False):
            if scene.ngeom >= scene.maxgeom:
                return
            g = scene.geoms[scene.ngeom]
            if horizontal:
                size = [half, 0.004, 0.0005]
                pos = [cx, cy + i * step, 0.0012]
            else:
                size = [0.004, half, 0.0005]
                pos = [cx + i * step, cy, 0.0012]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array(size), np.array(pos),
                                np.eye(3).flatten(), rgba)
            scene.ngeom += 1


def effector_color(kind: str | None, index: int):
    if kind in EFFECTOR_COLORS:
        return EFFECTOR_COLORS[kind]
    if kind is None:
        return EFFECTOR_COLORS["prescribed_force"]
    return FALLBACK_EFFECTOR_COLORS[sum(ord(c) for c in kind) % len(FALLBACK_EFFECTOR_COLORS)]


# ---------------------------------------------------------------- fotogramas

class FrameRenderer:
    def __init__(self, rec: dict, width: int, height: int, view: str):
        if view not in VIEWS:
            raise RecordingError(f"vista desconocida {view!r}; usa {sorted(VIEWS)}")
        self.rec = rec
        self.samples = rec["samples"]
        self.model = mujoco.MjModel.from_xml_string(rec["model"]["mjcf"])
        self.data = mujoco.MjData(self.model)
        nq = self.model.nq
        for i, s in enumerate(self.samples):
            if len(s["qpos"]) != nq:
                raise RecordingError(f"samples[{i}].qpos tiene {len(s['qpos'])} valores y el modelo espera {nq}")
        self.width, self.height = width, height
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        self.model.vis.headlight.ambient[:] = 0.45
        self.model.vis.headlight.diffuse[:] = 0.8
        self.renderer = mujoco.Renderer(self.model, height=height, width=width, max_geom=4000)
        self.body = find_actor_body(self.model)
        self.extent = actor_extent(self.model, self.body)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.azimuth, self.cam.elevation = VIEWS[view]
        self.view = view
        self.cam.distance = max(2.4, 1.9 * self.extent)
        self.font = load_font(max(14, width // 48))
        self.font_big = load_font(max(18, width // 34))
        self.fovy = float(self.model.vis.global_.fovy)

    def _arrows(self, sample):
        """Lista de (inicio, vector en metros, rgba, texto) del instante."""
        out = []
        for i, e in enumerate(sample.get("effectors") or []):
            try:
                p, f = np.array(e["point_m"], float), np.array(e["force_n"], float)
            except (KeyError, TypeError, ValueError):
                continue
            n = float(np.linalg.norm(f))
            if n >= MIN_ARROW_N:
                out.append((p, f / NEWTONS_PER_METER, effector_color(e.get("kind"), i),
                            f"{e.get('id', 'ef')} {n:.0f} N"))
        for c in sample.get("contacts") or []:
            try:
                p, f = np.array(c["point_m"], float), np.array(c["force_n"], float)
            except (KeyError, TypeError, ValueError):
                continue
            n = float(np.linalg.norm(f))
            if n >= MIN_ARROW_N:
                out.append((p, f / NEWTONS_PER_METER, CONTACT_RGBA, f"{n:.0f} N"))
        return out

    def render(self, index: int, caption: str = "") -> tuple[np.ndarray, dict]:
        """Renderiza la muestra `index`. Devuelve (imagen RGB uint8, información de depuración)."""
        sample = self.samples[index]
        self.data.qpos[:] = sample["qpos"]
        if "qvel" in sample and len(sample["qvel"]) == self.model.nv:
            self.data.qvel[:] = sample["qvel"]
        mujoco.mj_forward(self.model, self.data)          # NUNCA mj_step

        center = self.data.xpos[self.body].copy()
        self.cam.lookat[:] = [center[0], center[1], min(max(center[2], 0.45), 0.9)]
        self.renderer.update_scene(self.data, camera=self.cam)
        scene = self.renderer.scene
        add_grid(scene, center[:2])
        arrows = self._arrows(sample)
        for start, vec, rgba, _ in arrows:
            add_arrow(scene, start, vec, rgba)
        img = Image.fromarray(self.renderer.render().copy())

        draw = ImageDraw.Draw(img)
        cam = scene.camera[0]
        placed = []
        for start, vec, rgba, text in arrows:
            px = project(start + vec, cam, self.width, self.height, self.fovy)
            if px is None:
                continue
            col = tuple(int(255 * v) for v in rgba[:3])
            x, y = px[0] + 6, px[1] - 6
            if any(abs(x - ox) < 60 and abs(y - oy) < self.font.size for ox, oy in placed):
                continue                      # evita etiquetas apiladas (contactos muy juntos)
            placed.append((x, y))
            draw.text((x + 1, y + 1), text, font=self.font, fill=(0, 0, 0))
            draw.text((x, y), text, font=self.font, fill=col)
        label = f"t = {sample['t']:.3f} s"
        if caption:
            label += f"   {caption}"
        draw.rectangle([6, 6, 6 + int(self.font_big.getlength(label)) + 16, 6 + self.font_big.size + 12],
                       fill=(0, 0, 0))
        draw.text((14, 10), label, font=self.font_big, fill=(255, 255, 255))
        legend = "1 m de flecha = 1000 N   rojo: efector   amarillo: contacto"
        draw.text((10, self.height - self.font.size - 8), legend, font=self.font, fill=(235, 235, 235))
        info = {"xpos": self.data.xpos[self.body].copy(), "arrows": len(arrows)}
        return np.asarray(img), info

    def close(self):
        self.renderer.close()


def nearest_index(samples, t: float, record_hz: float, t0: float) -> int:
    i = int(round((t - t0) * record_hz))
    return min(max(i, 0), len(samples) - 1)


def frame_plan(rec: dict, fps: int, slow: float) -> list[tuple[int, str]]:
    """Plan del vídeo: lista de (índice de muestra, rótulo). Real seguido de lento."""
    samples = rec["samples"]
    record_hz = float((rec.get("meta") or {}).get("record_hz") or 0)
    if record_hz <= 0 and len(samples) > 1:
        record_hz = (len(samples) - 1) / (samples[-1]["t"] - samples[0]["t"])
    record_hz = record_hz or 1.0
    t0 = float(samples[0]["t"])
    dur = float(samples[-1]["t"]) - t0
    plan = []
    n_real = int(math.floor(dur * fps + 1e-9)) + 1          # incluye la última muestra
    for i in range(n_real):
        plan.append((nearest_index(samples, t0 + i / fps, record_hz, t0), "x1 (velocidad real)"))
    if 0.0 < slow < 1.0:
        n_slow = int(math.floor(dur * fps / slow + 1e-9)) + 1
        for i in range(n_slow):
            plan.append((nearest_index(samples, t0 + i * slow / fps, record_hz, t0), f"x{slow:g} (lento)".replace(".", ",")))
    return plan


def iter_video_frames(rec: dict, view="side", size=(960, 540), fps=30, slow=0.25) -> Iterator[tuple[np.ndarray, dict]]:
    fr = FrameRenderer(rec, size[0], size[1], view)
    try:
        for idx, caption in frame_plan(rec, fps, slow):
            yield fr.render(idx, caption)
    finally:
        fr.close()


def write_video(rec: dict, out: Path, view, size, fps, slow) -> int:
    import imageio_ffmpeg
    out.parent.mkdir(parents=True, exist_ok=True)
    w, h = size
    writer = imageio_ffmpeg.write_frames(
        str(out), (w, h), fps=fps, codec="libx264", pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
        quality=7, macro_block_size=2, output_params=["-movflags", "+faststart"], ffmpeg_log_level="error")
    writer.send(None)
    n = 0
    try:
        for frame, _ in iter_video_frames(rec, view, size, fps, slow):
            writer.send(np.ascontiguousarray(frame).tobytes())
            n += 1
    finally:
        writer.close()
    return n


def write_sheet(rec: dict, out: Path, view, size, tiles=8, cols=4, tile_w=640) -> None:
    fr = FrameRenderer(rec, size[0], size[1], view)
    try:
        n = len(fr.samples)
        idxs = sorted({round(k * (n - 1) / (tiles - 1)) for k in range(tiles)}) if n > 1 else [0]
        tile_h = round(tile_w * size[1] / size[0])
        rows = math.ceil(len(idxs) / cols)
        sheet = Image.new("RGB", (cols * tile_w, rows * tile_h), (0, 0, 0))
        for k, idx in enumerate(idxs):
            frame, _ = fr.render(idx)
            tile = Image.fromarray(frame).resize((tile_w, tile_h), Image.LANCZOS)
            sheet.paste(tile, ((k % cols) * tile_w, (k // cols) * tile_h))
        out.parent.mkdir(parents=True, exist_ok=True)
        sheet.save(out)
    finally:
        fr.close()


# ---------------------------------------------------------------- línea de órdenes

def parse_size(text: str) -> tuple[int, int]:
    try:
        w, h = (int(v) for v in text.lower().split("x"))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"tamaño inválido {text!r}; usa 960x540") from exc
    if w < 64 or h < 64 or w % 2 or h % 2:
        raise argparse.ArgumentTypeError("el tamaño debe ser par y de al menos 64x64")
    return w, h


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    p = argparse.ArgumentParser(description="Vídeo MP4 y hoja de fotogramas a partir de una grabación hpl.recording.v1")
    p.add_argument("recording", type=Path, help="grabación .json o .json.gz")
    p.add_argument("out", type=Path, help="vídeo de salida .mp4")
    p.add_argument("--view", choices=sorted(VIEWS), default="side")
    p.add_argument("--slow", type=float, default=0.25, help="factor de la pasada lenta (0,25 = x0,25); 0 la desactiva")
    p.add_argument("--size", type=parse_size, default=(960, 540), help="ancho x alto, p. ej. 960x540")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--sheet", type=Path, default=None, help="PNG con 8 fotogramas clave y su tiempo")
    a = p.parse_args(argv)
    if a.fps < 1 or a.slow < 0:
        p.error("--fps debe ser >= 1 y --slow >= 0")
    try:
        rec = load_recording(a.recording)
        n = write_video(rec, a.out, a.view, a.size, a.fps, a.slow)
        print(f"{a.out}: {n} fotogramas a {a.fps} fps, {a.size[0]}x{a.size[1]}, vista {a.view}")
        if a.sheet:
            write_sheet(rec, a.sheet, a.view, a.size)
            print(f"{a.sheet}: hoja de 8 fotogramas")
    except RecordingError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
