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
    # Actor de varios cuerpos (humanoide): la altura es la de todo el árbol en la postura por defecto, no la de su raíz.
    root = model.body_rootid[body]
    tree = [g for g in range(model.ngeom) if model.body_rootid[model.geom_bodyid[g]] == root]
    if len({int(model.geom_bodyid[g]) for g in tree}) > 1:
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        top = max(float(data.geom_xpos[g][2] + model.geom_rbound[g]) for g in tree)
        best = max(best, top)
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


MIN_ARROW_LEN_M = 0.08              # longitud mínima visible de una flecha
ARROW_RADIUS_M = 0.018              # radio mínimo visible (>= 1,5 cm)
BANNER_AFTER_S = 0.5                # el rótulo persistente sigue 0,5 s tras el último instante activo


def add_arrow(scene, start, vec_m, rgba, width=ARROW_RADIUS_M):
    """Añade una flecha de usuario a la escena (mjv_initGeom + mjv_connector).

    Longitud mínima MIN_ARROW_LEN_M aunque la fuerza sea pequeña.
    """
    if scene.ngeom >= scene.maxgeom:
        return False
    vec = np.asarray(vec_m, dtype=float)
    n = float(np.linalg.norm(vec))
    if 0.0 < n < MIN_ARROW_LEN_M:
        vec = vec * (MIN_ARROW_LEN_M / n)
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.asarray(rgba, dtype=np.float32))
    start = np.asarray(start, dtype=float)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, max(width, ARROW_RADIUS_M), start, start + vec)
    scene.ngeom += 1
    return True


# ---------------------------------------------------------------- equilibrio (humanoide)

COM_RGB = (40, 110, 255)            # centro de masas y su proyección en el suelo
XCOM_RGB = (235, 40, 200)           # punto de captura
SUPPORT_OK_RGB = (30, 170, 70)      # polígono de apoyo con el punto de captura dentro
SUPPORT_OUT_RGB = (245, 110, 20)    # polígono de apoyo con el punto de captura fuera (o sin apoyo)
GROUND_LIFT_M = 0.004               # los dibujos sobre el suelo se elevan 4 mm para no confundirse con él


def balance_of(sample: dict):
    """(com, xcom, polígono, margen) de la muestra, o None si la grabación no trae equilibrio (cilindro, grabaciones antiguas)."""
    try:
        com = np.array(sample["com"], float)
        xcom = np.array(sample["xcom"], float)
    except (KeyError, TypeError, ValueError):
        return None
    if com.shape != (3,) or xcom.shape != (2,):
        return None
    poly = []
    for p in sample.get("support_polygon") or []:
        try:
            poly.append((float(p[0]), float(p[1])))
        except (TypeError, ValueError, IndexError):
            continue
    margin = sample.get("capture_margin_m")
    return com, xcom, poly, (float(margin) if isinstance(margin, (int, float)) else None)


def draw_balance(img: Image.Image, sample: dict, cam, width: int, height: int, fovy: float) -> str | None:
    """Dibuja sobre `img` (RGBA) el polígono de apoyo, el punto de captura y el centro de masas con su proyección.

    Es una capa 2D proyectada desde la cámara de la escena (siempre visible, aunque el cuerpo tape el punto).
    Devuelve el texto del margen para el rótulo, o None si la muestra no trae equilibrio.
    """
    b = balance_of(sample)
    if b is None:
        return None
    com, xcom, poly, margin = b
    inside = margin is not None and margin >= 0.0
    support_rgb = SUPPORT_OK_RGB if inside else SUPPORT_OUT_RGB

    def px(x, y, z):
        return project(np.array([x, y, z]), cam, width, height, fovy)

    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    r = max(6, width // 110)

    pts = [px(x, y, GROUND_LIFT_M) for x, y in poly]
    if len(pts) >= 3 and all(p is not None for p in pts):
        d.polygon(pts, fill=support_rgb + (70,))
    if len(pts) >= 2 and all(p is not None for p in pts):
        d.line(pts + ([pts[0]] if len(pts) >= 3 else []), fill=support_rgb + (255,), width=max(3, r // 2))
    elif len(pts) == 1 and pts[0] is not None:
        d.ellipse([pts[0][0] - r, pts[0][1] - r, pts[0][0] + r, pts[0][1] + r], outline=support_rgb + (255,), width=3)

    ground_com = px(com[0], com[1], GROUND_LIFT_M)
    com_px = px(com[0], com[1], com[2])
    xi_px = px(xcom[0], xcom[1], GROUND_LIFT_M)
    if com_px is not None and ground_com is not None:       # plomada del centro de masas al suelo
        d.line([com_px, ground_com], fill=COM_RGB + (200,), width=2)
    if ground_com is not None and xi_px is not None:        # de la proyección del centro de masas al punto de captura
        d.line([ground_com, xi_px], fill=XCOM_RGB + (230,), width=3)
    if ground_com is not None:
        d.ellipse([ground_com[0] - r // 2, ground_com[1] - r // 2, ground_com[0] + r // 2, ground_com[1] + r // 2],
                  outline=COM_RGB + (255,), width=2)
    if xi_px is not None:
        d.ellipse([xi_px[0] - r, xi_px[1] - r, xi_px[0] + r, xi_px[1] + r], fill=XCOM_RGB + (255,),
                  outline=(255, 255, 255, 255), width=2)
    if com_px is not None:
        d.ellipse([com_px[0] - r, com_px[1] - r, com_px[0] + r, com_px[1] + r], fill=COM_RGB + (255,),
                  outline=(255, 255, 255, 255), width=2)
    img.alpha_composite(layer)
    if margin is None:
        return "sin apoyo"
    return f"margen de xi: {margin:+.3f} m".replace(".", ",")


def effector_color(kind: str | None, index: int):
    if kind in EFFECTOR_COLORS:
        return EFFECTOR_COLORS[kind]
    if kind is None:
        return EFFECTOR_COLORS["prescribed_force"]
    return FALLBACK_EFFECTOR_COLORS[sum(ord(c) for c in kind) % len(FALLBACK_EFFECTOR_COLORS)]


def brighten_mjcf(mjcf: str) -> str:
    """Inyecta cielo claro, suelo claro con cuadrícula y una luz direccional.

    Solo afecta al renderizado: no se modifica la grabación. Si el MJCF ya define
    un skybox o un material en el suelo, se respeta. Si no se puede analizar, se devuelve igual.
    """
    import xml.etree.ElementTree as ET
    try:
        root = ET.fromstring(mjcf)
    except ET.ParseError:
        return mjcf
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(0, asset)
    if not any(t.get("type") == "skybox" for t in asset.findall("texture")):
        ET.SubElement(asset, "texture", {"name": "hpl_sky", "type": "skybox", "builtin": "gradient",
                                         "rgb1": "0.80 0.90 1.00", "rgb2": "0.97 0.98 1.00",
                                         "width": "256", "height": "256"})
    ET.SubElement(asset, "texture", {"name": "hpl_grid", "type": "2d", "builtin": "checker", "mark": "edge",
                                     "rgb1": "0.90 0.92 0.93", "rgb2": "0.84 0.87 0.89",
                                     "markrgb": "0.45 0.52 0.58", "width": "512", "height": "512"})
    ET.SubElement(asset, "material", {"name": "hpl_floor", "texture": "hpl_grid", "texuniform": "true",
                                      "texrepeat": "1 1", "reflectance": "0"})
    world = root.find("worldbody")
    if world is not None:
        for g in world.iter("geom"):
            if g.get("type") == "plane" and g.get("material") is None:
                g.set("material", "hpl_floor")
                g.set("rgba", "1 1 1 1")
                g.set("size", "30 30 0.1")        # suelo grande: sin borde visible en el horizonte
        ET.SubElement(world, "light", {"name": "hpl_sun", "directional": "true", "pos": "0 0 4",
                                       "dir": "-0.4 -0.3 -1", "diffuse": "0.7 0.7 0.7",
                                       "specular": "0.1 0.1 0.1", "castshadow": "false"})
    return ET.tostring(root, encoding="unicode")


def effector_series(samples):
    """{id: (normas por muestra, kind)}; normas 0 si falta el efector en la muestra."""
    ids = []
    for s in samples:
        for e in s.get("effectors") or []:
            if e.get("id") not in ids:
                ids.append(e.get("id"))
    out = {}
    for eid in ids:
        norms, kind = np.zeros(len(samples)), None
        for i, s in enumerate(samples):
            for e in s.get("effectors") or []:
                if e.get("id") == eid:
                    try:
                        norms[i] = float(np.linalg.norm(np.array(e["force_n"], float)))
                    except (KeyError, TypeError, ValueError):
                        pass
                    kind = kind or e.get("kind")
        out[eid] = (norms, kind)
    return out


def sheet_indices(n: int, peak_indices: list[int], tiles: int = 8) -> list[int]:
    """8 fotogramas: uno en el pico de cada efector (hasta `tiles`) y el resto repartidos."""
    if n <= tiles:
        return list(range(n))
    grid = [round(k * (n - 1) / (tiles - 1)) for k in range(tiles)]
    used = set()
    for p in peak_indices[:tiles]:
        if p in grid:
            used.add(grid.index(p))
            continue
        cand = [j for j in range(tiles) if j not in used and j not in (0, tiles - 1)] or                [j for j in range(tiles) if j not in used]
        j = min(cand, key=lambda j: abs(grid[j] - p))
        grid[j] = p
        used.add(j)
    return sorted(set(grid))


def rgba_to_rgb(rgba):
    # Color del texto: el de la flecha aclarado, para contrastar con el recuadro oscuro.
    return tuple(int(255 * (0.45 * v + 0.55)) for v in rgba[:3])


def draw_label(base: Image.Image, xy, text, font, fg, bg=(20, 24, 30, 170)) -> None:
    """Texto sobre rectángulo semitransparente (se lee sobre cualquier color)."""
    overlay = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    x, y = xy
    w = int(font.getlength(text))
    pad = max(3, font.size // 5)
    d.rectangle([x - pad, y - pad // 2, x + w + pad, y + font.size + pad], fill=bg)
    d.text((x, y), text, font=font, fill=fg + (255,))
    base.alpha_composite(overlay)


# ---------------------------------------------------------------- fotogramas

class FrameRenderer:
    def __init__(self, rec: dict, width: int, height: int, view: str, follow: bool | None = None):
        if view not in VIEWS:
            raise RecordingError(f"vista desconocida {view!r}; usa {sorted(VIEWS)}")
        self.rec = rec
        self.samples = rec["samples"]
        self.model = mujoco.MjModel.from_xml_string(brighten_mjcf(rec["model"]["mjcf"]))
        self.data = mujoco.MjData(self.model)
        nq = self.model.nq
        for i, s in enumerate(self.samples):
            if len(s["qpos"]) != nq:
                raise RecordingError(f"samples[{i}].qpos tiene {len(s['qpos'])} valores y el modelo espera {nq}")
        self.width, self.height = width, height
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        self.model.vis.headlight.ambient[:] = 0.6
        self.model.vis.headlight.diffuse[:] = 0.6
        self.renderer = mujoco.Renderer(self.model, height=height, width=width, max_geom=4000)
        self.body = find_actor_body(self.model)
        self.extent = actor_extent(self.model, self.body)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.azimuth, self.cam.elevation = VIEWS[view]
        self.view = view
        self.font = load_font(max(16, width // 38))
        self.font_big = load_font(max(20, width // 32))
        self.fovy = float(self.model.vis.global_.fovy)
        self.series = effector_series(self.samples)
        self.t = np.array([float(s["t"]) for s in self.samples])
        self._plan_camera(follow)

    # -- cámara

    def _view_axes(self):
        az, el = math.radians(self.cam.azimuth), math.radians(self.cam.elevation)
        fwd = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        up = np.array([0.0, 0.0, 1.0])
        up = up - fwd * (up @ fwd)
        up /= np.linalg.norm(up)
        return fwd, up, np.cross(fwd, up)

    def _plan_camera(self, follow):
        """Encuadre inicial sobre toda la grabación; sigue al actor solo si no cabe."""
        pts = []
        for s in self.samples:
            self.data.qpos[:] = s["qpos"]
            mujoco.mj_kinematics(self.model, self.data)
            pts.append(self.data.xpos[self.body].copy())
        pts = np.array(pts)
        r = 0.5 * self.extent
        lo, hi = pts.min(axis=0) - r, pts.max(axis=0) + r
        lo[2], hi[2] = 0.0, max(hi[2], self.extent)
        for g in range(self.model.ngeom):         # elementos fijos (p. ej. el obstáculo) también deben verse
            if self.model.body_weldid[self.model.geom_bodyid[g]] == 0 and self.model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE:
                self.data.qpos[:] = self.samples[0]["qpos"]
                mujoco.mj_kinematics(self.model, self.data)
                c, s = self.data.geom_xpos[g], float(np.max(self.model.geom_size[g]))
                lo, hi = np.minimum(lo, c - s), np.maximum(hi, c + s)
        center = (lo + hi) / 2
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        fwd, up, right = self._view_axes()
        self.base_dist = max(2.4, 1.9 * self.extent)
        tan_y = math.tan(math.radians(self.fovy) / 2)
        tan_x = tan_y * self.width / self.height
        need = 0.0
        for c in corners - center:
            # a distancia d, la esquina está a profundidad d + c·fwd; exige |c·right| <= 0.9 tan_x · prof
            for off, lim in ((abs(c @ right), tan_x), (abs(c @ up), tan_y)):
                prof_needed = off / (0.9 * lim)
                need = max(need, prof_needed - float(c @ fwd))
        self.need_dist = max(self.base_dist, need)
        fits = self.need_dist <= 1.8 * self.base_dist
        self.follow = (not fits) if follow is None else follow
        self.static_center = center
        self.cam.distance = self.base_dist if self.follow else self.need_dist

    # -- contenido

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

    def banner_lines(self, index: int) -> list[tuple[str, tuple]]:
        """Rótulos persistentes: efector activo y hasta 0,5 s después."""
        lines = []
        for k, (eid, (norms, kind)) in enumerate(self.series.items()):
            active = np.nonzero(norms[: index + 1] >= MIN_ARROW_N)[0]
            if active.size == 0 or self.t[index] - self.t[active[-1]] > BANNER_AFTER_S:
                continue
            peak = float(norms.max())
            lines.append((f"{eid}: {norms[index]:.0f} N (pico {peak:.0f} N)", rgba_to_rgb(effector_color(kind, k))))
        return lines

    def render(self, index: int, caption: str = "") -> tuple[np.ndarray, dict]:
        """Renderiza la muestra `index`. Devuelve (imagen RGB uint8, información de depuración)."""
        sample = self.samples[index]
        self.data.qpos[:] = sample["qpos"]
        if "qvel" in sample and len(sample["qvel"]) == self.model.nv:
            self.data.qvel[:] = sample["qvel"]
        mujoco.mj_forward(self.model, self.data)          # NUNCA mj_step

        if self.follow:
            c = self.data.xpos[self.body]
            self.cam.lookat[:] = [c[0], c[1], min(max(c[2], 0.45), 0.9)]
        else:
            self.cam.lookat[:] = self.static_center
        self.renderer.update_scene(self.data, camera=self.cam)
        scene = self.renderer.scene
        arrows = self._arrows(sample)
        for start, vec, rgba, _ in arrows:
            add_arrow(scene, start, vec, rgba)
        img = Image.fromarray(self.renderer.render().copy()).convert("RGBA")

        cam = scene.camera[0]
        margin_text = draw_balance(img, sample, cam, self.width, self.height, self.fovy)
        placed = []
        for start, vec, rgba, text in arrows:
            n = float(np.linalg.norm(vec))
            if 0.0 < n < MIN_ARROW_LEN_M:
                vec = vec * (MIN_ARROW_LEN_M / n)
            px = project(start + vec, cam, self.width, self.height, self.fovy)
            if px is None:
                continue
            x, y = px[0] + 8, px[1] - self.font.size
            if any(abs(x - ox) < 90 and abs(y - oy) < self.font.size * 1.3 for ox, oy in placed):
                continue                      # evita etiquetas apiladas (contactos muy juntos)
            placed.append((x, y))
            draw_label(img, (x, y), text, self.font, rgba_to_rgb(rgba))
        label = f"t = {sample['t']:.3f} s"
        if caption:
            label += f"   {caption}"
        draw_label(img, (14, 10), label, self.font_big, (255, 255, 255), bg=(20, 24, 30, 200))
        # banda inferior persistente de efectores
        y = self.height - 10 - self.font_big.size
        for text, col in reversed(self.banner_lines(index)):
            draw_label(img, (14, y), text, self.font_big, col, bg=(20, 24, 30, 200))
            y -= int(self.font_big.size * 1.5)
        legend = "1 m de flecha = 1000 N"
        draw_label(img, (self.width - int(self.font.getlength(legend)) - 14, 10), legend, self.font,
                   (235, 235, 235), bg=(20, 24, 30, 170))
        if margin_text is not None:
            # leyenda del equilibrio abajo a la derecha; el color del margen es el del polígono
            inside = balance_of(sample)[3] is not None and balance_of(sample)[3] >= 0.0
            rows = [("centro de masas", COM_RGB), ("punto de captura (xi)", XCOM_RGB),
                    (margin_text, SUPPORT_OK_RGB if inside else SUPPORT_OUT_RGB)]
            y = self.height - 10 - self.font.size
            for text, col in reversed(rows):
                light = tuple(int(255 * (0.35 * v / 255 + 0.65)) for v in col)
                draw_label(img, (self.width - int(self.font.getlength(text)) - 14, y), text, self.font, light)
                y -= int(self.font.size * 1.5)
        info = {"xpos": self.data.xpos[self.body].copy(), "arrows": len(arrows), "follow": self.follow,
                "banner": [t for t, _ in self.banner_lines(index)], "balance": margin_text}
        return np.asarray(img.convert("RGB")), info

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


def iter_video_frames(rec: dict, view="side", size=(960, 540), fps=30, slow=0.25, follow=None) -> Iterator[tuple[np.ndarray, dict]]:
    fr = FrameRenderer(rec, size[0], size[1], view, follow)
    try:
        for idx, caption in frame_plan(rec, fps, slow):
            yield fr.render(idx, caption)
    finally:
        fr.close()


def write_video(rec: dict, out: Path, view, size, fps, slow, follow=None) -> int:
    import imageio_ffmpeg
    out.parent.mkdir(parents=True, exist_ok=True)
    w, h = size
    writer = imageio_ffmpeg.write_frames(
        str(out), (w, h), fps=fps, codec="libx264", pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
        quality=7, macro_block_size=2, output_params=["-movflags", "+faststart"], ffmpeg_log_level="error")
    writer.send(None)
    n = 0
    try:
        for frame, _ in iter_video_frames(rec, view, size, fps, slow, follow):
            writer.send(np.ascontiguousarray(frame).tobytes())
            n += 1
    finally:
        writer.close()
    return n


def write_sheet(rec: dict, out: Path, view, size, tiles=8, cols=4, tile_w=640, follow=None) -> None:
    fr = FrameRenderer(rec, size[0], size[1], view, follow)
    try:
        n = len(fr.samples)
        peaks = [int(np.argmax(norms)) for norms, _ in fr.series.values() if norms.max() >= MIN_ARROW_N]
        idxs = sheet_indices(n, peaks, tiles)
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
    p.add_argument("--follow", action=argparse.BooleanOptionalAction, default=None,
                   help="seguir al actor; por defecto solo si su movimiento no cabe en el encuadre inicial")
    p.add_argument("--sheet", type=Path, default=None, help="PNG con 8 fotogramas clave y su tiempo")
    a = p.parse_args(argv)
    if a.fps < 1 or a.slow < 0:
        p.error("--fps debe ser >= 1 y --slow >= 0")
    try:
        rec = load_recording(a.recording)
        n = write_video(rec, a.out, a.view, a.size, a.fps, a.slow, a.follow)
        print(f"{a.out}: {n} fotogramas a {a.fps} fps, {a.size[0]}x{a.size[1]}, vista {a.view}")
        if a.sheet:
            write_sheet(rec, a.sheet, a.view, a.size, follow=a.follow)
            print(f"{a.sheet}: hoja de 8 fotogramas")
    except RecordingError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
