#!/usr/bin/env python3
"""Pruebas de `render_recording.py`. Se ejecutan con pytest o como script:

    uv run --quiet --python 3.12 --with mujoco --with numpy --with imageio-ffmpeg --with pillow \
        python scripts/test_render_recording.py
"""
from __future__ import annotations

import copy
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_sample_recording as sample  # noqa: E402
import render_recording as rr  # noqa: E402

SIZE = (320, 180)
FPS = 20


def count_frames(path: Path):
    import imageio_ffmpeg
    reader = imageio_ffmpeg.read_frames(str(path))
    meta = next(reader)
    n, total = 0, 0.0
    for frame in reader:
        n += 1
        total += float(np.frombuffer(frame, dtype=np.uint8).mean())
    return n, total / max(n, 1), meta


def test_video_has_expected_frames_and_is_not_black():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        rec_path = tmp / "rec.json.gz"
        sample.write_recording(sample.make_recording(duration_s=1.0), rec_path)
        out, sheet = tmp / "v.mp4", tmp / "sheet.png"
        code = rr.main([str(rec_path), str(out), "--size", "320x180", "--fps", str(FPS),
                        "--sheet", str(sheet)])
        assert code == 0 and out.exists() and sheet.exists()
        rec = rr.load_recording(rec_path)
        expected = len(rr.frame_plan(rec, FPS, 0.25))        # 20 reales + 80 lentos
        n, mean, meta = count_frames(out)
        assert abs(n - expected) <= 1, (n, expected)
        assert mean > 20.0, f"vídeo casi negro: media {mean}"
        assert meta["pix_fmt"].startswith("yuv420p"), meta["pix_fmt"]
        assert meta["codec"].startswith("h264"), meta["codec"]


def test_unknown_schema_is_clear_error(capsys=None):
    rec = sample.make_recording(duration_s=0.2)
    rec["schema"] = "hpl.recording.v9"
    try:
        rr.validate_recording(rec)
    except rr.RecordingError as exc:
        assert "hpl.recording.v9" in str(exc)
    else:
        raise AssertionError("debía fallar con esquema desconocido")


def test_optional_fields_absent():
    rec = sample.make_recording(duration_s=0.3)
    for s in rec["samples"]:
        s.pop("effectors", None)
        s.pop("contacts", None)
        s.pop("qvel", None)
    rec.pop("peaks", None)
    rec.pop("summary", None)
    frames = list(rr.iter_video_frames(rec, "top", SIZE, FPS, 0))
    assert len(frames) == len(rr.frame_plan(rec, FPS, 0)) and frames[0][1]["arrows"] == 0


def test_all_views_render():
    rec = sample.make_recording(duration_s=0.2)
    for view in rr.VIEWS:
        img, _ = next(rr.iter_video_frames(rec, view, SIZE, FPS, 0))
        assert img.shape == (SIZE[1], SIZE[0], 3) and img.mean() > 20


def test_never_simulates_replays_qpos_without_gravity():
    """Modelo sin gravedad con muestras que describen una caída: se debe ver la caída de qpos."""
    mjcf = """<mujoco><option gravity="0 0 0"/><worldbody>
      <geom name="floor" type="plane" size="5 5 0.1"/>
      <body name="ball" pos="0 0 2"><freejoint/><geom type="sphere" size="0.2" mass="1"/></body>
    </worldbody></mujoco>"""
    zs = np.linspace(2.0, 0.2, 31)
    samples = [{"t": round(i / 30, 6), "qpos": [0, 0, float(z), 1, 0, 0, 0], "qvel": [0] * 6}
               for i, z in enumerate(zs)]
    rec = {"schema": "hpl.recording.v1", "meta": {"record_hz": 30}, "model": {"mjcf": mjcf}, "samples": samples}

    real_step = mujoco.mj_step
    def forbidden(*a, **k):
        raise AssertionError("render_recording llamó a mj_step")
    mujoco.mj_step = forbidden
    try:
        frames = list(rr.iter_video_frames(rec, "side", SIZE, 30, 0))
    finally:
        mujoco.mj_step = real_step

    first, last = frames[0], frames[-1]
    assert abs(first[1]["xpos"][2] - 2.0) < 1e-9
    assert abs(last[1]["xpos"][2] - 0.2) < 1e-9, last[1]["xpos"]   # según qpos, no según física
    # Sin gravedad, la bola habría seguido a 2 m; la imagen debe cambiar de verdad.
    assert np.abs(first[0].astype(int) - last[0].astype(int)).mean() > 0.5


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"ok    {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"FALLA {name}: {type(exc).__name__}: {exc}")
    sys.exit(1 if failed else 0)
