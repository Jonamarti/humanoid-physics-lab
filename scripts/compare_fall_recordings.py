#!/usr/bin/env python3
"""Render a synchronized before/after comparison without advancing physics."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import imageio_ffmpeg
import numpy as np
from PIL import Image
import render_recording as rr


def compare(before_path, after_path, out, view="side", size=(960, 540), fps=24, slow=.25, sheet=None):
    before, after = rr.load_recording(Path(before_path)), rr.load_recording(Path(after_path))
    if before["meta"]["scenario_sha256"] != after["meta"]["scenario_sha256"]:
        raise ValueError("Before and after must use exactly the same scenario")
    if before["model"]["mjcf"] != after["model"]["mjcf"]:
        raise ValueError("The comparison must preserve the model, contacts and anatomical limits")
    if [s["t"] for s in before["samples"]] != [s["t"] for s in after["samples"]]:
        raise ValueError("Sample times do not match")
    renderers = [rr.FrameRenderer(rec, size[0], size[1], view, False) for rec in (before, after)]
    # Enclose both independently computed framing volumes in a common camera.
    center = sum((r.static_center for r in renderers)) / 2
    distance = max(r.need_dist + np.linalg.norm(r.static_center-center) / np.sin(np.deg2rad(r.fovy/2)) for r in renderers)
    for r in renderers:
        r.static_center = center.copy()
        r.cam.distance = distance
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio_ffmpeg.write_frames(str(out), (2*size[0], size[1]), fps=fps, codec="libx264", pix_fmt_in="rgb24", pix_fmt_out="yuv420p", quality=7, macro_block_size=2, output_params=["-movflags", "+faststart"], ffmpeg_log_level="error")
    writer.send(None)
    plan = rr.frame_plan(before, fps, slow)
    peaks = [int(np.argmin(np.abs(r.t - rec["diagnostics"]["contact_force"]["t_s"]))) for r,rec in zip(renderers,(before,after)) if rec.get("diagnostics",{}).get("contact_force")]
    sheet_indices = rr.sheet_indices(len(before["samples"]), peaks, 8)
    tiles = {}
    try:
        for index, caption in plan:
            frames = [r.render(index, label+" | "+caption)[0] for r,label in zip(renderers,("ANTES","DESPUES"))]
            joined = np.concatenate(frames, axis=1)
            writer.send(np.ascontiguousarray(joined).tobytes())
        if sheet:
            for i in sheet_indices:
                frames = [r.render(i, label)[0] for r,label in zip(renderers,("ANTES","DESPUES"))]
                tiles[i] = Image.fromarray(np.concatenate(frames, axis=1)).resize((1280,360), Image.Resampling.LANCZOS)
            canvas = Image.new("RGB", (2560,4*360))
            for k,i in enumerate(sheet_indices):
                canvas.paste(tiles[i], ((k%2)*1280,(k//2)*360))
            Path(sheet).parent.mkdir(parents=True, exist_ok=True)
            canvas.save(sheet)
    finally:
        writer.close()
        for r in renderers:
            r.close()
    return {"file":str(out),"view":view,"frames":len(plan),"fps":fps,"size":[2*size[0],size[1]],"scenario_sha256":before["meta"]["scenario_sha256"],"same_model":True,"same_times":True,"camera_center":center.tolist(),"camera_distance":float(distance)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("before",type=Path)
    p.add_argument("after",type=Path)
    p.add_argument("out",type=Path)
    p.add_argument("--view",choices=sorted(rr.VIEWS),default="side")
    p.add_argument("--sheet",type=Path)
    a=p.parse_args()
    result=compare(a.before,a.after,a.out,a.view,sheet=a.sheet)
    a.out.with_suffix(".validation.json").write_text(json.dumps(result,indent=2),encoding="utf8")
    print(json.dumps(result))

if __name__=="__main__":
    main()
