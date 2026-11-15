"""Smoke and integrity checks for synchronized physical comparisons."""
import copy
import json
import tempfile
from pathlib import Path
import imageio_ffmpeg
import numpy as np
import compare_fall_recordings as comparison
import make_sample_recording as sample


def main():
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp)
        rec=sample.make_recording(duration_s=.2)
        a,b=root/"before.json",root/"after.json"
        a.write_text(json.dumps(rec),encoding="utf8")
        b.write_text(json.dumps(rec),encoding="utf8")
        original=comparison.rr.mujoco.mj_step
        def forbidden(*args):
            raise AssertionError("The comparison must never advance physics")
        comparison.rr.mujoco.mj_step=forbidden
        try:
            result=comparison.compare(a,b,root/"compare.mp4",size=(320,180),fps=5,slow=0,sheet=root/"sheet.png")
        finally:
            comparison.rr.mujoco.mj_step=original
        reader=imageio_ffmpeg.read_frames(str(root/"compare.mp4"))
        meta=next(reader)
        frames=list(reader)
        assert len(frames)==result["frames"]==2
        assert tuple(meta["size"])==(640,180)
        assert all(np.frombuffer(frame,dtype=np.uint8).mean()>10 for frame in frames)
        assert (root/"sheet.png").exists()
        print("ok: synchronized render without mj_step, shared camera, video and sheet")
        for kind in ("scenario","model","times"):
            invalid=copy.deepcopy(rec)
            if kind=="scenario":
                invalid["meta"]["scenario_sha256"]="different"
            elif kind=="model":
                invalid["model"]["mjcf"]+="\n<!-- different -->"
            else:
                invalid["samples"][-1]["t"]+=.001
            b.write_text(json.dumps(invalid),encoding="utf8")
            try:
                comparison.compare(a,b,root/"invalid.mp4",size=(320,180),fps=5,slow=0)
            except ValueError:
                print("ok: rejected mismatched "+kind)
            else:
                raise AssertionError("Accepted mismatched "+kind)
    return 0

if __name__=="__main__":
    raise SystemExit(main())
