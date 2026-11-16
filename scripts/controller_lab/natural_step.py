"""ProtoMotions natural posture and a normal-length single-step reference."""
from pathlib import Path
import json
from cases import prepare_pack
import benchmark
from compare import rescore
import numpy as np

if __name__=="__main__":
    root=Path(__file__).resolve().parents[2]
    out=root/"out/protomotions-natural-v3"
    prepare_pack(root,out/"references.pt",step_shift=.09,step_lift=.14,
        step_advance=.28,natural_arms=True)
    benchmark.CASES=[
        {"id":"natural_stand","clip":0,"kind":"posture"},
        {"id":"natural_step_left","clip":3,"kind":"step","goal_advance_m":.28},
    ]
    results=benchmark.run(root,"protomotions",None,destination=out,reference_file=out/"references.pt")
    for s in results:
        p=out/"protomotions"/s["case"]
        with np.load(p/"recording.npz") as a:s=rescore(s,a["rows"])
        (p/"summary.json").write_text(json.dumps(s,indent=2),encoding="utf-8")
    print("NATURAL_RESULTS",json.dumps([{k:s[k] for k in ("case","outcome","artifacts","foot_delta_m","supported_clearance","supported_landing")} for s in results]))
