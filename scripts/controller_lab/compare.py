"""Refuse comparisons with different models, references or initial integration state."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import numpy as np

IDENTITY=("case","model_sha256","reference_sha256","initial_state_sha256","mujoco","duration_s")
def validate_pair(left,right):
    for key in IDENTITY:
        if left[key]!=right[key]: raise ValueError("Incomparable "+key)
    if not np.allclose(left["external_impulse_ns"],right["external_impulse_ns"],atol=1e-8,rtol=0):
        raise ValueError("Incomparable external impulse")

def rescore(summary,rows):
    """A foot moving during a fall is not counted as a supported physical step."""
    valid=[False,False]
    land=[False,False]
    mass=summary["mass_kg"]
    for e in summary["events"]:
        i=int(np.argmin(np.abs(rows[:,0]-e["t"])))
        r=rows[i]; foot=e["foot"]
        healthy=r[9]>.7*summary["initial_root_height_m"] and r[10]<np.deg2rad(25) and r[11]==0
        if e["event"]=="clearance_3cm" and healthy and r[7+1-foot]>.6*mass*9.81:
            valid[foot]=True
        if e["event"]=="loaded" and valid[foot] and healthy:
            land[foot]=True
    delta=np.asarray(summary["foot_delta_m"])
    physical=any(valid[i] and land[i] and np.linalg.norm(delta[i,:2])>.06 for i in range(2))
    summary["physical_step"]=bool(physical)
    summary["recovered_with_step"]=bool(summary["kind"]=="recovery" and physical and summary["stable_final_second"] and not summary["artifacts"])
    if summary["kind"]=="step":
        summary["task_achieved"]=bool(summary["stable_final_second"] and not summary["artifacts"] and
            valid[0] and land[0] and summary.get("desired_advance_m",.12)-.05<=delta[0,0]<=summary.get("desired_advance_m",.12)+.05 and abs(delta[0,1])<.05)
        if not summary["task_achieved"] and summary["outcome"]=="complete": summary["outcome"]="incomplete"
    if summary["kind"]=="recovery" and summary["task_achieved"]:
        summary["outcome"]="recovered_with_step" if physical else "recovered_without_step"
    summary["supported_clearance"]=valid
    summary["supported_landing"]=land
    return summary

def collect(root):
    out=root/"out/controller-comparison"
    all_rows=[]
    for controller in ("pd","mjpc","protomotions"):
        for path in sorted((out/controller).glob("*/summary.json")):
            s=json.loads(path.read_text(encoding="utf-8"))
            with np.load(path.parent/"recording.npz") as a:
                s=rescore(s,a["rows"])
            path.write_text(json.dumps(s,indent=2),encoding="utf-8")
            all_rows.append(s)
    by={(s["controller"],s["case"]):s for s in all_rows}
    cases=sorted({s["case"] for s in all_rows})
    for case in cases:
        for controller in ("mjpc","protomotions"):
            validate_pair(by["pd",case],by[controller,case])
    fields=("controller","case","outcome","task_achieved","stable_final_second","physical_step",
        "recovered_with_step","max_normal_force_n","max_floor_penetration_m",
        "max_body_penetration_m","max_joint_speed_rad_s","max_loaded_foot_slip_m",
        "positive_work_j","negative_work_j","external_work_j","mean_control_latency_ms","wall_s")
    with (out/"comparison.csv").open("w",encoding="utf-8",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        writer.writerows({k:r[k] for k in fields} for r in all_rows)
    (out/"comparison.json").write_text(json.dumps(all_rows,indent=2),encoding="utf-8")
    integrity=dict(pairs_checked=len(cases)*2,identity=list(IDENTITY),
        external_impulse_tolerance_ns=1e-8,all_comparable=True,
        controllers={c:{"complete":sum(r["task_achieved"] for r in all_rows if r["controller"]==c),
            "total":sum(r["controller"]==c for r in all_rows)} for c in ("pd","mjpc","protomotions")})
    (out/"integrity.json").write_text(json.dumps(integrity,indent=2),encoding="utf-8")
    return integrity

if __name__=="__main__":
    print(json.dumps(collect(Path(__file__).resolve().parents[2]),indent=2))
