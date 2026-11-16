"""Frozen evaluation audit: gait metrics and before/after chest-pulse comparability."""
from pathlib import Path
import argparse,csv,json
import numpy as np

def record(path):
    s=json.loads((path/"summary.json").read_text(encoding="utf-8"))
    r=json.loads((path/"review.json").read_text(encoding="utf-8"))
    a=np.load(path/"recording.npz")
    rows=a["rows"];frames=a["frames"]
    sol=np.load(path/"sole-series.npz")["series"]
    desired_y=sol[0,[4,6]]
    r["max_lateral_landing_error_m"]=max(
        [abs(e["to_xy"][1]-desired_y[e["foot"]]) for e in r["events"]],default=0.)
    # Preserve >=3cm lift gate. Add lower-clearance cycle count separately.
    r["strict_step_clearance_m"]=.03
    r["steps_after_hit"]=sum(e["lift_s"]>=s["scenario"].get("hit_time",0) for e in r["events"])
    d=dict(case=s["case"],steps=r["steps"],alternating=r["alternating"],
        stable_2s=r["stable_final_two_seconds"],fell=r["fell"],speed_m_s=r["mean_forward_speed_m_s"],
        distance_m=r["actual_progress_m"],positive_work_j=s["positive_work_j"],
        work_per_m_j=s["positive_work_j"]/max(.01,abs(r["actual_progress_m"])),
        max_lateral_landing_error_m=r["max_lateral_landing_error_m"],
        floor_penetration_m=s["max_floor_penetration_m"],foot_translation_proxy_m=s["max_loaded_foot_slip_m"],
        peak_normal_force_n=s["max_normal_force_n"],max_joint_speed_rad_s=s["max_joint_speed_rad_s"],
        artifacts=";".join(s["artifacts"]),triggers=len(s.get("planner_events",[])),
        steps_after_hit=r["steps_after_hit"],impulse_ns=s["scenario"].get("impulse_ns",0))
    r["strict_valid_recovery_with_step"]=bool(r["steps_after_hit"] and r["stable_final_two_seconds"] and not s["artifacts"] and not r["fell"])
    (path/"review.json").write_text(json.dumps(r,indent=2),encoding="utf-8")
    return d,s,a

def report(root):
    data=[];pairs=[];cache={}
    for p in sorted(root.iterdir()):
        if not (p/"review.json").exists():continue
        row,s,a=record(p);data.append(row);cache[p.name]=(row,s,a)
    for name,(row,s,a) in cache.items():
        if not name.endswith("_reactive"):continue
        other=name[:-9]+"_baseline"
        if other not in cache:continue
        rb,sb,b=cache[other];t=s["scenario"].get("hit_time",0.)
        mask=a["frames"][:,0]<t
        same_model=s["physical_model_sha256"]==sb["physical_model_sha256"]
        same_initial=np.array_equal(a["initial_state"],b["initial_state"])
        same_reference=s["reference_sha256"]==sb["reference_sha256"]
        prehit=np.array_equal(a["frames"][mask],b["frames"][mask]) and np.array_equal(a["controls"][mask],b["controls"][mask])
        impulse=np.allclose(s["external_impulse_ns"],sb["external_impulse_ns"],rtol=0,atol=1e-8)
        pairs.append(dict(case=name,model_equal=same_model,initial_equal=same_initial,
            input_reference_equal=same_reference,prehit_execution_equal=prehit,impulse_equal=impulse,
            valid=bool(same_model and same_initial and same_reference and prehit and impulse)))
    if data:
        with (root/"metrics.csv").open("w",newline="",encoding="utf-8") as f:
            writer=csv.DictWriter(f,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
    result=dict(cases=len(data),fell=sum(v["fell"] for v in data),
        stable=sum(v["stable_2s"] for v in data),with_qualified_steps=sum(v["steps_after_hit"]>0 for v in data),
        stable_with_step=sum(v["steps_after_hit"]>0 and v["stable_2s"] and not v["fell"] for v in data),
        artifact_free=sum(not v["artifacts"] for v in data),
        validated_recovery_with_step=sum(v["steps_after_hit"]>0 and v["stable_2s"] and not v["fell"] and not v["artifacts"] for v in data),
        pairs=pairs,all_pairs_valid=all(v["valid"] for v in pairs))
    (root/"audit.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps(result),flush=True)
    return result
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("root",type=Path);a=p.parse_args();report(a.root)
