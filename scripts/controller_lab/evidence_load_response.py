"""Collect native evidence without relaxing gates or dropping failed trials."""
from pathlib import Path
import argparse,csv,hashlib,json
import numpy as np

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def collect(root):
    out=root/"docs/experiments/load-response-20261116";out.mkdir(parents=True,exist_ok=True)
    cases=[];rows=[]
    for review in sorted((root/"out/load-response").glob("*/*/rollover-review.json")):
        p=review.parent;s=json.loads((p/"summary.json").read_text(encoding="utf-8"))
        r=json.loads(review.read_text(encoding="utf-8"))
        item=dict(path=p.relative_to(root).as_posix(),summary=s,rollover=r)
        for filename in ("loading-energy.json","acceptance-plan.json","impedance-plan.json","contact-slip.json"):
            if (p/filename).exists():item[filename]=json.loads((p/filename).read_text(encoding="utf-8"))
        item["recording_hashes"]={f:digest(p/f) for f in ("recording.npz","dynamics.npz","rollover.npz","model.mjb","initial-observations.npz") if (p/f).exists()}
        cases.append(item)
        row=dict(case=item["path"],steps=r["steps"],heel_strikes=r["heel_first_strikes"],
            rollovers=r["completed_heel_to_forefoot_cycles"],stable=r["stable"],
            positive_work_j=s["positive_work_j"],floor_pen_m=s["max_floor_penetration_m"],
            loaded_origin_translation_m=s["max_loaded_foot_slip_m"],artifacts=";".join(s["artifacts"]))
        for segment in ("chest","head"):
            v=r["segments"][segment]
            row.update({segment+"_hf_acceleration_rms":v["acceleration_band_4_15hz_rms"],
                segment+"_peak_acceleration":v["max_acceleration_m_s2"],
                segment+"_roll_std_deg":v["roll_std_deg"],segment+"_pitch_std_deg":v["pitch_std_deg"]})
        rows.append(row)
    (out/"cases.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    with (out/"metrics.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    hypotheses={}
    for p in sorted((root/"out/load-response").glob("*/hypothesis.txt")):
        hypotheses[p.parent.name]=dict(hypothesis=p.read_text(encoding="utf-8"),
            scenarios=json.loads((p.parent/"scenarios.json").read_text(encoding="utf-8")))
    (out/"hypotheses.json").write_text(json.dumps(hypotheses,indent=2),encoding="utf-8")
    before=root/"out/load-response/prepared-early-v4/before";after=before.parent/"prepared_early"
    s=[json.loads((p/"summary.json").read_text(encoding="utf-8")) for p in [before,after]]
    a,b=[np.load(p/"initial-observations.npz") for p in [before,after]]
    equality={key:s[0][key]==s[1][key] for key in ("initial_state_sha256","model_sha256","physical_model_sha256")}
    equality["initial_observations"]=a.files==b.files and all(np.array_equal(a[k],b[k]) for k in a.files)
    old=np.load(root/"out/load-response/phase-v1/before/recording.npz");repeat=np.load(before/"recording.npz")
    equality["before_reproduced_exactly"]=all(np.array_equal(old[k],repeat[k]) for k in ("frames","rows","controls","initial_state"))
    preserved=json.loads((root/"docs/experiments/heel-toe-20261116/PRESERVED.json").read_text(encoding="utf-8"))
    old_video_hashes={name:digest(root/"out/heel-toe/delivery"/name)==meta["sha256"] for name,meta in preserved["videos"].items()}
    source=root/"scripts/controller_lab"
    audit=dict(selected_diagnostic=after.relative_to(root).as_posix(),physics_certified=False,
        equality=equality,old_videos_unchanged=old_video_hashes,
        flat_version=dict(tag=preserved["flat_baseline_tag"],commit=preserved["flat_baseline_commit"]),
        heel_toe_version=dict(tag="codex/heel-toe-before-load-response-20261116",commit="8c703f4"),
        native_hz=960,policy_hz=30,recording_hz=120,seed=20261116,
        checkpoint_sha256="c13e56e85494f8c93fe9eed5b07cadef7281c2eb1188231755f858e1d0f340f0",
        muscles_simulated=False,root_actuation=False,runtime_actual_pose_writes=False,
        source_hashes={p.name:digest(p) for p in source.glob("*.py")},
        videos=json.loads((root/"out/load-response/delivery/recepcion-antes-despues.json").read_text(encoding="utf-8")))
    if not all(equality.values()) or not all(old_video_hashes.values()):raise RuntimeError("Preservation/comparison audit failed")
    (out/"audit.json").write_text(json.dumps(audit,indent=2),encoding="utf-8")
    print("EVIDENCE",len(cases),"cases, all comparison/preservation equalities passed")

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("root",type=Path,nargs="?",default=Path(__file__).resolve().parents[2])
    collect(p.parse_args().root.resolve())

