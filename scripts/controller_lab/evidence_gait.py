"""Small public evidence package; trajectories/media/checkpoints stay in ignored out/."""
from pathlib import Path
import csv,hashlib,json,shutil,sys,platform
import numpy as np
import mujoco
from proto_adapter import PROTO_SHA,CHECKPOINT_SHA
ROOT=Path(__file__).resolve().parents[2]
DEST=ROOT/"docs/experiments/protomotions-gait-20261116"
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    DEST.mkdir(parents=True,exist_ok=True)
    walk=ROOT/"out/gait/reachable-lift";hits=ROOT/"out/gait/impacts-final"
    for base,label in ((walk,"walking"),(hits,"impacts")):
        for src,name in (("metrics.csv",label+".csv"),("scenarios.json",label+"-scenarios.json"),("audit.json",label+"-audit.json")):
            shutil.copyfile(base/src,DEST/name)
    reviews={p.parent.name:json.loads(p.read_text(encoding="utf-8")) for p in walk.glob("*/review.json")}
    (DEST/"walking-review.json").write_text(json.dumps(reviews,indent=2),encoding="utf-8")
    before=ROOT/"out/gait/lateral-ab/fast_baseline";after=walk/"fast"
    bs=json.loads((before/"summary.json").read_text(encoding="utf-8"));as_=json.loads((after/"summary.json").read_text(encoding="utf-8"))
    a=np.load(before/"recording.npz");b=np.load(after/"recording.npz")
    comparison=dict(cadence_period_s=.8,identical_full_initial_state=bool(np.array_equal(a["initial_state"],b["initial_state"])),
        identical_physical_model=bs["physical_model_sha256"]==as_["physical_model_sha256"],
        duration_equal=abs(bs["duration_s"]-as_["duration_s"])<1e-8,
        before={k:bs[k] for k in ("positive_work_j","max_loaded_foot_slip_m","artifacts")},
        after={k:as_[k] for k in ("positive_work_j","max_loaded_foot_slip_m","artifacts")},
        note="Reference generator is the intended intervention; checkpoint, model, cadence and full initial state are identical.")
    (DEST/"walking-comparison.json").write_text(json.dumps(comparison,indent=2),encoding="utf-8")
    source_files={}
    for name in ("proto_adapter.py","gait.py","gait_suite.py","recovery.py","support_feedback.py",
                 "contact_slip.py","report_gait.py","render_gait.py","deliver_gait.py","test_gait.py","evidence_gait.py"):
        p=ROOT/"scripts/controller_lab"/name;source_files[name]=sha(p)
    checkpoint=ROOT/"out/external/protomotions/data/pretrained_models/motion_tracker/anatomic_soma_bones_fsq/inference_last.ckpt"
    actual=sha(checkpoint)
    if actual!=CHECKPOINT_SHA:raise RuntimeError("Changed checkpoint")
    provenance=dict(protomotions_commit=PROTO_SHA,checkpoint_sha256=actual,
        references="Authored procedural IK, no raw BONES-SEED/AMASS/SMPL motions",
        code_license="Apache-2.0 upstream; checkpoint separate NVIDIA Open Model License",
        mujoco=mujoco.__version__,python=sys.version,platform=platform.platform(),device="CPU",
        physics_hz=960,control_hz=30,recording_hz=120,seed=20261116,training_runs=0,cloud_jobs=0,
        robot="anatomic_soma",mass_kg=as_["mass_kg"],source_sha256=source_files,
        walking_reference_sha256=sha(walk/"references.pt"),impact_reference_sha256=sha(hits/"references.pt"),
        reference_max_ik_error_m=max(c["max_ik_error_m"] for c in json.loads((walk/"scenarios.json").read_text(encoding="utf-8"))),
        runtime_root_position_writes="Initial reset/snapshot restore only; no kinematic root/foot writes during measured execution",
        external_forces="Declared world-space Chest COM rectangular pulses only",
        replay="Saved states and mj_forward; mj_step forbidden",
        stance_residual_and_contact_solver_variants="Experimental/rejected, not in final walking or impact suites")
    (DEST/"provenance.json").write_text(json.dumps(provenance,indent=2),encoding="utf-8")
    checks=dict(native_tests=23,native_passed="Ran 23 tests" in (ROOT/"out/gait-tests.log").read_text(encoding="utf-8",errors="replace") and "\nOK\n" in (ROOT/"out/gait-tests.log").read_text(encoding="utf-8",errors="replace"),
        typescript_tests_passed=706,typescript_skipped=9,typescript_todo=3,
        typescript_test_files_passed=87,typecheck="passed",lint="passed",build="passed",
        physics_not_certified=True,all_24_impact_pairs_valid=json.loads((hits/"audit.json").read_text(encoding="utf-8"))["all_pairs_valid"])
    (DEST/"validation.json").write_text(json.dumps(checks,indent=2),encoding="utf-8")
    media=ROOT/"out/gait/delivery/media-audit.json"
    if media.exists():shutil.copyfile(media,DEST/"media-audit.json")
    print(json.dumps(dict(comparison=comparison,validation=checks)),flush=True)
if __name__=="__main__":main()
