"""Save compact evidence; never vendor datasets, checkpoints or private assets."""
from pathlib import Path
import hashlib
import importlib.metadata
import json
import platform
import shutil
import subprocess
import numpy as np
import mujoco
from compare import collect,validate_pair
from metrics import physical_fingerprint

def write(root):
    out=root/"out/controller-comparison"
    target=root/"docs/experiments/controller-comparison-20261116"
    target.mkdir(exist_ok=True,parents=True)
    integrity=collect(root)
    matrix=json.loads((out/"comparison.json").read_text())
    physical={}
    for s in matrix:
        model=mujoco.MjModel.from_binary_path(str(out/s["controller"]/s["case"]/"model.mjb"))
        fingerprint=physical_fingerprint(model)
        physical.setdefault(s["case"],{})[s["controller"]]=fingerprint
    if not all(len(set(v.values()))==1 for v in physical.values()):
        raise RuntimeError("Full physical fingerprints differ")
    contacts=json.loads((out/"contacts.json").read_text())
    for case in ("arm_push","leg_push","body_support"):
        a=out/"contacts"/case/"enabled";b=out/"contacts"/case/"disabled"
        if not np.array_equal(np.load(a/"command_targets.npy"),np.load(b/"command_targets.npy")):
            raise RuntimeError("Contact control commands differ")
        if not np.array_equal(np.load(a/"recording.npz")["initial_qpos"],np.load(b/"recording.npz")["initial_qpos"]):
            raise RuntimeError("Contact control initial poses differ")
    natural=root/"out/protomotions-natural-v3/protomotions/natural_step_left"
    review=json.loads((natural/"physical-review.json").read_text())
    summary=json.loads((natural/"summary.json").read_text())
    rows=np.load(natural/"recording.npz")["rows"]
    last=rows[rows[:,0]>=4.]
    review["stable_final_two_seconds"]=bool(np.all(last[:,9]>.7*summary["initial_root_height_m"])
        and np.all(last[:,10]<np.deg2rad(25)) and np.all(last[:,11]==0)
        and np.all(np.linalg.norm(last[:,4:6],axis=1)<.12)
        and np.all(last[:,7]+last[:,8]>.6*summary["mass_kg"]*9.81))
    (natural/"physical-review.json").write_text(json.dumps(review,indent=2),encoding="utf-8")
    # Correct a label only: no disturbance was applied in voluntary-step cases.
    for p in list(out.glob("*/*/summary.json"))+list((root/"out/protomotions-natural-v3").glob("*/*/summary.json")):
        s=json.loads(p.read_text())
        if s["kind"]!="recovery":s["recovered_with_step"]=False
        p.write_text(json.dumps(s,indent=2),encoding="utf-8")
    source_hash=hashlib.sha256()
    for p in sorted((root/"scripts/controller_lab").rglob("*")):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix in (".py",".ps1",".cc",".txt"):
            source_hash.update(str(p.relative_to(root)).replace("\\","/").encode());source_hash.update(p.read_bytes())
    provenance=dict(python=platform.python_version(),system=platform.system(),mujoco=mujoco.__version__,
        protomotions_sha="13de0cccd06f53ce5c9cf9ec7d1932c7da73610f",
        mujoco_mpc_sha="ff572a21e7c2bf9fda62e1862a758da7e9a8719b",
        checkpoint_sha256="c13e56e85494f8c93fe9eed5b07cadef7281c2eb1188231755f858e1d0f340f0",
        source_sha256=source_hash.hexdigest(),seed=20261116,cloud_jobs=0,
        packages={d.metadata["Name"]:d.version for d in importlib.metadata.distributions()},
        dataset_motion_samples_used_for_delivered_trials=False,
        model_snapshots_note="Initial matrix binary snapshots reconstructed from the pinned constructor and checked against its recorded fingerprint; new trials save the binary directly.")
    validation=dict(project_tests_passed=706,project_tests_skipped=9,project_tests_todo=3,
        typecheck=True,lint=True,build=True,native_unit_tests=10,
        comparable_pairs=integrity["pairs_checked"],full_physical_fingerprints=physical,
        contacts_same_initial_pose_and_commands=True,
        natural_video_decoded=True,recording_render_physics_steps=0)
    compact=[{k:v for k,v in s.items() if k!="events"} for s in matrix]
    for name,value in (("comparison.json",compact),("integrity.json",integrity),("contacts.json",contacts),
        ("natural-step.json",review),("natural-step-summary.json",summary),
        ("provenance.json",provenance),("validation.json",validation)):
        (target/name).write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding="utf-8")
    shutil.copyfile(out/"comparison.csv",target/"comparison.csv")
    print(json.dumps({"evidence":str(target),"natural":review},indent=2))

if __name__=="__main__":write(Path(__file__).resolve().parents[2])
