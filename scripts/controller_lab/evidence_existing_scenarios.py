"""Audit paired attacks, provenance and failures without changing acceptance gates."""
from pathlib import Path
import argparse,csv,hashlib,json
import numpy as np
import mujoco

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def model_equivalence(a,b):
    differences=[]
    owners=[("model",a,b)]
    if hasattr(a,"opt") and hasattr(b,"opt"):owners.append(("option",a.opt,b.opt))
    for prefix,left,right in owners:
        for key in dir(left):
            if key.startswith("_"):continue
            x=getattr(left,key);y=getattr(right,key)
            if isinstance(x,np.ndarray):
                if np.array_equal(x,y):continue
                if x.shape!=y.shape:return False,[dict(array=prefix+"."+key,reason="shape")]
                delta=np.abs(x-y)
                differences.append(dict(array=prefix+"."+key,max_difference=float(delta.max())))
                # Historical v2 recalculated inertia-based Kd. Record raw inequality;
                # allow only velocity-column rounding, never geometry/gain/limit changes.
                if prefix!="model" or key!="actuator_biasprm":return False,differences
                if not np.array_equal(x[:,:2],y[:,:2]) or not np.array_equal(x[:,3:],y[:,3:]):return False,differences
                bound=64*np.finfo(np.float64).eps*np.maximum(1,np.maximum(np.abs(x[:,2]),np.abs(y[:,2])))
                if np.any(delta[:,2]>bound):return False,differences
            elif isinstance(x,(int,float,np.integer,np.floating,bytes)) and x!=y:
                return False,[*differences,dict(array=prefix+"."+key,reason="scalar")]
    return True,differences

def case_path(trials,name):
    catalog=trials/"case-paths.json"
    if not catalog.exists():return trials/name
    repository=Path(__file__).resolve().parents[2]
    relative=json.loads(catalog.read_text(encoding="utf-8"))[name]
    return repository/relative

def collect(root,trials):
    evidence=root/"docs/experiments/existing-scenarios-20261116";evidence.mkdir(parents=True,exist_ok=True)
    results=json.loads((trials/"results.json").read_text(encoding="utf-8"))
    byid={r["summary"]["case"]:r for r in results}
    checks=[];rows=[]
    for name in ["no_attack","punch_forward20","punch_backward20","kick_left36","kick_backward36","outer_reap","leg_drive"]:
        paths=[case_path(trials,name+"_"+kind) for kind in ["tracker","reactive"]]
        data=[np.load(p/"recording.npz") for p in paths]
        obs=[np.load(p/"initial-observations.npz") for p in paths]
        summaries=[byid[p.name]["summary"] for p in paths]
        reference_end=min([2.]+[e["trigger_s"] for s in summaries for e in s["planner_events"]])
        mask=data[0]["frames"][:,0]<reference_end
        max_difference=float(np.abs(data[0]["frames"][mask]-data[1]["frames"][mask]).max())
        commands_equal=np.array_equal(data[0]["controls"][mask],data[1]["controls"][mask])
        models=[mujoco.MjModel.from_binary_path(str(p/"model.mjb")) for p in paths]
        equivalent,differences=model_equivalence(*models)
        check=dict(case=name,model_equivalent_at_float64_roundoff=equivalent,model_parameter_differences=differences,full_initial_state_equal=np.array_equal(data[0]["initial_state"],data[1]["initial_state"]),
            initial_observations_equal=all(np.array_equal(obs[0][k],obs[1][k]) for k in obs[0].files),
            model_equal=summaries[0]["physical_model_sha256"]==summaries[1]["physical_model_sha256"],
            reference_equal=summaries[0]["reference_sha256"]==summaries[1]["reference_sha256"],
            pre_intervention_commands_equal=commands_equal,pre_intervention_state_max_difference=max_difference,
            pre_intervention_until_s=reference_end)
        if not all(check[k] for k in ["full_initial_state_equal","initial_observations_equal","model_equivalent_at_float64_roundoff","reference_equal","pre_intervention_commands_equal"]):
            raise RuntimeError("Pair comparison failed: "+name)
        if max_difference>1e-10:raise RuntimeError("Pre-intervention state divergence: "+name)
        checks.append(check)
    for result in results:
        s=result["summary"];r=result["review"];p=case_path(trials,s["case"])
        attacker=s.get("attacker",{})
        if s["max_direct_root_wrench"]!=0:raise RuntimeError("Direct robot root force")
        if attacker:
            if max(attacker["drive_cap_ratios"].values())>1+1e-9:raise RuntimeError("Attacker cap exceeded")
            if attacker["max_grip_pair_moment_residual_nm"]>1e-8:raise RuntimeError("Grip pair moment failed")
        if "impulse_ns" in s["scenario"]:
            expected=np.array(s["scenario"]["direction"])*s["scenario"]["impulse_ns"]
            np.testing.assert_allclose(s["external_impulse_ns"],expected,atol=1e-9,rtol=0)
        rows.append(dict(case=s["case"],mass_kg=s["mass_kg"],fell=r["fell"],stable=r["stable_final_two_seconds"],steps=r["steps"],
            clean_recovery=bool(r["stable_final_two_seconds"] and r["steps"] and not s["artifacts"]),
            max_contact_n=s["max_normal_force_n"],max_actor_contact_n=attacker.get("max_actor_contact_force_n",0),
            max_actor_penetration_m=attacker.get("max_actor_penetration_m",0),floor_penetration_m=s["max_floor_penetration_m"],
            max_joint_speed_rad_s=s["max_joint_speed_rad_s"],foot_origin_translation_m=s["max_loaded_foot_slip_m"],
            positive_joint_work_j=s["positive_work_j"],artifacts=";".join(s["artifacts"]),
            max_grip_force_n=attacker.get("max_grip_force_n",0),planner_events=len(s["planner_events"])))
        result["recording_hashes"]={name:sha(p/name) for name in ["recording.npz","dynamics.npz","model.mjb"]}
    with (evidence/"metrics.csv").open("w",encoding="utf-8",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (evidence/"results.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
    (evidence/"pairs.json").write_text(json.dumps(checks,indent=2),encoding="utf-8")
    source=root/"scripts/controller_lab"
    manifest=dict(trial_path=trials.relative_to(root).as_posix(),cases=len(results),pairs=len(checks),
        old_gait_branch="codex/protomotions-heel-toe",old_gait_commit="3fc4fe63e10f4fb0eaec6f69b604c543103eb02e",
        tori_tracks_source_commit="1d10c0643a038534d82941c1cfeda7e1b01e87c8",
        source_track_hashes={n:sha(root/"scenarios/physical_refs"/n) for n in ["outer_reap.json","leg_drive.json"]},
        delivered_source_hashes={n:sha(source/n) for n in ["existing_scenarios.py","scene_forces.py","proto_adapter.py","export_scene.ts","render_existing_scenarios.py"]},
        physics_hz=960,policy_hz=30,recording_hz=120,targets_hz=960,physical_preparation_s=3.,
        actor_model="published SOMA42.466827kg, not historical Uke75kg",
        attacker_model="public existing Tori75kg independent finite-mass segments",
        hit_model="analytical rectangular chest pulses80ms/180ms, not striking limbs",
        physics_certified=False,model_comparison_note="Historical prepared-v2 recalculated damping: byte hashes differ solely by <=64eps rounding of velocity bias; pairs.json records actual deltas. Delivered runner freezes configuration once. Physical artifact thresholds unchanged.",
        preserved_preparation_without_warmup="out/existing-scenarios/trials-v1",
        preliminary_resampled_tori="out/existing-scenarios/prepared-v2",
        case_paths=json.loads((trials/"case-paths.json").read_text(encoding="utf-8")) if (trials/"case-paths.json").exists() else None,
        videos={})
    delivery=root/"out/existing-scenarios/delivery"
    manifest["preliminary_videos"]={}
    for p in delivery.glob("*.json"):
        destination=manifest["videos"] if p.name in ("golpes-protomotions.json","derribos-protomotions-960hz.json") else manifest["preliminary_videos"]
        destination[p.name]=json.loads(p.read_text(encoding="utf-8"))
    targets=evidence/"targets-audit.json"
    if targets.exists():manifest["target_fidelity"]=json.loads(targets.read_text(encoding="utf-8"))
    (evidence/"provenance.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    print("AUDITED",len(results),"trials,",len(checks),"matched pairs")

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--trials",default="out/existing-scenarios/final-v3");a=p.parse_args()
    root=Path(__file__).resolve().parents[2];collect(root,(root/a.trials).resolve())
