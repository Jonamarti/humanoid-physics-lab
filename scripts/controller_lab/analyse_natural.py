"""Inspect actual sole clearance and support from saved physical trajectories."""
from pathlib import Path
import json
import numpy as np
import mujoco

def minimum_sole_clearance(m,d,floor,geoms):
    return min(mujoco.mj_geomDistance(m,d,floor,int(g),10,None) for g in geoms)

def analyse(path):
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"))
    d=mujoco.MjData(m)
    recording=np.load(path/"recording.npz")
    frames=recording["frames"];rows=recording["rows"]
    s=json.loads((path/"summary.json").read_text())
    root=m.body("Hips").id
    floor=m.geom("floor").id
    groups=[{m.body(n).id for n in names} for names in (
        ("LeftFoot","LeftToeBase"),("RightFoot","RightToeBase"))]
    geoms=[np.flatnonzero(np.isin(m.geom_bodyid,list(g)) & ((m.geom_contype|m.geom_conaffinity)!=0)) for g in groups]
    series=[];arms=[];lift=[None,None];land=[None,None];dwell=[0.,0.]
    for frame in frames:
        t=frame[0];d.qpos[:]=frame[1:];mujoco.mj_forward(m,d)
        r=rows[np.argmin(abs(rows[:,0]-t))]
        clearance=[minimum_sole_clearance(m,d,floor,gg) for gg in geoms]
        healthy=r[9]>.7*s["initial_root_height_m"] and r[10]<np.deg2rad(25) and r[11]==0
        for i in range(2):
            qualifies=healthy and clearance[i]>=.03 and r[7+i]<.05*s["mass_kg"]*9.81 and r[8-i]>.6*s["mass_kg"]*9.81
            dwell[i]=dwell[i]+8*m.opt.timestep if qualifies else 0.
            if dwell[i]>=.03 and lift[i] is None:lift[i]=float(t)
            if lift[i] is not None and land[i] is None and t>lift[i]+.05 and healthy and clearance[i]<.01 and r[7+i]>.1*s["mass_kg"]*9.81:
                land[i]=float(t)
        p=d.xpos[root]
        left=d.xpos[m.body("LeftHand").id]-p
        right=d.xpos[m.body("RightHand").id]-p
        arms.append([t,*left,*right])
        series.append([t,*clearance,r[7],r[8]])
    series=np.asarray(series);arms=np.asarray(arms)
    result=dict(case=s["case"],measurement="minimum floor distance over all foot/toe colliders, replay at 120Hz",
        max_sole_clearance_m=series[:,1:3].max(axis=0).tolist(),
        supported_lift_time_s=lift,loaded_landing_time_s=land,
        actual_foot_advance_m=s["foot_delta_m"][0][0],
        requested_foot_advance_m=s["desired_advance_m"],
        stable_final_second=s["stable_final_second"],artifacts=s["artifacts"],
        legacy_centroid_pass=s["task_achieved"],
        physical_step_observed=bool(lift[0] is not None and land[0] is not None and
            s["stable_final_second"] and not s["artifacts"] and s["foot_delta_m"][0][0]>.06),
        mean_hand_height_relative_pelvis_m=arms[:,[3,6]].mean(axis=0).tolist(),
        mean_hand_lateral_distance_m=np.abs(arms[:,[2,5]]).mean(axis=0).tolist())
    last=rows[rows[:,0]>=max(0,rows[-1,0]-2)]
    result["stable_final_two_seconds"]=bool(len(last) and np.all(last[:,9]>.7*s["initial_root_height_m"]) and np.all(last[:,10]<np.deg2rad(25)) and np.all(last[:,11]==0) and np.all(np.linalg.norm(last[:,4:6],axis=1)<.12) and np.all(last[:,7]+last[:,8]>.6*s["mass_kg"]*9.81))
    result["full_requested_distance_achieved"]=bool(result["physical_step_observed"] and abs(s["foot_delta_m"][0][0]-s["desired_advance_m"])<=.05 and abs(s["foot_delta_m"][0][1])<.05)
    np.savez_compressed(path/"sole-clearance.npz",series=series,arms=arms)
    (path/"physical-review.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result

if __name__=="__main__":
    root=Path(__file__).resolve().parents[2]
    for p in (root/"out/protomotions-natural-v3/protomotions").iterdir():
        if p.is_dir():print(json.dumps(analyse(p)),flush=True)
