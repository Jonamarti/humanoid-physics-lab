"""Frozen goals. IK builds reference data only; execution never writes root/feet."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

CASES = [
    {"id":"stand", "clip":0, "kind":"posture"},
    {"id":"squat", "clip":1, "kind":"squat"},
    {"id":"transfer", "clip":2, "kind":"transfer"},
    {"id":"step_left", "clip":3, "kind":"step"},
    {"id":"push_25", "clip":0, "kind":"recovery", "impulse_ns":25, "direction":[1,0,0]},
    {"id":"push_38", "clip":0, "kind":"recovery", "impulse_ns":38.4, "direction":[1,0,0]},
    {"id":"pull_38", "clip":0, "kind":"recovery", "impulse_ns":38.4, "direction":[-1,0,0]},
    {"id":"side_38", "clip":0, "kind":"recovery", "impulse_ns":38.4, "direction":[0,1,0]},
    {"id":"push_80", "clip":0, "kind":"recovery", "impulse_ns":80, "direction":[1,0,0]},
]
DURATION = 6.0
PULSE_START = 1.5
PULSE_DURATION = .16

def smooth(x):
    x=np.clip(x,0,1)
    return x*x*x*(10+x*(-15+6*x))

def prepare_pack(root: Path, path: Path, step_shift=.045, step_lift=.06, step_advance=.12, natural_arms=False):
    import mujoco
    import torch
    from scipy.spatial.transform import Rotation
    import os
    import sys
    source=root/"out/external/protomotions"
    sys.path.insert(0,str(source))
    os.chdir(source)
    from protomotions.robot_configs.anatomic_soma import AnatomicSomaRobotConfig
    from protomotions.simulator.mujoco.simulator import MujocoSimulator
    robot=AnatomicSomaRobotConfig()
    m=MujocoSimulator._load_mjcf_stripped(str(source/"protomotions/data/assets/mjcf/anatomic_soma.xml"),None)
    q0=m.qpos0.copy()
    q0[:3]=[0,0,1.]
    q0[3:7]=[1,0,0,0]
    # Our own lightly flexed neutral pose, not a pose copied from a dataset.
    for side,sign in (("Right",1),("Left",-1)):
        for name,value in ((side+"Shin_x",sign*.12),(side+"Leg_x",-sign*.06),(side+"Foot_x",-sign*.06)):
            q0[m.joint(name).qposadr[0]]=value
    if natural_arms:
        for side,sign in (("Right",1),("Left",-1)):
            q0[m.joint(side+"Arm_y").qposadr[0]]=-sign*1.45
            q0[m.joint(side+"ForeArm_z").qposadr[0]]=sign*.25
    scratch=mujoco.MjData(m)
    scratch.qpos[:]=q0
    mujoco.mj_forward(m,scratch)
    floor=m.geom("floor").id
    footids={m.body(n).id for n in ("LeftFoot","LeftToeBase","RightFoot","RightToeBase")}
    clearances=[mujoco.mj_geomDistance(m,scratch,floor,int(g),10,None)
        for g in np.flatnonzero(np.isin(m.geom_bodyid,list(footids)))]
    q0[2]+=.002-min(clearances)
    scratch.qpos[:]=q0
    mujoco.mj_forward(m,scratch)
    feet=[m.body(n).id for n in ("LeftFoot","RightFoot")]
    foot_p=scratch.xpos[feet].copy()
    foot_q=scratch.xquat[feet].copy()
    lower=[j for j in range(m.njnt) if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE
        and any(k in (m.joint(j).name or "") for k in ("Leg_","Shin_","Foot_","ToeBase_"))]
    cols=np.array([m.jnt_dofadr[j] for j in lower])
    qcols=np.array([m.jnt_qposadr[j] for j in lower])
    rotation=Rotation.from_quat(q0[3:7],scalar_first=True)
    fwd=rotation.apply([1.,0,0]); left=rotation.apply([0.,1.,0])
    common_ids=np.array([m.body(name).id for name in robot.kinematic_info.body_names])
    joint_names=[m.joint(j).name for j in range(m.njnt) if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE]
    dqmap=np.array([joint_names.index(name) for name in robot.kinematic_info.dof_names])
    times=np.arange(181)/30
    chunks=[]
    ik_errors=[]
    for kind in ("stand","squat","transfer","step"):
        q=q0.copy()
        qs=[]; ps=[]; rots=[]
        maxerr=0.
        for t in times:
            root=q0[:3].copy()
            targets=foot_p.copy()
            target_foot_q=foot_q.copy()
            if kind=="squat":
                a=smooth((t-.5)/1.25)-smooth((t-3.0)/1.25)
                root[2]-=.15*a
            elif kind=="transfer":
                root+=left*.06*(smooth((t-.5)/1.5)-smooth((t-3.5)/1.5))
            elif kind=="step":
                if natural_arms:
                    # Explicit LIFT, SWING and LAND goals. The foot clears before advancing.
                    lift=smooth((t-1.5)/.4)
                    travel=smooth((t-1.9)/.8)
                    landing_lower=smooth((t-2.7)/.4)
                    root[2]-=.04*smooth((t-.5)/1)-.025*smooth((t-3.1)/1)
                    root-=left*step_shift*(smooth((t-.5)/1)-smooth((t-3.1)/1))
                    root+=fwd*(step_advance/2)*smooth((t-1.9)/2)
                    targets[0]+=fwd*step_advance*travel
                    targets[0,2]+=step_lift*(lift-landing_lower)
                    u=np.clip((t-1.5)/1.6,0,1)
                    target_foot_q[0]=(Rotation.from_rotvec([0,-.20*(lift-landing_lower),0])*Rotation.from_quat(foot_q[0],scalar_first=True)).as_quat(scalar_first=True)
                    for side in ("Right","Left"):
                        q[m.joint(side+"Arm_x").qposadr[0]]=q0[m.joint(side+"Arm_x").qposadr[0]]-.18*np.sin(np.pi*u)**2
                else:
                    root[2]-=.04*smooth((t-.5)/1)
                    root-=left*step_shift*(smooth((t-.5)/1)-smooth((t-2.5)/1))
                    root+=fwd*(step_advance/2)*smooth((t-1.5)/2)
                    u=np.clip((t-1.5)/.8,0,1)
                    targets[0]+=fwd*step_advance*smooth(u)
                    targets[0,2]+=step_lift*np.sin(np.pi*u)**2

            q[:3]=root
            q[3:7]=q0[3:7]
            for _ in range(35):
                scratch.qpos[:q0.size]=q
                mujoco.mj_forward(m,scratch)
                errors=[]; jac=[]
                for i,b in enumerate(feet):
                    jp=np.zeros((3,m.nv)); jr=np.zeros_like(jp)
                    mujoco.mj_jacBody(m,scratch,jp,jr,b)
                    er=Rotation.from_quat(target_foot_q[i],scalar_first=True)*Rotation.from_quat(scratch.xquat[b],scalar_first=True).inv()
                    errors.extend([targets[i]-scratch.xpos[b], .15*er.as_rotvec()])
                    jac.extend([jp[:,cols], .15*jr[:,cols]])
                e=np.concatenate(errors); J=np.concatenate(jac)
                if np.max(np.abs(e))<2e-5: break
                delta=J.T@np.linalg.solve(J@J.T+np.eye(12)*1e-5,e)
                q[qcols]=np.clip(q[qcols]+np.clip(delta,-.12,.12),
                    m.jnt_range[lower,0],m.jnt_range[lower,1])
            scratch.qpos[:q0.size]=q
            mujoco.mj_forward(m,scratch)
            maxerr=max(maxerr,float(np.max(np.linalg.norm(targets-scratch.xpos[feet],axis=1))))
            qs.append(q.copy())
            ps.append(scratch.xpos[common_ids].copy())
            rots.append(scratch.xquat[common_ids].copy())
        qs=np.array(qs); ps=np.array(ps); rots=np.array(rots)
        # MotionLib's common quaternion convention is xyzw.
        grs=np.roll(rots,-1,axis=-1)
        gvs=np.gradient(ps,1/30,axis=0)
        gaws=np.zeros_like(ps)
        for b in range(53):
            rr=Rotation.from_quat(rots[:,b],scalar_first=True)
            rv=(rr[1:]*rr[:-1].inv()).as_rotvec()*30
            gaws[1:,b]=rv; gaws[0,b]=rv[0]
        dps=qs[:,7:][:,dqmap]
        dvs=np.gradient(dps,1/30,axis=0)
        contact=np.zeros((181,53),dtype=bool)
        for bname in ("LeftFoot","LeftToeBase","RightFoot","RightToeBase"):
            idx=list(robot.kinematic_info.body_names).index(bname)
            contact[:,idx]=True
            if kind=="step" and bname.startswith("Left"):
                contact[(times>1.5)&(times<(3.1 if natural_arms else 2.3)),idx]=False
        chunks.append(dict(gts=ps,grs=grs,gvs=gvs,gavs=gaws,dps=dps,dvs=dvs,contacts=contact))
        ik_errors.append({"kind":kind,"max_foot_reference_error_m":maxerr})
    pack={k:torch.from_numpy(np.concatenate([c[k] for c in chunks])).to(
        dtype=torch.bool if k=="contacts" else torch.float32) for k in chunks[0]}
    pack.update(motion_num_frames=torch.tensor([181]*4),motion_dt=torch.tensor([1/30]*4),
        motion_lengths=torch.tensor([6.]*4),motion_weights=torch.ones(4)/4,
        motion_files=tuple("hpl_"+k+".reference" for k in ("stand","squat","transfer","step")),
        length_starts=torch.tensor([0,181,362,543]),lrs=None)
    path.parent.mkdir(parents=True,exist_ok=True)
    torch.save(pack,path)
    meta={"seed":20261116,"duration_s":DURATION,"cases":CASES,"ik":ik_errors,
        "note":"Kinematic goals only; not evidence of physical motion or successful recovery.",
        "sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
    path.with_suffix(".json").write_text(json.dumps(meta,indent=2),encoding="utf-8")
    return meta
