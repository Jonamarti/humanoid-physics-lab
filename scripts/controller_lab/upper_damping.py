"""Implicit, motor-limited viscous damping selected from initial joint inertia."""
import numpy as np
import mujoco

class UpperDamping:
    def __init__(self):
        self.base=None
    def __call__(self,case,m,d):
        if self.base is None:self.base=m.actuator_biasprm.copy()
        m.actuator_biasprm[:]=self.base
        if not case.get("upper_damping") and not case.get("leg_absorption"):return {"enabled":False}
        M=np.empty((m.nv,m.nv));mujoco.mj_fullM(m,M,d.qM)
        changes=[]
        for a in range(m.nu):
            j=m.actuator_trnid[a,0];name=m.joint(j).name or ""
            upper=case.get("upper_damping") and any(name.startswith(p+"_") for p in ("Spine1","Spine2","Chest","Neck1","Head")) and name.endswith(("_x","_y"))
            leg=case.get("leg_absorption") and any(name==side+part for side in ("Left","Right") for part in ("Leg_x","Shin_x"))
            if not upper and not leg:continue
            v=m.jnt_dofadr[j];kp=m.actuator_gainprm[a,0];old=-self.base[a,2]
            inertia=float(M[v,v])
            cap=np.inf if case.get("inertia_matched_hip") and name in ("LeftLeg_x","RightLeg_x") else 2*old
            new=max(old,min(cap,2*.85*np.sqrt(kp*inertia)))
            m.actuator_biasprm[a,2]=-new
            changes.append(dict(joint=name,kp=float(kp),old_kd=float(old),new_kd=float(new),locked_joint_diagonal_inertia_kg_m2=inertia))
        return dict(enabled=True,target_isolated_joint_damping_ratio=.85,default_max_kd_multiplier=2.,
            inertia_matched_hip=case.get("inertia_matched_hip",False),note="Approximate joint-inertia scaling, not a neuromuscular model. Native960Hz velocity bias and original effort caps; no stiffness/contact/geometry change.",
            joints=changes)
