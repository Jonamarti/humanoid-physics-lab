"""Execution-only telemetry, sampled at every actual MuJoCo substep."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco

def model_fingerprint(m):
    h=hashlib.sha256()
    for name in ("body_mass","body_inertia","body_pos","geom_type","geom_size","geom_pos",
        "geom_quat","geom_friction","geom_contype","geom_conaffinity","geom_solref","geom_solimp",
        "geom_condim","geom_margin","geom_gap","body_gravcomp","jnt_type","jnt_range",
        "jnt_stiffness","jnt_solref","jnt_solimp","dof_damping","dof_frictionloss","dof_armature","actuator_gainprm","actuator_biasprm",
        "actuator_forcerange","actuator_forcelimited","actuator_gear"):
        h.update(name.encode()); h.update(np.asarray(getattr(m,name)).tobytes())
    h.update(str((m.opt.timestep,m.opt.integrator,m.opt.solver,m.opt.iterations,m.opt.tolerance,m.opt.impratio,m.opt.cone,list(m.opt.gravity))).encode())
    return h.hexdigest()


def physical_fingerprint(m):
    """Version 2: includes actuator enums, joint axes and all compiled arrays."""
    h=hashlib.sha256()
    for prefix,obj in (("model",m),("option",m.opt)):
        for name in sorted(dir(obj)):
            if name.startswith("_"):continue
            value=getattr(obj,name)
            key=(prefix+"."+name).encode()
            if isinstance(value,np.ndarray):
                h.update(key);h.update(value.dtype.str.encode());h.update(str(value.shape).encode())
                h.update(np.ascontiguousarray(value).tobytes())
            elif isinstance(value,(int,float,np.integer,np.floating)):
                h.update(key);h.update(repr(value).encode())
            elif isinstance(value,bytes):
                h.update(key);h.update(value)
    return h.hexdigest()

class Telemetry:
    def __init__(self,m,d,prefix=""):
        self.m=m; self.d=d; self.rows=[]; self.frames=[]; self.controls=[]
        self.root=m.body(prefix+"Hips").id
        self.feet=[m.body(prefix+n).id for n in ("LeftFoot","RightFoot")]
        self.foot_groups=[{m.body(prefix+n).id for n in names} for names in (
            ("LeftFoot","LeftToeBase"),("RightFoot","RightToeBase"))]
        self.mass=float(m.body_subtreemass[self.root])
        self.start=d.time; self.q0=d.qpos.copy(); self.v0=d.qvel.copy()
        self.initial_state=np.zeros(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(m,d,self.initial_state,mujoco.mjtState.mjSTATE_INTEGRATION)
        self.external_impulse=np.zeros(3)
        self.initial_feet=d.xpos[self.feet].copy()
        self.positive_work=0.; self.negative_work=0.; self.external_work=0.
        self.max_force=0.; self.max_floor_pen=0.; self.max_body_pen=0.
        self.max_joint_speed=0.; self.max_effort_excess=0.; self.root_force_max=0.
        self.peak=None; self.anchors=[None,None]; self.max_slip=0.; self.events=[]
        self.last_load=[False,False]; self.lifted=[False,False]; self.landed=[False,False]
        self.airtime=0.; self.max_airtime=0.; self.idx=0
        self.last_com=None; self.min_height=float(d.xpos[self.root,2])
        self.initial_height=self.min_height
        self.final_com_speed=0.
        self.control_latencies=[]

    def sample(self):
        m,d=self.m,self.d; dt=m.opt.timestep; t=d.time-self.start
        load=np.zeros(2); nonfoot_floor=False
        for cidx,c in enumerate(d.contact):
            if c.efc_address<0: continue
            force=np.zeros(6)
            mujoco.mj_contactForce(m,d,cidx,force)
            normal=max(0.,float(force[0]))
            b1,b2=int(m.geom_bodyid[c.geom1]),int(m.geom_bodyid[c.geom2])
            floor=(b1==0 or b2==0)
            if normal>self.max_force:
                self.max_force=normal
                self.peak={"time_s":t,"normal_n":normal,
                    "bodies":[m.body(b1).name,m.body(b2).name]}
            pen=max(0.,-float(c.dist))
            if floor:
                self.max_floor_pen=max(self.max_floor_pen,pen)
                b=b2 if b1==0 else b1
                f=np.asarray(c.frame).reshape(3,3).T@force[:3]
                if b2==0: f=-f
                for i,group in enumerate(self.foot_groups):
                    if b in group: load[i]+=max(0.,f[2])
                if m.body_rootid[b]==self.root and b not in set.union(*self.foot_groups) and normal>.02*self.mass*9.81:
                    nonfoot_floor=True
            else: self.max_body_pen=max(self.max_body_pen,pen)
        power=float(np.dot(d.qfrc_actuator,d.qvel))
        joint_power=d.qfrc_actuator*d.qvel
        self.positive_work+=float(np.maximum(0.,joint_power).sum())*dt
        self.negative_work+=float(np.minimum(0.,joint_power).sum())*dt
        velocities=np.zeros((m.nbody,6))
        for b in np.flatnonzero(np.any(d.xfrc_applied!=0,axis=1)):
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,int(b),velocities[b],0)
            self.external_impulse+=d.xfrc_applied[b,:3]*dt
            self.external_work+=float(np.dot(d.xfrc_applied[b,:3],velocities[b,3:])+
                np.dot(d.xfrc_applied[b,3:],velocities[b,:3]))*dt
        self.root_force_max=max(self.root_force_max,float(np.max(np.abs(d.qfrc_applied[:6]))))
        excess=np.maximum(d.actuator_force-m.actuator_forcerange[:,1],
            m.actuator_forcerange[:,0]-d.actuator_force)
        self.max_effort_excess=max(self.max_effort_excess,float(np.max(excess)))
        hinge=m.jnt_dofadr[m.jnt_type==mujoco.mjtJoint.mjJNT_HINGE]
        self.max_joint_speed=max(self.max_joint_speed,float(np.max(np.abs(d.qvel[hinge]))))
        com=d.subtree_com[self.root].copy()
        speed=np.zeros(3) if self.last_com is None else (com-self.last_com)/dt
        self.last_com=com
        self.final_com_speed=float(np.linalg.norm(speed[:2]))
        self.min_height=min(self.min_height,float(d.xpos[self.root,2]))
        for i in range(2):
            loaded=bool(load[i]>.1*self.mass*9.81)
            if loaded and not self.last_load[i]:
                self.anchors[i]=d.xpos[self.feet[i],:2].copy()
                self.events.append({"t":t,"foot":i,"event":"loaded"})
                if self.lifted[i]: self.landed[i]=True
            if not loaded: self.anchors[i]=None
            if loaded and self.anchors[i] is not None:
                self.max_slip=max(self.max_slip,float(np.linalg.norm(d.xpos[self.feet[i],:2]-self.anchors[i])))
            if load[i]<.05*self.mass*9.81 and d.xpos[self.feet[i],2]-self.initial_feet[i,2]>.03:
                if not self.lifted[i]: self.events.append({"t":t,"foot":i,"event":"clearance_3cm"})
                self.lifted[i]=True
            self.last_load[i]=loaded
        self.airtime=self.airtime+dt if sum(load)<.05*self.mass*9.81 else 0.
        self.max_airtime=max(self.max_airtime,self.airtime)
        tilt=float(np.arccos(np.clip(d.xmat[self.root].reshape(3,3)[2,2],-1,1)))
        self.rows.append([t,*com,*speed,*load,float(d.xpos[self.root,2]),tilt,float(nonfoot_floor),power])
        if self.idx%8==0:
            self.frames.append(np.r_[t,d.qpos.copy()])
            self.controls.append(d.ctrl.copy())
        self.idx+=1

    def finish(self,case,controller,out,wall_s):
        rows=np.asarray(self.rows); final=rows[rows[:,0]>=max(0,rows[-1,0]-1)]
        stable=bool(len(final) and final[-1,0]-final[0,0]>.98 and
            np.all(final[:,9]>.7*self.initial_height) and
            np.all(final[:,10]<np.deg2rad(25)) and np.all(final[:,11]==0) and
            np.all(np.linalg.norm(final[:,4:6],axis=1)<.12) and
            np.all(final[:,7]+final[:,8]>.6*self.mass*9.81))
        moved=np.linalg.norm(self.d.xpos[self.feet,:2]-self.initial_feet[:,:2],axis=1)
        physical_step=any(self.lifted[i] and self.landed[i] and moved[i]>.06 for i in range(2))
        artifacts=[]
        if self.max_airtime>.08: artifacts.append("unintended_flight")
        if self.max_floor_pen>.01: artifacts.append("floor_penetration")
        if self.max_body_pen>.02: artifacts.append("body_penetration")
        if self.max_force>10000: artifacts.append("contact_force")
        if self.max_joint_speed>25: artifacts.append("joint_speed")
        if self.max_slip>.02: artifacts.append("loaded_foot_slip")
        if self.max_effort_excess>1e-6: artifacts.append("actuator_limit")
        if self.root_force_max>1e-9: artifacts.append("root_force")
        root_excursion=float(rows[:,9].max()-rows[:,9].min())
        foot_delta=self.d.xpos[self.feet]-self.initial_feet
        clean_stable=stable and not artifacts
        if case["kind"]=="squat":
            achieved=clean_stable and root_excursion>=.12 and abs(rows[-1,9]-self.initial_height)<.03
        elif case["kind"]=="transfer":
            fractions=np.max(rows[:,7:9],axis=1)/np.maximum(rows[:,7]+rows[:,8],1.)
            achieved=clean_stable and np.sum(fractions>=.75)*self.m.opt.timestep>=.2
        elif case["kind"]=="step":
            achieved=clean_stable and self.lifted[0] and self.landed[0] and case.get("goal_advance_m",.12)-.05<=foot_delta[0,0]<=case.get("goal_advance_m",.12)+.05 and abs(foot_delta[0,1])<.05
        else:
            achieved=clean_stable
        outcome="complete" if achieved else ("fall" if self.min_height<.5*self.initial_height else "incomplete")
        if case["kind"]=="recovery" and achieved:
            outcome="recovered_with_step" if physical_step else "recovered_without_step"
        result=dict(case=case["id"],controller=controller,kind=case["kind"],
            model_sha256=model_fingerprint(self.m),mujoco=mujoco.__version__,
            mass_kg=self.mass,duration_s=float(rows[-1,0]),wall_s=wall_s,
            max_normal_force_n=self.max_force,peak=self.peak,
            max_floor_penetration_m=self.max_floor_pen,max_body_penetration_m=self.max_body_pen,
            positive_work_j=self.positive_work,negative_work_j=self.negative_work,
            external_work_j=self.external_work,external_impulse_ns=self.external_impulse.tolist(),max_joint_speed_rad_s=self.max_joint_speed,
            max_loaded_foot_slip_m=self.max_slip,max_airborne_time_s=self.max_airtime,
            min_root_height_m=self.min_height,initial_root_height_m=self.initial_height,
            desired_advance_m=case.get("goal_advance_m",.12),outcome=outcome,task_achieved=bool(achieved),root_excursion_m=root_excursion,foot_delta_m=foot_delta.tolist(),
            stable_final_second=stable,physical_step=physical_step,
            recovered_with_step=bool(case["kind"]=="recovery" and stable and physical_step and not artifacts),
            artifacts=artifacts,events=self.events,
            mean_control_latency_ms=float(np.mean(self.control_latencies))*1000 if self.control_latencies else None)
        out.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(out/"recording.npz",frames=np.array(self.frames),rows=rows,
            controls=np.array(self.controls),initial_qpos=self.q0,initial_qvel=self.v0,initial_state=self.initial_state)
        (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        from snapshots import save_model
        save_model(self.m,out)
        return result
