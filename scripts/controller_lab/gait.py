"""Own IK goals for the unchanged published tracker; never a physics position driver."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from cases import smooth
from proto_adapter import create_agent, reset_clip, action
from metrics import Telemetry, physical_fingerprint

class ReferenceBuilder:
    def __init__(self, root):
        import os,torch
        os.environ["TORCHDYNAMO_DISABLE"]="1"
        torch.set_num_threads(2)
        self.root=root.resolve()
        source=self.root/"out/external/protomotions"
        sys.path.insert(0,str(source))
        from protomotions.robot_configs.anatomic_soma import AnatomicSomaRobotConfig
        from protomotions.simulator.mujoco.simulator import MujocoSimulator
        self.robot=AnatomicSomaRobotConfig()
        self.m=MujocoSimulator._load_mjcf_stripped(str(source/"protomotions/data/assets/mjcf/anatomic_soma.xml"),None)
        m=self.m; self.d=mujoco.MjData(m)
        self.q0=m.qpos0.copy(); q=self.q0
        q[:3]=[0,0,1]; q[3:7]=[1,0,0,0]
        for side,sign in (("Right",1),("Left",-1)):
            for name,v in ((side+"Shin_x",sign*.12),(side+"Leg_x",-sign*.06),
                           (side+"Foot_x",-sign*.06),(side+"Arm_y",-sign*1.45),
                           (side+"ForeArm_z",sign*.25)):
                q[m.joint(name).qposadr[0]]=v
        self.feet=[m.body(n).id for n in ("LeftFoot","RightFoot")]
        self.d.qpos[:]=q; mujoco.mj_forward(m,self.d)
        floor=m.geom("floor").id
        ids=[m.body(n).id for n in ("LeftFoot","LeftToeBase","RightFoot","RightToeBase")]
        gs=np.flatnonzero(np.isin(m.geom_bodyid,ids))
        q[2]+=.002-min(mujoco.mj_geomDistance(m,self.d,floor,int(g),10,None) for g in gs)
        self.d.qpos[:]=q; mujoco.mj_forward(m,self.d)
        self.foot0=self.d.xpos[self.feet].copy(); self.rot0=self.d.xquat[self.feet].copy()
        self.lower=[j for j in range(m.njnt) if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE
                    and any(k in (m.joint(j).name or "") for k in ("Leg_","Shin_","Foot_","ToeBase_"))]
        self.cols=m.jnt_dofadr[self.lower];self.qcols=m.jnt_qposadr[self.lower]
        self.body_ids=[m.body(n).id for n in self.robot.kinematic_info.body_names]
        self.dof_qcols=[m.joint(n).qposadr[0] for n in self.robot.kinematic_info.dof_names]
        self.max_error=0.

    def solve(self,q,root,targets,rotations,orientation=None):
        m,d=self.m,self.d
        q=q.copy();q[:3]=root;q[3:7]=self.q0[3:7] if orientation is None else orientation
        for _ in range(35):
            d.qpos[:]=q;mujoco.mj_forward(m,d)
            es=[];js=[]
            for i,b in enumerate(self.feet):
                jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
                mujoco.mj_jacBody(m,d,jp,jr,b)
                er=Rotation.from_quat(rotations[i],scalar_first=True)*Rotation.from_quat(d.xquat[b],scalar_first=True).inv()
                es.extend([targets[i]-d.xpos[b],.15*er.as_rotvec()])
                js.extend([jp[:,self.cols],.15*jr[:,self.cols]])
            e=np.concatenate(es);J=np.concatenate(js)
            if abs(e).max()<2e-5:break
            delta=J.T@np.linalg.solve(J@J.T+np.eye(12)*1e-5,e)
            q[self.qcols]=np.clip(q[self.qcols]+np.clip(delta,-.12,.12),
                m.jnt_range[self.lower,0],m.jnt_range[self.lower,1])
        d.qpos[:]=q;mujoco.mj_forward(m,d)
        self.max_error=max(self.max_error,float(np.linalg.norm(targets-d.xpos[self.feet],axis=1).max()))
        return q

    def arrays(self,qs,contacts):
        qs=np.asarray(qs);ps=[];rots=[]
        for q in qs:
            self.d.qpos[:]=q;mujoco.mj_forward(self.m,self.d)
            ps.append(self.d.xpos[self.body_ids].copy());rots.append(self.d.xquat[self.body_ids].copy())
        ps=np.array(ps);rots=np.array(rots);gavs=np.zeros_like(ps)
        for b in range(len(self.body_ids)):
            rr=Rotation.from_quat(rots[:,b],scalar_first=True)
            rv=(rr[1:]*rr[:-1].inv()).as_rotvec()*30
            gavs[1:,b]=rv;gavs[0,b]=rv[0]
        dps=qs[:,self.dof_qcols]
        return dict(gts=ps,grs=np.roll(rots,-1,axis=-1),gvs=np.gradient(ps,1/30,axis=0),
            gavs=gavs,dps=dps,dvs=np.gradient(dps,1/30,axis=0),contacts=np.array(contacts))

    def walking(self,period=2.4,steps=6,stride=.28,inward=.035,lift=.14,support_center=False,forward_sync=False,right_inward=None,settle=3.,dynamic_root=False,natural_swing=False):
        self.max_error=0.
        duration=1.+steps*period+settle
        times=np.arange(round(duration*30)+1)/30
        from scipy.interpolate import CubicSpline
        mid_x=np.r_[0.,stride*(np.arange(1,steps+1)-.5)]
        forward_curve=CubicSpline(np.arange(steps+1)*period,mid_x,bc_type=((1,0.),(1,0.))) if steps else None
        omega=np.sqrt(9.81/max(.3,self.q0[2]-.08))
        qs=[];cs=[];q=self.q0.copy()
        for t in times:
            root=self.q0[:3].copy();root[2]-=.04*smooth(t/.8)
            targets=self.foot0.copy();rots=self.rot0.copy();contact=np.zeros(53,dtype=bool)
            foot_on=[True,True]
            for k in range(steps):
                i=k%2;sgn=1 if i==0 else -1
                inward_i=inward if i==0 or right_inward is None else right_inward
                u=(t-1-k*period)/period
                x0=targets[i,0];x1=self.foot0[i,0]+stride*(k+1)
                if u>=1:
                    targets[i,0]=x1;targets[i,1]=self.foot0[i,1]-sgn*inward_i
                    continue
                if u<0:break
                old_mid=targets[:,:2].mean(axis=0)
                end=targets.copy();end[i,0]=x1;end[i,1]=self.foot0[i,1]-sgn*inward_i
                new_mid=end[:,:2].mean(axis=0)
                zup=(smooth((u-.12)/.2)-smooth((u-.74)/.2)) if natural_swing else (smooth((u-.25)/.17)-smooth((u-.75)/.15))
                travel=smooth((u-.2)/.52) if natural_swing else smooth((u-.42)/.33)
                targets[i,:2]+=(end[i,:2]-targets[i,:2])*travel
                targets[i,2]+=lift*zup
                rots[i]=(Rotation.from_rotvec([0,-.20*zup,0])*Rotation.from_quat(self.rot0[i],scalar_first=True)).as_quat(scalar_first=True)
                root[:2]=old_mid+(new_mid-old_mid)*smooth(u if forward_sync else (u-.35)/.65)
                lateral=(.9*(targets[1-i,1]-old_mid[1])) if support_center else -sgn*.09
                if dynamic_root:
                    root[0]=self.q0[0]+float(forward_curve(np.clip(t-1,0,steps*period)))
                    mid_y=old_mid[1]+(new_mid[1]-old_mid[1])*smooth(u)
                    shape=1-np.cosh(omega*period*(u-.5))/np.cosh(omega*period*.5)
                    if k==0:shape*=smooth(u/.15)
                    if k==steps-1:shape*=1-smooth((u-.85)/.15)
                    root[1]=mid_y+(targets[1-i,1]-mid_y)*shape
                else:root[1]+=lateral*(smooth(u/.25)-smooth((u-.9)/.1))
                # Opposed shoulder rotations with mirrored anatomical axes.
                for side in ("Left","Right"):
                    a=self.m.joint(side+"Arm_x").qposadr[0]
                    q[a]=self.q0[a]-(1 if i==0 else -1)*.18*np.sin(np.pi*np.clip((u-.25)/.65,0,1))**2
                foot_on[i]=not((.12<u<.94) if natural_swing else (.25<u<.9))
                break
            else:
                root[:2]=targets[:,:2].mean(axis=0)
            if t<1:root[:2]=self.q0[:2]
            for i,side in enumerate(("Left","Right")):
                for part in ("Foot","ToeBase"):
                    contact[list(self.robot.kinematic_info.body_names).index(side+part)]=foot_on[i]
            q=self.solve(q,root,targets,rots)
            qs.append(q);cs.append(contact)
        return self.arrays(qs,cs),dict(period_s=period,steps_requested=steps,stride_m=stride,
            nominal_speed_m_s=stride/period,duration_s=float(times[-1]),inward_m=inward,lift_m=lift,
            max_ik_error_m=self.max_error,support_center=support_center,forward_sync=forward_sync,
            right_inward_m=right_inward,settle_s=settle,walking_end_s=1+steps*period,dynamic_root=dynamic_root,natural_swing=natural_swing)

def pulse_force(t,dt,start,duration,impulse,direction):
    if duration<=0 or dt<=0 or impulse<0:raise ValueError("Invalid pulse duration, timestep or impulse")
    direction=np.asarray(direction,dtype=float)
    if direction.shape!=(3,) or not np.isclose(np.linalg.norm(direction),1.):raise ValueError("Direction must be a unit 3-vector")
    overlap=max(0.,min(t+dt,start+duration)-max(t,start))
    return direction*impulse/duration*(overlap/dt)

def save_pack(chunks,path):
    import torch
    n=[len(c["gts"]) for c in chunks]
    pack={k:torch.from_numpy(np.concatenate([c[k] for c in chunks])).to(
        dtype=torch.bool if k=="contacts" else torch.float32) for k in chunks[0]}
    pack.update(motion_num_frames=torch.tensor(n),motion_dt=torch.ones(len(n))/30,
        motion_lengths=torch.tensor([(v-1)/30 for v in n]),motion_weights=torch.ones(len(n))/len(n),
        motion_files=tuple("own_gait_"+str(i)+".reference" for i in range(len(n))),
        length_starts=torch.tensor(np.cumsum([0]+n[:-1])),lrs=None)
    path.parent.mkdir(parents=True,exist_ok=True);torch.save(pack,path)

def run(root,pack,out,cases,planner_factory=None,same_initial=False,warmup=0.):
    import torch
    agent=create_agent(root,pack);env=agent.env;sim=env.simulator;m=sim.model;d=sim.data
    original=mujoco.mj_step;results=[];base_solref=m.geom_solref.copy();base_noslip=m.opt.noslip_iterations;base_cone=m.opt.cone
    library=env.reference_motion.motion_lib
    base_library={k:getattr(library,k).clone() for k in ("gts","grs","gvs","gavs","dps","dvs","contacts")}
    initial=None;spawn=None;env_snapshot=None
    if any(module.training for module in agent.model.modules()):
        raise RuntimeError("Inference contains training-mode modules")
    if warmup:
        mujoco.mj_resetData(m,d);obs=reset_clip(agent,0);d.time=0.
        preparation=Telemetry(m,d)
        def prepare_step(model,data,*args,**kwargs):
            original(model,data,*args,**kwargs)
            if model is m:preparation.sample()
        mujoco.mj_step=prepare_step
        try:
            for k in range(round(warmup/env.dt)):
                a=action(agent,obs,k)
                with torch.no_grad():obs,*_=env.step(a)
        finally:mujoco.mj_step=original
        prep=preparation.finish(dict(id="physical_preparation",kind="posture"),"protomotions",out/"preparation",warmup)
        # Local trial clock only. Positions and velocities remain simulated truth.
        d.time=0.
        initial=np.zeros(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(m,d,initial,mujoco.mjtState.mjSTATE_INTEGRATION)
        spawn=env._spawn.respawn_root_offset.clone();env_snapshot=env.save_state()
    try:
        for case in cases:
            torch.manual_seed(20261116);np.random.seed(20261116)
            for key,value in base_library.items():getattr(library,key).copy_(value)
            m.geom_solref[:]=base_solref
            m.opt.noslip_iterations=case.get("noslip_iterations",base_noslip)
            m.opt.cone=case.get("cone",base_cone)
            if "contact_time_constant" in case:m.geom_solref[:,0]=case["contact_time_constant"]
            mujoco.mj_resetData(m,d);obs=reset_clip(agent,case["clip"]);d.time=0.
            if same_initial:
                state_type=mujoco.mjtState.mjSTATE_INTEGRATION
                if initial is None:
                    initial=np.zeros(mujoco.mj_stateSize(m,state_type));mujoco.mj_getState(m,d,initial,state_type)
                    spawn=env._spawn.respawn_root_offset.clone();env_snapshot=env.save_state()
                else:
                    env.restore_state(env_snapshot)
                    mujoco.mj_setState(m,d,initial,state_type);mujoco.mj_forward(m,d)
                    mujoco.mj_setState(m,d,initial,state_type);env._spawn.respawn_root_offset.copy_(spawn)
                    if int(env.reference.ids[0])!=case["clip"]:
                        env.reference.assign(torch.tensor([0]),ids=torch.tensor([case["clip"]]),times=torch.zeros(1))
                        env.observe(torch.tensor([0]))
                    obs=env.get_obs()
            initial_observations={k:v.detach().numpy().copy() for k,v in obs.items()}
            planner=planner_factory(env,case) if planner_factory else None
            from support_feedback import SupportFeedback
            support=SupportFeedback(env) if case.get("support_feedback") else None
            tel=Telemetry(m,d);qvels=[];forces=[];loads=[];started=time.perf_counter()
            def measured(model,data,*args,**kwargs):
                if model is not m:return original(model,data,*args,**kwargs)
                data.xfrc_applied[:]=0
                t=data.time;start=case.get("hit_time",3.);dur=case.get("hit_duration",.08)
                overlap=max(0.,min(t+m.opt.timestep,start+dur)-max(t,start))
                if case.get("impulse_ns",0) and overlap:
                    data.xfrc_applied[m.body("Chest").id,:3]=pulse_force(t,m.opt.timestep,start,dur,case["impulse_ns"],case["direction"])
                original(model,data,*args,**kwargs);tel.sample()
                if (tel.idx-1)%8==0:
                    qvels.append(data.qvel.copy());forces.append(data.xfrc_applied[m.body("Chest").id].copy())
                    loads.append(data.actuator_force.copy())
            mujoco.mj_step=measured
            for k in range(round(case["duration_s"]/env.dt)):
                tick=time.perf_counter()
                if planner and planner.update(tel):
                    env.observe(torch.tensor([0]));obs=env.get_obs()
                a=action(agent,obs,k)
                if support:a=support.apply(a,tel)
                tel.control_latencies.append(time.perf_counter()-tick)
                with torch.no_grad():obs,*_=env.step(a)
                if not np.isfinite(d.qpos).all():raise RuntimeError("Nonfinite physical state")
            p=out/case["id"]
            result=tel.finish(dict(case,id=case["id"],kind=case.get("kind","gait")),"protomotions",p,time.perf_counter()-started)
            np.savez_compressed(p/"initial-observations.npz",**initial_observations)
            np.savez_compressed(p/"dynamics.npz",qvel=np.array(qvels),chest_force=np.array(forces),joint_torque=np.array(loads))
            result.update(physical_preparation_s=warmup,network_evaluation_mode=True,supplementary_stance_force_bound_n=60 if support else 0,supplementary_stance_force_max_n=support.max_force if support else 0,planner_events=planner.events if planner else [],scenario=case,physical_model_sha256=physical_fingerprint(m),
                reference_sha256=hashlib.sha256(pack.read_bytes()).hexdigest(),
                initial_state_sha256=hashlib.sha256(tel.initial_state.tobytes()).hexdigest())
            (p/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
            review=analyse(p);results.append(dict(result,review=review))
            print("GAIT_RESULT",json.dumps(dict(case=case["id"],**review)),flush=True)
            mujoco.mj_step=original
    finally:
        mujoco.mj_step=original;sim.shutdown()
    (out/"results.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
    return results

def analyse(path):
    from analyse_natural import minimum_sole_clearance
    s=json.loads((path/"summary.json").read_text(encoding="utf-8"))
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"));d=mujoco.MjData(m)
    with np.load(path/"recording.npz") as a:frames=a["frames"];rows=a["rows"];q0=a["initial_qpos"]
    ids=[m.body(n).id for n in ("LeftFoot","RightFoot")]
    groups=[[m.body(side+n).id for n in ("Foot","ToeBase")] for side in ("Left","Right")]
    gs=[np.flatnonzero(np.isin(m.geom_bodyid,g)&((m.geom_contype|m.geom_conaffinity)!=0)) for g in groups]
    events=[];active=[None,None];dwell=[0.,0.];last=[None,None];series=[]
    floor=m.geom("floor").id;bw=s["mass_kg"]*9.81
    d.qpos[:]=q0;mujoco.mj_forward(m,d);initial=d.xpos[ids].copy()
    for f in frames:
        t=f[0];d.qpos[:]=f[1:];mujoco.mj_forward(m,d)
        r=rows[np.argmin(abs(rows[:,0]-t))]
        clear=np.array([minimum_sole_clearance(m,d,floor,g) for g in gs])
        healthy=r[9]>.7*s["initial_root_height_m"] and r[10]<np.deg2rad(25) and r[11]==0
        for i in range(2):
            good=healthy and clear[i]>=.03 and r[7+i]<.05*bw and r[8-i]>.6*bw
            dwell[i]=dwell[i]+1/120 if good else 0.
            if dwell[i]>=.03 and active[i] is None:
                active[i]=dict(foot=i,lift_s=float(t),from_xy=(initial[i,:2] if last[i] is None else last[i]).tolist())
            if active[i] and t>active[i]["lift_s"]+.05 and healthy and clear[i]<.01 and r[7+i]>.1*bw:
                ev=active[i];ev["land_s"]=float(t);ev["to_xy"]=d.xpos[ids[i],:2].tolist()
                ev["delta_xy"]=(np.array(ev["to_xy"])-ev["from_xy"]).tolist()
                if np.linalg.norm(ev["delta_xy"])>=.06:events.append(ev)
                last[i]=d.xpos[ids[i],:2].copy();active[i]=None;dwell[i]=0
        series.append([t,*clear,*d.xpos[ids,:2].ravel()])
    final=rows[rows[:,0]>=rows[-1,0]-2]
    stable=bool(len(final)>1900 and np.all(final[:,9]>.7*s["initial_root_height_m"]) and
        np.all(final[:,10]<np.deg2rad(25)) and np.all(final[:,11]==0) and
        np.all(np.linalg.norm(final[:,4:6],axis=1)<.12) and np.all(final[:,7:9].sum(axis=1)>.6*bw))
    moving=rows[(rows[:,0]>=1)&(rows[:,0]<=s["scenario"].get("walking_end_s",s["duration_s"]-3))]
    alternating=all(a["foot"]!=b["foot"] for a,b in zip(events,events[1:]))
    review=dict(steps=len(events),alternating=alternating,events=events,stable_final_two_seconds=stable,
        fell=bool(np.any(rows[:,9]<.5*s["initial_root_height_m"])),
        actual_progress_m=float(rows[-1,1]-rows[0,1]),
        mean_forward_speed_m_s=float((moving[-1,1]-moving[0,1])/(moving[-1,0]-moving[0,0])) if len(moving)>1 else 0.,
        mean_abs_lateral_speed_m_s=float(np.mean(abs(moving[:,5]))) if len(moving) else 0.,
        positive_joint_work_j=s["positive_work_j"],artifacts=s["artifacts"],
        passed=bool(stable and alternating and len(events)>=s["scenario"].get("steps_requested",1) and not s["artifacts"]))
    np.savez_compressed(path/"sole-series.npz",series=np.array(series))
    (path/"review.json").write_text(json.dumps(review,indent=2),encoding="utf-8")
    return review

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",default="out/gait/probe")
    p.add_argument("--period",type=float,default=2.4);p.add_argument("--steps",type=int,default=4)
    p.add_argument("--inward",type=float,default=.035);p.add_argument("--prepare-only",action="store_true")
    p.add_argument("--support-center",action="store_true");p.add_argument("--forward-sync",action="store_true");p.add_argument("--stride",type=float,default=.28)
    args=p.parse_args();root=Path(__file__).resolve().parents[2];out=(root/args.output).resolve()
    builder=ReferenceBuilder(root);chunk,meta=builder.walking(args.period,args.steps,inward=args.inward,stride=args.stride,support_center=args.support_center,forward_sync=args.forward_sync)
    save_pack([chunk],out/"references.pt")
    (out/"reference.json").write_text(json.dumps(meta,indent=2),encoding="utf-8")
    if not args.prepare_only:run(root,out/"references.pt",out,[dict(meta,id="walk",clip=0)])
