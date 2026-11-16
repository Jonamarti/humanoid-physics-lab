"""Contact-triggered loading response expressed as coherent IK references.

Muscle-function analogue only: the plant still has joint motors, not muscles.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from cases import smooth

def experiment_output(root,default,output=None):
    path=(root/(output or default)).resolve()
    if path.exists() and any(path.iterdir()):
        raise FileExistsError(f"Preserved experiment exists: {path}. Choose a new --output.")
    return path

def acceptance(t,start,rise=.14,hold=.02,recover=.22):
    t=np.asarray(t)
    return smooth((t-start)/rise)-smooth((t-start-rise-hold)/recover)

def priming_depth(times,period,steps,depth=.003):
    value=np.zeros_like(times,dtype=float)
    for k in range(steps):
        u=(times-1-k*period)/period
        value+=depth*(smooth((u-.70)/.20)-smooth((u-1.35)/.35))
    return np.minimum(value,depth)

def qpos_from_clip(builder,clip):
    q=np.repeat(builder.q0[None,:],len(clip["gts"]),axis=0)
    q[:,:3]=clip["gts"][:,0];q[:,3:7]=np.roll(clip["grs"][:,0],1,axis=-1)
    q[:,builder.dof_qcols]=clip["dps"]
    return q

def lower_reference(builder,clip,depth):
    qbase=qpos_from_clip(builder,clip);out=[]
    ids=[list(builder.robot.kinematic_info.body_names).index(side+"Foot") for side in ("Left","Right")]
    heights=np.broadcast_to(np.asarray(depth),len(qbase))
    builder.max_error=0.
    for idx,q in enumerate(qbase):
        root=q[:3].copy();root[2]-=heights[idx]
        rotations=np.roll(clip["grs"][idx,ids],1,axis=-1)
        out.append(builder.solve(q,root,clip["gts"][idx,ids],rotations,orientation=q[3:7]))
    result=builder.arrays(out,clip["contacts"])
    return result,qbase,np.array(out),builder.max_error

def validate_blend(builder,base,qbase,qlow):
    ids=[list(builder.robot.kinematic_info.body_names).index(side+"Foot") for side in ("Left","Right")]
    maximum=0.
    for weight in (.25,.5,.75):
        q=qbase+(qlow-qbase)*weight
        generated=builder.arrays(q,base["contacts"])
        maximum=max(maximum,float(np.linalg.norm(generated["gts"][:,ids]-base["gts"][:,ids],axis=-1).max()))
    if maximum>.002:raise RuntimeError("Yield blend moves reference feet by more than2mm")
    return maximum

class LoadAcceptance:
    def __init__(self,env,case,builder,clips,depth):
        self.env=env;self.case=case;self.builder=builder;self.events=[]
        self.depth=float(depth);self.enabled=case.get("load_acceptance",False)
        self.index=0;self.unloaded=np.zeros(2);self.armed=[False,False]
        self.last_event=np.full(2,-10.)
        self.reference_updates=0
        lib=env.reference_motion.motion_lib;motion=int(env.reference.ids[0])
        self.start=int(lib.length_starts[motion]);self.count=int(lib.motion_num_frames[motion])
        self.raw=clips[motion];self.times=np.arange(self.count)/30
        self.frozen={k:getattr(lib,k)[self.start:self.start+self.count].clone()
                     for k in ("gts","grs","gvs","gavs","dps","dvs")}
        self.base_q=qpos_from_clip(builder,self.raw)
        self.low_q=None;self.blend_error=0.
        if self.enabled:
            _,_,self.low_q,ik_error=lower_reference(builder,self.raw,self.depth)
            if ik_error>.002:raise RuntimeError("Yield goal is unreachable")
            self.blend_error=validate_blend(builder,self.raw,self.base_q,self.low_q)

    def update(self,tel):
        if not self.enabled:return False
        c=np.asarray(tel.contact_rows)
        if not len(c):return False
        pending=[]
        bw=tel.mass*9.81
        healthy=tel.rows[-1][9]>.7*tel.initial_height and tel.rows[-1][10]<np.deg2rad(25) and not tel.rows[-1][11]
        for row in c[self.index:]:
            if row[0]<.8:continue
            for i in range(2):
                load=row[1+i];heel=row[3+i]
                self.unloaded[i]=self.unloaded[i]+tel.m.opt.timestep if load<.05*bw else 0.
                if self.unloaded[i]>=.04:self.armed[i]=True
                if healthy and self.armed[i] and load>.15*bw and heel>.08*bw and row[0]-self.last_event[i]>.25:
                    pending.append(dict(foot=i,contact_s=float(row[0]),load_n=float(load),heel_n=float(heel)))
                    self.armed[i]=False;self.last_event[i]=row[0]
        self.index=len(c)
        if self.case.get("qualified_lifts"):
            pending=[dict(e) for e in tel.heel_events[getattr(self,"heel_event_index",0):]]
            self.heel_event_index=len(tel.heel_events)
        if not pending:return False
        # Start from the observed state now, not retrospectively at first contact.
        for event in pending:
            event.update(goal_start_s=float(self.env.reference.times[0]),
                observed_at_physics_s=float(tel.d.time),yield_depth_m=self.depth)
            self.events.append(event)
        self.rebuild()
        return True

    def rebuild(self):
        import torch
        weight=np.zeros(self.count)
        for event in self.events:
            weight+=acceptance(self.times,event["goal_start_s"])
        weight=np.clip(weight,0,1)
        q=self.base_q+(self.low_q-self.base_q)*weight[:,None]
        generated=self.builder.arrays(q,self.raw["contacts"])
        lib=self.env.reference_motion.motion_lib
        for key in ("gts","gvs","gavs","dps","dvs"):
            delta=generated[key]-self.raw[key]
            value=self.frozen[key]+torch.as_tensor(delta,dtype=self.frozen[key].dtype)
            getattr(lib,key)[self.start:self.start+self.count].copy_(value)
        # Align quaternion signs with the immutable published-library convention.
        quat=generated["grs"].copy()
        signs=np.where(np.sum(self.raw["grs"]*self.frozen["grs"].numpy(),axis=-1)<0,-1.,1.)
        quat*=signs[...,None]
        lib.grs[self.start:self.start+self.count].copy_(torch.as_tensor(quat,dtype=lib.grs.dtype))
        self.reference_updates+=1
        self.final_weight=weight

    def save(self,path):
        meta=dict(enabled=self.enabled,depth_m=self.depth,events=self.events,
            native_load_detection_hz=960,policy_update_hz=30,reference_updates=self.reference_updates,
            max_interpolated_foot_error_m=self.blend_error,
            interpretation="Reference admittance for coordinated knee/hip yield. Not muscle force/activation.",
            profile=dict(rise_s=.14,hold_s=.02,recover_s=.22))
        (path/"acceptance-plan.json").write_text(json.dumps(meta,indent=2),encoding="utf-8")
        if self.enabled:
            np.savez_compressed(path/"acceptance-reference.npz",base_qpos=self.base_q,
                yielded_qpos=self.low_q,times=self.times,weight=getattr(self,"final_weight",np.zeros(self.count)))

def energy_by_joint(path):
    """120Hz diagnostic; native total motor work remains authoritative."""
    s=json.loads((path/"summary.json").read_text(encoding="utf-8"))
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"))
    a=np.load(path/"recording.npz");d=np.load(path/"dynamics.npz")
    time=a["frames"][:,0];dt=1/120
    velocity=d["qvel"][:,m.jnt_dofadr[m.actuator_trnid[:,0]]]
    power=d["joint_torque"]*velocity
    reviews=json.loads((path/"rollover-review.json").read_text(encoding="utf-8"))
    result={}
    for name,parts in (("knee",("Shin_x",)),("hip",("Leg_x",)),("ankle",("Foot_x",))):
        total_negative=0.;total_positive=0.
        for side,foot in (("Left",0),("Right",1)):
            ids=[i for i in range(m.nu) if m.joint(m.actuator_trnid[i,0]).name in [side+p for p in parts]]
            mask=np.zeros(len(time),dtype=bool)
            for event in reviews["strikes"]:
                if event["foot"]==foot:mask|=(time>=event["t"])&(time<event["t"]+.20)
            values=power[mask][:,ids]
            total_negative+=float(-np.minimum(values,0).sum()*dt)
            total_positive+=float(np.maximum(values,0).sum()*dt)
        result[name]=dict(absorbed_j=total_negative,generated_j=total_positive)
    (path/"loading-energy.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result

def main(output=None):
    from heel_toe_suite import prepare
    from heel_toe import save_pack,run,RolloverTelemetry,analyse_rollover
    from heel_toe_model import HeelToeModel
    from landing_feedback import LandingFeedback
    root=Path(__file__).resolve().parents[2];out=experiment_output(root,"out/load-response/phase-v1",output)
    b,_,_,gaze,meta=prepare(root,.5,6)
    times=np.arange(len(gaze["gts"]))/30
    primed,_,_,error=lower_reference(b,gaze,priming_depth(times,.5,6))
    if error>.002:raise RuntimeError("Primed reference unreachable")
    clips=[gaze,primed];save_pack(clips,out/"references.pt")
    common=dict(meta,upper_damping=True,leg_absorption=True,landing_feedback=True,
        landing_mode="replace",foot_contact_tau=.01)
    cases=[dict(common,id="before",clip=0),
        dict(common,id="load_only",clip=0,load_acceptance=True,yield_depth=.008),
        dict(common,id="primed_load",clip=1,load_acceptance=True,yield_depth=.005,priming_depth=.003)]
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    (out/"hypothesis.txt").write_text(
        "Preserve heel-to-toe and contact10ms. The root-height reference was effectively flat after setup; globally increasing damping did not remove torso vibration. Human quadriceps/knee negative work at weight acceptance and ankle/hip phase coordination suggest controlled yield, not a rigid trunk. Compare nominal, heel-load-triggered8mm reference admittance, then the same maximum8mm split into3mm anticipatory priming plus5mm measured-load response. Foot world goals, friction, gains, cadence, motor limits and full initial state stay fixed. No runtime body pose writes. Reject loss of heel-first contacts/six alternating steps, greater torso/head vibration, energy-driven diving or physical artifacts. Loading negative joint work is measured but is not muscle recruitment.",encoding="utf-8")
    factory=lambda env,case:LoadAcceptance(env,case,b,clips,case.get("yield_depth",0.))
    run(root,out/"references.pt",out,cases,same_initial=True,planner_factory=factory,
        telemetry_factory=RolloverTelemetry,model_configurator=HeelToeModel(),action_filter_factory=LandingFeedback)
    for case in cases:
        p=out/case["id"];analyse_rollover(p)
        print("LOADING_ENERGY",case["id"],json.dumps(energy_by_joint(p)),flush=True)
if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument("--output")
    main(parser.parse_args().output)
