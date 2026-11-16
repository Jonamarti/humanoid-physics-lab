"""Heel/forefoot goals and native contact telemetry, without kinematic execution."""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfiltfilt
from gait import ReferenceBuilder,save_pack,run
from cases import smooth
from metrics import Telemetry

BASELINE_COMMIT="2b8d963"

def roll_angle(t,side,period,steps,heel_strike=-.14):
    """Positive = heel up; negative = toes up. Six alternating swings."""
    scheduled=list(range(side,steps,2))
    starts=[1+k*period for k in scheduled]
    if not starts:return 0.
    # Every swing follows toe-off, swing clearance, heel contact, foot lowering.
    upcoming=next((s for s in starts if t<s+.12*period),None)
    if upcoming is not None and t>=upcoming-.35*period:
        return .20*float(smooth((t-(upcoming-.35*period))/(.47*period)))
    prior=[s for s in starts if t>=s+.12*period]
    if not prior:return 0.
    u=(t-prior[-1])/period
    if u<.40:return .20+(-.20-.20)*float(smooth((u-.12)/.28))
    if u<.72:return -.20
    if u<.94:return -.20+(heel_strike+.20)*float(smooth((u-.72)/.22))
    if u<1.24:return heel_strike*(1-float(smooth((u-.94)/.30)))
    return 0.

def local_heel(builder,i):
    m,d=builder.m,builder.d
    b=builder.feet[i];R=d.xmat[b].reshape(3,3)
    points=[]
    for g in np.flatnonzero(m.geom_bodyid==b):
        if not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
        if m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_CAPSULE:raise RuntimeError("Expected capsule sole")
        axis=d.geom_xmat[g].reshape(3,3)[:,2]
        for sign in (-1,1):
            point=d.geom_xpos[g]+sign*axis*m.geom_size[g,1]-[0,0,m.geom_size[g,0]]
            points.append(R.T@(point-d.xpos[b]))
    return min(points,key=lambda p:p[0])

def rollover(builder,base,meta,heel_strike=-.14):
    m,d=builder.m,builder.d
    d.qpos[:]=builder.q0;mujoco.mj_forward(m,d)
    heels=[local_heel(builder,i) for i in range(2)]
    toe_bodies=[m.body(s+"ToeBase").id for s in ("Left","Right")]
    pivots=[d.xmat[b].reshape(3,3).T@(d.xpos[toe]-d.xpos[b])
        for b,toe in zip(builder.feet,toe_bodies)]
    groups=[[m.body(side+p).id for p in ("Foot","ToeBase")] for side in ("Left","Right")]
    geom_groups=[np.flatnonzero(np.isin(m.geom_bodyid,g)&((m.geom_contype|m.geom_conaffinity)!=0)) for g in groups]
    foot_indices=[list(builder.robot.kinematic_info.body_names).index(side+"Foot") for side in ("Left","Right")]
    qs=[];contacts=base["contacts"].copy();builder.max_error=0.
    floor=m.geom("floor").id
    for frame in range(len(base["gts"])):
        t=frame/30
        q=builder.q0.copy();q[:3]=base["gts"][frame,0]
        q[3:7]=np.roll(base["grs"][frame,0],1)
        q[builder.dof_qcols]=base["dps"][frame]
        targets=base["gts"][frame,foot_indices].copy()
        rotations=[]
        for i,side in enumerate(("Left","Right")):
            theta=roll_angle(t,i,meta["period_s"],meta["steps_requested"],heel_strike)
            r0=Rotation.from_quat(builder.rot0[i],scalar_first=True)
            rotation=Rotation.from_rotvec([0,theta,0])*r0
            # Smoothly blend pivots around zero angle; no ankle-position jump.
            heel_weight=float(smooth((.06-theta)/.12))
            pivot=(1-heel_weight)*pivots[i]+heel_weight*heels[i]
            targets[i]+=(r0.as_matrix()-rotation.as_matrix())@pivot
            rotations.append(rotation.as_quat(scalar_first=True))
            # MTP articulation keeps the toe segment planted during heel rise.
            q[m.joint(side+"ToeBase_x").qposadr[0]]=builder.q0[m.joint(side+"ToeBase_x").qposadr[0]]+(1 if i==0 else -1)*max(0.,theta)
        q=builder.solve(q,q[:3],targets,rotations,orientation=q[3:7])
        # Ground clearance is a goal constraint, not a force or a runtime pose write.
        clearance=[min(mujoco.mj_geomDistance(m,d,floor,int(g),10,None) for g in gs) for gs in geom_groups]
        correction=np.maximum(0.,.002-np.array(clearance))
        if np.max(correction)>1e-6:
            targets[:,2]+=correction
            q=builder.solve(q,q[:3],targets,rotations,orientation=q[3:7])
        for i,side in enumerate(("Left","Right")):
            for part in ("Foot","ToeBase"):
                bid=m.body(side+part).id
                gs=np.flatnonzero((m.geom_bodyid==bid)&((m.geom_contype|m.geom_conaffinity)!=0))
                gap=min(mujoco.mj_geomDistance(m,d,floor,int(g),10,None) for g in gs)
                contacts[frame,list(builder.robot.kinematic_info.body_names).index(side+part)]=gap<.006
        qs.append(q)
    return builder.arrays(qs,contacts),dict(meta,rollover=True,heel_strike_rad=heel_strike,max_ik_error_m=builder.max_error)

def lean_reference(builder,base,meta,angle=.10):
    """Whole-body reference lean; feet retain identical world-space goals."""
    ids=[list(builder.robot.kinematic_info.body_names).index(side+"Foot") for side in ("Left","Right")]
    qs=[];builder.max_error=0.
    for frame in range(len(base["gts"])):
        t=frame/30
        ramp=float(smooth((t-.4)/.5)*(1-smooth((t-meta["walking_end_s"])/1.)))
        rotation=Rotation.from_rotvec([0,angle*ramp,0])
        q=builder.q0.copy();q[:3]=base["gts"][frame,0]
        q[3:7]=(rotation*Rotation.from_quat(base["grs"][frame,0])).as_quat(scalar_first=True)
        q[builder.dof_qcols]=base["dps"][frame]
        targets=base["gts"][frame,ids]
        rots=np.roll(base["grs"][frame,ids],1,axis=-1)
        q=builder.solve(q,q[:3],targets,rots,orientation=q[3:7])
        qs.append(q)
    return builder.arrays(qs,base["contacts"]),dict(meta,lean_rad=angle,max_ik_error_m=builder.max_error)

def gaze_reference(builder,base,meta):
    """Counterrotate neck/head goals to retain an approximately level gaze."""
    qs=[]
    for frame in range(len(base["gts"])):
        t=frame/30;ramp=float(smooth((t-.4)/.5)*(1-smooth((t-meta["walking_end_s"])/1.)))
        q=builder.q0.copy();q[:3]=base["gts"][frame,0];q[3:7]=np.roll(base["grs"][frame,0],1)
        q[builder.dof_qcols]=base["dps"][frame]
        for name,fraction in (("Neck1_x",.6),("Head_x",.4)):
            q[builder.m.joint(name).qposadr[0]]-=fraction*meta["lean_rad"]*ramp
        qs.append(q)
    return builder.arrays(qs,base["contacts"]),dict(meta,gaze_counterrotation=True)

class RolloverTelemetry(Telemetry):
    def __init__(self,m,d):
        super().__init__(m,d)
        self.contact_rows=[];self.upper_rows=[]
        self.segments=[m.body(n).id for n in ("Hips","Chest","Head")]
    def sample(self):
        super().sample()
        m,d=self.m,self.d;t=d.time-self.start
        weights=np.zeros(2);cop=np.zeros((2,3));heel=np.zeros(2);fore=np.zeros(2)
        for idx,c in enumerate(d.contact):
            if c.efc_address<0:continue
            b1,b2=m.geom_bodyid[[c.geom1,c.geom2]]
            if b1!=0 and b2!=0:continue
            b=int(b2 if b1==0 else b1)
            force=np.zeros(6);mujoco.mj_contactForce(m,d,idx,force)
            world=np.asarray(c.frame).reshape(3,3).T@force[:3]
            if b2==0:world=-world
            normal=max(0.,world[2])
            for i,g in enumerate(self.foot_groups):
                if b in g:
                    weights[i]+=normal;cop[i]+=normal*c.pos
                    x=(d.xmat[self.feet[i]].reshape(3,3).T@(c.pos-d.xpos[self.feet[i]]))[0]
                    if x<.045:heel[i]+=normal
                    else:fore[i]+=normal
        local_cop=np.full(2,np.nan);pitch=[]
        for i,b in enumerate(self.feet):
            R=d.xmat[b].reshape(3,3)
            if weights[i]>1:local_cop[i]=(R.T@(cop[i]/weights[i]-d.xpos[b]))[0]
            pitch.append(float(Rotation.from_matrix(R).as_euler("xyz")[1]))
        self.contact_rows.append([t,*weights,*heel,*fore,*local_cop,*pitch])
        if (self.idx-1)%8==0:
            data=[]
            for b in self.segments:
                jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
                mujoco.mj_jacBody(m,d,jp,jr,b)
                angles=Rotation.from_matrix(d.xmat[b].reshape(3,3)).as_euler("xyz")
                data.extend([*d.xpos[b],*(jp@d.qvel),*angles,*(jr@d.qvel)])
            feet=d.xpos[self.feet,:2].ravel()
            self.upper_rows.append([t,*data,*feet])
    def finish(self,case,controller,out,wall_s):
        result=super().finish(case,controller,out,wall_s)
        np.savez_compressed(out/"rollover.npz",contacts=np.array(self.contact_rows),upper=np.array(self.upper_rows))
        return result

def first_dwell(times,mask,duration=.08):
    start=None
    for t,ok in zip(times,mask):
        if ok:
            if start is None:start=t
            if t-start>=duration:return float(start)
        else:start=None
    return None

def analyse_rollover(path):
    s=json.loads((path/"summary.json").read_text(encoding="utf-8"))
    r=json.loads((path/"review.json").read_text(encoding="utf-8"))
    a=dict(np.load(path/"rollover.npz"));u=a["upper"];c=a["contacts"]
    raw=np.load(path/"recording.npz")["rows"]
    period=s["scenario"]["period_s"];end=s["scenario"]["walking_end_s"]
    selected=(u[:,0]>=1)&(u[:,0]<end)
    band=butter(3,[4,15],btype="bandpass",fs=120,output="sos")
    result=dict(case=s["case"],note="Native solved contacts at960Hz; actual body kinematics at120Hz",steps=r["steps"],alternating=r["alternating"],stable=r["stable_final_two_seconds"],fell=r["fell"],artifacts=s["artifacts"],positive_work_j=s["positive_work_j"],segments={})
    for i,name in enumerate(("pelvis","chest","head")):
        vel=u[:,4+i*12:7+i*12]
        acc=np.gradient(vel,1/120,axis=0);filtered=sosfiltfilt(band,acc,axis=0)
        angles=np.rad2deg(u[:,7+i*12:10+i*12])
        result["segments"][name]=dict(acceleration_band_4_15hz_rms=float(np.sqrt(np.mean(filtered[selected]**2))),
            acceleration_rms=float(np.sqrt(np.mean(acc[selected]**2))),max_acceleration_m_s2=float(np.linalg.norm(acc[selected],axis=1).max()),
            roll_rms_deg=float(np.sqrt(np.mean(angles[selected,0]**2))),pitch_mean_deg=float(np.mean(angles[selected,1])),
            pitch_std_deg=float(np.std(angles[selected,1])),roll_std_deg=float(np.std(angles[selected,0])),
            angular_velocity_band_4_15hz_rms=float(np.sqrt(np.mean(sosfiltfilt(band,u[:,10+i*12:13+i*12],axis=0)[selected]**2))))
    base=np.median(raw[(raw[:,0]>=.6)&(raw[:,0]<=.8),1])
    com=first_dwell(raw[:,0],(raw[:,0]>.8)&(raw[:,1]>base+.01)&(raw[:,4]>.04))
    f0=u[0,-4:].reshape(2,2)
    dx=(u[:,-4:].reshape(-1,2,2)[:,:,0]-f0[:,0]).max(axis=1)
    foot=first_dwell(u[:,0],(u[:,0]>.8)&(dx>.02),.04)
    result["com_forward_onset_s"]=com;result["foot_forward_onset_s"]=foot
    result["com_lead_s"]=foot-com if com is not None and foot is not None else None
    # Only physically completed swing/landing cycles, not brief stance reloads.
    bw=s["mass_kg"]*9.81;strikes=[]
    for event in r["events"]:
        i=event["foot"]
        eligible=c[(c[:,0]>=event["lift_s"]+.05)&(c[:,0]<=event["land_s"]+.04)&(c[:,1+i]>.1*bw)]
        if not len(eligible):continue
        t=eligible[0,0]
        window=c[(c[:,0]>=t)&(c[:,0]<t+.05)]
        h=window[:,3+i].sum();f=window[:,5+i].sum()
        strikes.append(dict(foot=i,t=float(t),heel_fraction_first_50ms=float(h/max(1,h+f)),
            foot_pitch_deg=float(np.rad2deg(window[:,9+i].mean()))))
    for strike in strikes:
        future=[e for e in r["events"] if e["foot"]==strike["foot"] and e["lift_s"]>strike["t"]+.1]
        if not future:continue
        i=strike["foot"];stop=min(e["lift_s"] for e in future)
        loaded=c[(c[:,0]>strike["t"]+.1)&(c[:,0]<stop)&(c[:,1+i]>.1*bw)]
        if not len(loaded):continue
        window=loaded[loaded[:,0]>=loaded[-1,0]-.05]
        h=window[:,3+i].sum();f=window[:,5+i].sum()
        strike["late_forefoot_fraction"]=float(f/max(1,h+f))
    result["completed_heel_to_forefoot_cycles"]=sum(e["heel_fraction_first_50ms"]>.65 and e.get("late_forefoot_fraction",0)>.65 for e in strikes)
    result["cycles_with_next_swing"]=sum("late_forefoot_fraction" in e for e in strikes)
    result["strikes"]=sorted(strikes,key=lambda x:x["t"])
    result["heel_first_strikes"]=sum(x["heel_fraction_first_50ms"]>.65 for x in strikes)
    (path/"rollover-review.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print("ROLLOVER",json.dumps(result),flush=True)
    return result

def main():
    root=Path(__file__).resolve().parents[2];out=root/"out/heel-toe/contact-v1"
    b=ReferenceBuilder(root)
    base,meta=b.walking(.5,6,lift=.18,support_center=True,forward_sync=True,settle=5.,dynamic_root=True,natural_swing=True)
    rolled,rolled_meta=rollover(b,base,meta)
    if rolled_meta["max_ik_error_m"]>.002:raise RuntimeError("Unreachable rollover goals "+str(rolled_meta))
    save_pack([base,rolled],out/"references.pt")
    cases=[dict(meta,id="flat",clip=0),dict(rolled_meta,id="rollover",clip=1)]
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    run(root,out/"references.pt",out,cases,same_initial=True,telemetry_factory=RolloverTelemetry)
    for c in cases:analyse_rollover(out/c["id"])
if __name__=="__main__":main()
