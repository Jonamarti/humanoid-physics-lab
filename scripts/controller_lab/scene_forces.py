"""Finite-mass replay of public Tori tracks; no robot pose driving."""
import json
from pathlib import Path
import numpy as np
import mujoco

def unit(x,fallback=(1.,0,0)):
    x=np.asarray(x,dtype=float);n=np.linalg.norm(x)
    return x/n if n>1e-12 else np.array(fallback,dtype=float)

def two_link(root,end,l1,l2,bend):
    delta=end-root;dist=np.linalg.norm(delta)
    if dist<.05:delta=np.array([0.,0,-.05]);dist=.05
    direction=delta/dist;D=min(dist,l1+l2-1e-4)
    along=(l1*l1-l2*l2+D*D)/(2*D);off=np.sqrt(max(0,l1*l1-along*along))
    side=unit(bend-direction*np.dot(bend,direction),unit(np.cross(direction,[0,1,0])))
    return root+direction*along+side*off,root+direction*D

def segment_command(params,pos,velocity,axis,omega,inertia,now,next_,dt):
    w=40.;target=now.mean(axis=0);future=next_.mean(axis=0)
    force=params["mass_kg"]*(w*w*(target-pos)+2*w*((future-target)/dt-velocity)+[0,0,9.81])
    horizontal=np.linalg.norm(force[:2]);cap=params["max_horizontal_force_n"]
    force[:2]*=min(1,cap/max(horizontal,1e-12))
    force[2]=np.clip(force[2],-params["max_vertical_force_n"],params["max_vertical_force_n"])
    a=unit(now[1]-now[0],axis);b=unit(next_[1]-next_[0],a)
    point=np.linalg.norm(now[1]-now[0])<1e-9
    desired=np.zeros(3) if point else np.cross(a,b)/dt
    err=desired-omega;axial=axis*np.dot(err,axis);transverse=err-axial
    torque=2*w*(inertia[0]*transverse+inertia[1]*axial)
    if not point:torque+=inertia[0]*w*w*np.cross(axis,a)
    torque*=min(1,params["max_torque_nm"]/max(np.linalg.norm(torque),1e-12))
    return force,torque,float(np.linalg.norm(target-pos))

class CoupledGrip:
    def __init__(self,spec):
        self.spec=spec;self.status="waiting";self.rest=0.;self.saturated=0.;self.events=[]
    def force(self,t,dt,hand,hvel,grip,gvel):
        p=self.spec;delta=hand-grip;distance=np.linalg.norm(delta)
        engage=p.get("engage_s",0.);release=p.get("release_s",1e6)
        old=self.status
        if self.status in ("held","waiting") and t>=release:self.status="released"
        elif self.status=="waiting" and t>=engage:
            if distance<=p.get("capture_radius_m",.12):self.status="held";self.rest=distance+.02
            elif t>=engage+p.get("capture_window_s",.35):self.status="missed"
        magnitude=0.
        if self.status=="held":
            if distance>p.get("break_distance_m",.25):self.status="slipped"
            else:
                direction=unit(delta);extension=max(0,distance-self.rest)
                wanted=max(0,2500*extension+80*np.dot(hvel-gvel,direction)) if extension else 0.
                cap=p.get("max_force_n",500.)
                self.saturated=self.saturated+dt if wanted>=cap else 0.
                if self.saturated+1e-12>=p.get("slip_after_s",.2):self.status="slipped"
                else:
                    magnitude=min(wanted,cap)
                    speed=np.dot(direction,gvel)
                    if magnitude*speed>p.get("max_power_w",600.):magnitude=p.get("max_power_w",600.)/speed
        if self.status!=old:self.events.append(dict(t_s=float(t),status=self.status,distance_m=float(distance)))
        return unit(delta)*magnitude

ANCHORS={"nape":("Neck1",[0,0,0]),"sternum":("Chest",[.05,0,0]),
    "right_sleeve_elbow":("RightForeArm",[0,0,0]),"left_sleeve_elbow":("LeftForeArm",[0,0,0]),
    "right_knee_back":("RightShin",[-.025,0,0]),"left_knee_back":("LeftShin",[-.025,0,0])}

def point_state(m,d,body,local):
    point=d.xpos[body]+d.xmat[body].reshape(3,3)@local
    jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
    mujoco.mj_jac(m,d,jp,jr,point,body)
    return point,jp@d.qvel

def wrench(d,body,point,force):
    d.xfrc_applied[body,:3]+=force
    d.xfrc_applied[body,3:]+=np.cross(point-d.xipos[body],force)

class ToriScene:
    def __init__(self,m,d,template,offset,enabled=True,grips=True):
        self.m=m;self.d=d;self.template=template;self.offset=np.asarray(offset)
        self.enabled=enabled;self.grips_enabled=grips;self.lead=2.
        self.params=template["params"];self.targets=np.array(template["targets"]);self.hz=template["hz"]
        self.names=[s["name"] for s in self.params["segments"]]
        self.bodies=[m.body("projectile_scene_"+n).id for n in self.names]
        self.grips=[];self.rows=[];self.max_deviation=0.;self.max_grip_force=0.;self.max_pair_residual=0.
        self.max_contact_force=0.;self.contact_impulse=np.zeros(3);self.contact_time=0.;self.max_actor_penetration=0.
        self.tracking_work=0.;self.grip_work=0.;self.max_horizontal_ratio=0.;self.max_vertical_ratio=0.;self.max_torque_ratio=0.
        self.actor_root=m.body("Hips").id
        for side,spec in template["source"]["tori"]["body"].get("grips",{}).items():
            if spec["site"] not in ANCHORS:raise ValueError("Unsupported semantic grip "+spec["site"])
            name,local=ANCHORS[spec["site"]]
            self.grips.append(dict(side=side,spec=spec,link=CoupledGrip(spec),body=m.body(name).id,local=np.array(local),source=m.body("projectile_scene_tori_"+side+"_forearm").id))
    def target(self,t):
        x=np.clip(t*self.hz,0,len(self.targets)-1.000001);k=int(x);a=x-k
        return (1-a)*self.targets[k]+a*self.targets[k+1]+self.offset
    def closing_targets(self,targets,scene_t):
        targets=targets.copy();left=unit(targets[2,0]-targets[2,1])
        for g in self.grips:
            p=g["spec"]
            if not self.grips_enabled or g["link"].status!="waiting" or not(p.get("engage_s",0)<=scene_t<=p.get("engage_s",0)+p.get("capture_window_s",.35)):continue
            point,_=point_state(self.m,self.d,g["body"],g["local"])
            side=g["side"];up=self.names.index("tori_"+side+"_upperarm");fore=self.names.index("tori_"+side+"_forearm")
            shoulder=targets[up,0];sign=1 if side=="left" else -1
            mid,end=two_link(shoulder,point,self.params["dims"]["upper_arm"],self.params["dims"]["forearm"],np.array([0,0,-1])+.6*sign*left)
            targets[up]=[shoulder,mid];targets[fore]=[mid,end]
        return targets
    def initialize(self):
        # Trial start only; subsequent movement is integrated from finite forces.
        m,d=self.m,self.d
        for b in self.bodies:
            j=m.body_jntadr[b];q=m.jnt_qposadr[j]
            d.qpos[q:q+3]=m.qpos0[q:q+3]+self.offset
        mujoco.mj_forward(m,d)
        targets=self.closing_targets(self.target(0),0.)
        from scipy.spatial.transform import Rotation
        for b,ends in zip(self.bodies,targets):
            j=m.body_jntadr[b];q=m.jnt_qposadr[j]
            axis=unit(ends[1]-ends[0],[0,0,1]);z=axis[2]
            quat=np.array([0,1,0,0]) if z<-1+1e-9 else np.array([np.sqrt((1+z)/2),-axis[1]/np.sqrt(2*(1+z)),axis[0]/np.sqrt(2*(1+z)),0])
            d.qpos[q:q+3]=ends.mean(axis=0);d.qpos[q+3:q+7]=quat
            d.qvel[m.jnt_dofadr[j]:m.jnt_dofadr[j]+6]=0.
        mujoco.mj_forward(m,d)
    def apply(self,t):
        if not self.enabled:return
        m,d=self.m,self.d;dt=m.opt.timestep;scene_t=max(0.,t-self.lead)
        targets=self.closing_targets(self.target(scene_t),scene_t)
        future=self.closing_targets(self.target(max(0.,t+dt-self.lead)),scene_t)
        for i,(b,p) in enumerate(zip(self.bodies,self.params["segments"])):
            velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,b,velocity,0)
            axis=d.xmat[b].reshape(3,3)[:,2]
            force,torque,deviation=segment_command(p,d.xipos[b],velocity[3:],axis,velocity[:3],
                [max(m.body_inertia[b,:2]),m.body_inertia[b,2]],targets[i],future[i],dt)
            d.xfrc_applied[b,:3]+=force;d.xfrc_applied[b,3:]+=torque
            self.tracking_work+=(np.dot(force,velocity[3:])+np.dot(torque,velocity[:3]))*dt
            self.max_deviation=max(self.max_deviation,deviation)
            self.max_horizontal_ratio=max(self.max_horizontal_ratio,np.linalg.norm(force[:2])/p["max_horizontal_force_n"])
            self.max_vertical_ratio=max(self.max_vertical_ratio,abs(force[2])/p["max_vertical_force_n"])
            self.max_torque_ratio=max(self.max_torque_ratio,np.linalg.norm(torque)/p["max_torque_nm"])
        grip_forces=[]
        for g in self.grips:
            grip,gvel=point_state(m,d,g["body"],g["local"])
            hand,hvel=point_state(m,d,g["source"],np.array([0,0,self.params["dims"]["forearm"]/2]))
            force=g["link"].force(t-self.lead,dt,hand,hvel,grip,gvel) if self.grips_enabled else np.zeros(3)
            wrench(d,g["body"],grip,force);wrench(d,g["source"],hand,-force)
            self.max_grip_force=max(self.max_grip_force,np.linalg.norm(force))
            # Central force at actual endpoints conserves net force and moment.
            residual=np.linalg.norm(np.cross(grip-hand,force))
            self.max_pair_residual=max(self.max_pair_residual,residual)
            self.grip_work+=np.dot(force,gvel-hvel)*dt
            grip_forces.append(float(np.linalg.norm(force)))
        self.rows.append([t,*grip_forces,self.max_deviation])
    def solved(self):
        m,d=self.m,self.d;dt=m.opt.timestep
        for idx,c in enumerate(d.contact):
            b1,b2=m.geom_bodyid[[c.geom1,c.geom2]]
            robot1=m.body_rootid[b1]==self.actor_root;robot2=m.body_rootid[b2]==self.actor_root
            hit=(robot1 and b2 in self.bodies) or (robot2 and b1 in self.bodies)
            if not hit or c.efc_address<0:continue
            force=np.zeros(6);mujoco.mj_contactForce(m,d,idx,force)
            self.max_contact_force=max(self.max_contact_force,float(force[0]))
            world=c.frame.reshape(3,3).T@force[:3]
            self.contact_impulse+=(world if robot2 else -world)*dt
            self.contact_time+=dt;self.max_actor_penetration=max(self.max_actor_penetration,max(0.,-float(c.dist)))
    def save(self,out):
        np.savez_compressed(out/"attacker.npz",rows=np.array(self.rows))
        result=dict(attacker="existing finite-mass independent segments, not articulated or learned",mass_kg=sum(self.m.body_mass[self.bodies]),
            max_grip_force_n=self.max_grip_force,grip_events={g["side"]:g["link"].events for g in self.grips},
            max_segment_deviation_m=self.max_deviation,tracking_work_j=self.tracking_work,grip_pair_work_j=self.grip_work,
            max_actor_contact_force_n=self.max_contact_force,actor_contact_impulse_ns=self.contact_impulse.tolist(),
            summed_contact_time_s=self.contact_time,max_actor_penetration_m=self.max_actor_penetration,
            max_grip_pair_moment_residual_nm=self.max_pair_residual,
            drive_cap_ratios=dict(horizontal=self.max_horizontal_ratio,vertical=self.max_vertical_ratio,torque=self.max_torque_ratio),
            runtime_pose_writes=0,semantic_mapping=ANCHORS,lead_in_s=self.lead)
        (out/"attacker.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        return result
