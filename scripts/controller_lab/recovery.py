"""Observed-state recovery goals for a motion tracker, not a push-aware script."""
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from cases import smooth

def capture_error(com,velocity,feet,height):
    omega=np.sqrt(9.81/max(.3,height))
    cp=np.asarray(com[:2])+np.asarray(velocity[:2])/omega
    lo=np.min(feet[:,:2],axis=0)-[.085,.035]
    hi=np.max(feet[:,:2],axis=0)+[.085,.035]
    return cp,cp-np.clip(cp,lo,hi)

class RecoveryPlanner:
    def __init__(self,env,case,builder):
        self.env=env;self.builder=builder;self.enabled=case.get("reactive",False)
        self.events=[];self.triggered=False
    def update(self,tel):
        if not self.enabled or self.triggered or not tel.rows:return False
        m,d=tel.m,tel.d;t=d.time
        # Warm-up ends before any test hit. No hit schedule, force or direction is read.
        if t<.8:return False
        r=np.asarray(tel.rows[-1]);feet=d.xpos[tel.feet].copy()
        cp,error=capture_error(r[1:4],r[4:7],feet,r[3])
        if np.linalg.norm(error)<.035 or np.linalg.norm(r[4:6])<.18:return False
        if r[9]<.75*tel.initial_height or r[10]>np.deg2rad(35):return False
        self.triggered=True
        if abs(error[1])>abs(error[0])*.6:
            foot=0 if error[1]>0 else 1
        else:foot=int(np.argmin(r[7:9]))
        stance=1-foot
        target=cp+r[4:6]*.18
        if abs(error[1])<=abs(error[0])*.6:target[1]=feet[foot,1]
        delta=target-feet[foot,:2]
        norm=np.linalg.norm(delta)
        delta*=min(1.,.42/max(norm,1e-6))
        target=feet[foot,:2]+delta
        # Preserve foot ordering and a minimum 12cm base; do not cross legs.
        target[1]=max(target[1],feet[stance,1]+.12) if foot==0 else min(target[1],feet[stance,1]-.12)
        offset=self.env.reference.spawn_translation(None)[0].numpy()
        q=d.qpos[:self.builder.m.nq].copy();q[:3]-=offset
        feet-=offset
        target-=offset[:2]
        b=self.builder;qs=[];cs=[]
        root0=q[:3].copy();root_rot0=q[3:7].copy();rot0=d.xquat[tel.feet].copy();b.max_error=0.
        nominal_z=b.q0[2]-.04
        # Fast response to observed momentum, with a 0.9s supported swing.
        period=.9
        for tt in np.arange(361)/30:
            u=tt/period
            lift=smooth(u/.2)-smooth((u-.72)/.23)
            travel=smooth((u-.12)/.65)
            goals=feet.copy();goals[foot,:2]+=(target-feet[foot,:2])*travel
            goals[foot,2]+= .14*lift
            # Keep original sole height for grounded feet; no frame lifting trick.
            rots=rot0.copy()
            rots[foot]=(Rotation.from_rotvec([0,-.12*lift,0])*Rotation.from_quat(rot0[foot],scalar_first=True)).as_quat(scalar_first=True)
            root=root0.copy()
            end=(feet[stance,:2]+target)*.5
            root[:2]+=(end-root0[:2])*smooth(np.clip(u,0,1))
            support=feet[stance,1]-root0[1]
            root[1]+=support*.65*(smooth(u/.2)-smooth((u-.75)/.25))
            root[2]+=(nominal_z-root0[2])*smooth(tt/.5)
            quat=root_rot0*(1-smooth(tt/.5))+np.array([1.,0,0,0])*smooth(tt/.5)
            quat/=np.linalg.norm(quat)
            q=b.solve(q,root,goals,rots,orientation=quat)
            qs.append(q);contact=np.zeros(53,dtype=bool)
            for i,side in enumerate(("Left","Right")):
                for part in ("Foot","ToeBase"):
                    contact[list(b.robot.kinematic_info.body_names).index(side+part)]=(i!=foot or not(0<u<.95))
            cs.append(contact)
        arrays=b.arrays(qs,cs);lib=self.env.reference_motion.motion_lib
        for key,value in arrays.items():
            dest=getattr(lib,key)
            src=torch.as_tensor(value,dtype=dest.dtype,device=dest.device)
            if dest.shape!=src.shape:raise RuntimeError("Recovery library shape differs")
            dest.copy_(src)
        self.env.reference.assign(torch.tensor([0]),times=torch.zeros(1))
        self.events.append(dict(trigger_s=float(t),foot=foot,capture_xy=cp.tolist(),error_xy=error.tolist(),
            target_local_xy=target.tolist(),observed_com_velocity=r[4:7].tolist(),
            reference_max_ik_error_m=b.max_error))
        return True
