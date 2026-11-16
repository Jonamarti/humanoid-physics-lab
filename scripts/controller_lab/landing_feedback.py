"""Terminal-swing ankle/MTP goals on top of learned whole-body balance."""
import numpy as np
from scipy.spatial.transform import Rotation
from benchmark import inverse_pd_action
from cases import smooth

def landing_target(raw,current,error,weight,last,dt,bounds,limit,mode):
    if mode=="replace":
        goal=np.clip(current+error,*bounds)
        proposed=raw+weight*(goal-raw)
        return float(np.clip(proposed,last-4*dt,last+4*dt))
    return float(raw+weight*np.clip(.5*error,-limit,limit))

class LandingFeedback:
    def __init__(self,env,case):
        self.env=env;self.case=case;self.maximum_requested_torque=np.zeros(2)
        self.enabled=case.get("landing_feedback",False)
        self.last_targets=None
    def apply(self,raw,tel):
        if not self.enabled or not tel.rows:return raw
        env=self.env;sim=env.simulator;m,d=tel.m,tel.d
        time=float(env.reference.times[0]);period=self.case["period_s"];steps=self.case["steps_requested"]
        params=env.config.action_config.params
        common=np.asarray(params["pd_action_offset"])+np.asarray(params["action_scale"])*raw.detach().numpy()[0]
        targets=common[sim.data_conversion.dof_convert_to_sim.numpy()][m.jnt_dofadr[m.actuator_trnid[:,0]]-6].copy()
        if self.last_targets is None:self.last_targets=targets.copy()
        state=env.reference.poses_at(None,env.reference.times)
        # Published common-body ordering is provided by the simulator conversion.
        # Convert actual native semantic body index to reference common index.
        to_common=sim.data_conversion.body_convert_to_common.numpy()
        bw=tel.mass*9.81
        for i,side in enumerate(("Left","Right")):
            swings=list(range(i,steps,2))
            phases=[(time-1-k*period)/period for k in swings]
            candidates=[u for u in phases if .55<=u<=1.16]
            if not candidates:continue
            u=candidates[-1]
            phase=float(smooth((u-.55)/.12)*(1-smooth((u-.98)/.18)))
            load=tel.rows[-1][7+i]
            weight=phase*float(np.clip((.8*bw-load)/(.5*bw),0,1))
            if weight<=0:continue
            body=m.body(side+"Foot").id
            index=int(to_common[body-1])
            desired=Rotation.from_quat(state.rigid_body_rot[0,index].numpy())
            actual=Rotation.from_quat(d.xquat[body],scalar_first=True)
            error=(desired*actual.inv()).as_rotvec()
            for name in (side+"Foot_x",side+"ToeBase_x"):
                j=m.joint(name).id
                a=int(np.flatnonzero(m.actuator_trnid[:,0]==j)[0])
                kp=m.actuator_gainprm[a,0]
                limit=.1*max(abs(m.actuator_forcerange[a]))/kp
                if name.endswith("ToeBase_x"):
                    common_dof=int(sim.data_conversion.dof_convert_to_common[m.jnt_dofadr[j]-6])
                    e=float(state.dof_pos[0,common_dof])-d.qpos[m.jnt_qposadr[j]]
                else:e=float(np.dot(error,d.xaxis[j]))
                proposed=landing_target(targets[a],d.qpos[m.jnt_qposadr[j]],e,weight,
                    self.last_targets[a],env.dt,m.jnt_range[j],limit,self.case.get("landing_mode"))
                delta=proposed-targets[a];targets[a]=proposed
                self.maximum_requested_torque[i]=max(self.maximum_requested_torque[i],abs(float(delta*kp)))
        self.last_targets=targets.copy()
        return inverse_pd_action(env,targets)
