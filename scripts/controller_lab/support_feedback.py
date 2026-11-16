"""Bounded stance damping through existing leg motors; no root force."""
import numpy as np
import mujoco
from benchmark import inverse_pd_action

class SupportFeedback:
    def __init__(self,env):
        self.env=env;self.anchors=[None,None];self.age=[0.,0.];self.max_force=0.
    def apply(self,raw,tel):
        env=self.env;sim=env.simulator;m,d=tel.m,tel.d
        if not tel.rows:return raw
        rows=tel.rows[-1];bw=tel.mass*9.81
        params=env.config.action_config.params
        common=np.asarray(params["pd_action_offset"])+np.asarray(params["action_scale"])*raw.detach().numpy()[0]
        target_dof=common[sim.data_conversion.dof_convert_to_sim.numpy()]
        targets=target_dof[m.jnt_dofadr[m.actuator_trnid[:,0]]-6].copy()
        torque=np.zeros(m.nv)
        for i,b in enumerate(tel.feet):
            load=rows[7+i]
            if load<.1*bw:self.anchors[i]=None;self.age[i]=0.;continue
            self.age[i]+=env.dt
            if self.anchors[i] is None:self.anchors[i]=d.xpos[b,:2].copy()
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jacBody(m,d,jp,jr,b)
            velocity=jp@d.qvel
            force=-500*(d.xpos[b,:2]-self.anchors[i])-80*velocity[:2]
            force*=min(1.,60/max(np.linalg.norm(force),1e-9))
            force*=min(1.,self.age[i]/.1)
            self.max_force=max(self.max_force,float(np.linalg.norm(force)))
            # Each foot only uses its own leg motors.
            side="Left" if i==0 else "Right"
            cols=[m.jnt_dofadr[j] for j in range(m.njnt) if
                m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE and (m.joint(j).name or "").startswith(side)
                and any(part in m.joint(j).name for part in ("Leg_","Shin_","Foot_","ToeBase_"))]
            torque[cols]+=jp[:2,cols].T@force
        kp=m.actuator_gainprm[:,0];dofs=m.jnt_dofadr[m.actuator_trnid[:,0]]
        # A supplementary target change remains bounded, and native effort caps apply.
        targets+=np.clip(torque[dofs]/np.maximum(kp,1e-6),-.12,.12)
        return inverse_pd_action(env,targets)
