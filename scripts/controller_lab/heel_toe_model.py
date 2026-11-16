"""Declared per-case model changes; baseline physical parameters are restored."""
import numpy as np
from upper_damping import UpperDamping

class HeelToeModel:
    def __init__(self):
        self.damping=UpperDamping();self.solref=None
    def __call__(self,case,m,d):
        if self.solref is None:self.solref=m.geom_solref.copy()
        m.geom_solref[:]=self.solref
        notes=self.damping(case,m,d)
        if "foot_contact_tau" in case:
            tau=float(case["foot_contact_tau"])
            if tau<2*m.opt.timestep:raise ValueError("Contact time constant below two native timesteps")
            ids=[m.body(side+part).id for side in ("Left","Right") for part in ("Foot","ToeBase")]
            gs=np.flatnonzero(np.isin(m.geom_bodyid,ids)).tolist()+[m.geom("floor").id]
            m.geom_solref[gs,0]=tau
            notes["foot_contact"]=dict(time_constant=tau,geom_ids=gs,friction_unchanged=True)
        return notes
