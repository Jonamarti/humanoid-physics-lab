"""Load-triggered hip/knee impedance, kept inside native motor effort caps."""
import json
from pathlib import Path
import numpy as np
from cases import smooth
from heel_toe import RolloverTelemetry
from load_response import experiment_output

class SupportedHeelDetector:
    """Require a supported, genuinely airborne swing before a heel reception."""
    def __init__(self):
        self.dwell=np.zeros(2);self.armed=[False,False]
        self.last=np.full(2,-10.);self.events=[]
    def update(self,t,loads,heels,clearance,bw,dt,healthy=True):
        for i in range(2):
            good=healthy and clearance[i]>=.03 and loads[i]<.05*bw and loads[1-i]>.6*bw
            self.dwell[i]=self.dwell[i]+dt if good else 0.
            if self.dwell[i]>=.03:self.armed[i]=True
            if healthy and self.armed[i] and loads[i]>.15*bw and heels[i]>.08*bw and t-self.last[i]>.25:
                self.events.append(dict(foot=i,contact_s=float(t),load_n=float(loads[i]),heel_n=float(heels[i])))
                self.last[i]=t;self.armed[i]=False
        return self.events

def impedance_envelope(t,start):
    return float(smooth((t-start)/.025)-smooth((t-start-.14)/.12))

def set_impedance(m,ids,base_gain,base_bias,weight,reduction=0.,damping_fraction=0.):
    if not 0<=reduction<1 or not 0<=damping_fraction<=1:
        raise ValueError("Invalid stiffness/damping fractions")
    for i,actuators in enumerate(ids):
        alpha=float(np.clip(weight[i],0,1))
        for actuator in actuators:
            kp=base_gain[actuator,0]*(1-reduction*alpha)
            m.actuator_gainprm[actuator,0]=kp
            m.actuator_biasprm[actuator,1]=-kp
            m.actuator_biasprm[actuator,2]=base_bias[actuator,2]*(1+damping_fraction*alpha)

class WeightAcceptanceTelemetry(RolloverTelemetry):
    def __init__(self,m,d,case):
        super().__init__(m,d)
        self.enabled=case.get("weight_impedance",False) or bool(case.get("loading_damping_fraction",0))
        self.case=case;self.schedule=[];self.impedance_events=[]
        self.detector=SupportedHeelDetector();self.heel_events=self.detector.events
        self.floor=m.geom("floor").id
        self.sole_geoms=[np.flatnonzero(np.isin(m.geom_bodyid,list(g)) & ((m.geom_contype|m.geom_conaffinity)!=0)) for g in self.foot_groups]
        self.base_gain=m.actuator_gainprm.copy();self.base_bias=m.actuator_biasprm.copy()
        self.ids=[]
        for side in ("Left","Right"):
            self.ids.append([int(np.flatnonzero(m.actuator_trnid[:,0]==m.joint(side+p).id)[0]) for p in ("Leg_x","Shin_x")])
        self.armed=[False,False];self.unloaded=np.zeros(2);self.last=np.full(2,-10.)
    def sample(self):
        super().sample()
        m,d=self.m,self.d;t=d.time-self.start
        if self.case.get("qualified_lifts"):
            from analyse_natural import minimum_sole_clearance
            row=self.contact_rows[-1]
            clear=[minimum_sole_clearance(m,d,self.floor,g) for g in self.sole_geoms]
            healthy=self.rows[-1][9]>.7*self.initial_height and self.rows[-1][10]<np.deg2rad(25) and not self.rows[-1][11]
            if t>.8:self.detector.update(t,row[1:3],row[3:5],clear,self.mass*9.81,m.opt.timestep,healthy)
        if not self.enabled:return
        ids=np.array(self.ids).ravel()
        applied=m.actuator_gainprm[ids,0].copy();applied_kd=-m.actuator_biasprm[ids,2].copy()
        row=self.contact_rows[-1];bw=self.mass*9.81
        healthy=self.rows[-1][9]>.7*self.initial_height and self.rows[-1][10]<np.deg2rad(25) and not self.rows[-1][11]
        if t>.8 and not self.case.get("qualified_lifts"):
            for i in range(2):
                self.unloaded[i]=self.unloaded[i]+m.opt.timestep if row[1+i]<.05*bw else 0.
                if self.unloaded[i]>=.04:self.armed[i]=True
                if healthy and self.armed[i] and row[1+i]>.15*bw and row[3+i]>.08*bw and t-self.last[i]>.25:
                    self.last[i]=t;self.armed[i]=False
                    self.impedance_events.append(dict(foot=i,contact_s=float(t),load_n=float(row[1+i])))
        if self.case.get("qualified_lifts"):self.impedance_events=self.heel_events
        weight=np.zeros(2)
        for event in self.impedance_events:
            weight[event["foot"]]+=impedance_envelope(t,event["contact_s"])
        weight=np.clip(weight,0,1)
        set_impedance(m,self.ids,self.base_gain,self.base_bias,weight,
            reduction=.20 if self.case.get("weight_impedance") else 0.,
            damping_fraction=self.case.get("loading_damping_fraction",0))
        self.schedule.append([t,*applied,*applied_kd,*weight])
    def finish(self,case,controller,out,wall_s):
        # The final model snapshot describes the resting plant, while the
        # complete actually-applied stiffness schedule is preserved separately.
        self.m.actuator_gainprm[:]=self.base_gain;self.m.actuator_biasprm[:]=self.base_bias
        result=super().finish(case,controller,out,wall_s)
        if self.enabled:
            np.savez_compressed(out/"impedance-schedule.npz",rows=np.array(self.schedule))
            meta=dict(enabled=True,actuator_ids=np.array(self.ids).ravel().tolist(),
                order=["LeftLeg_x","LeftShin_x","RightLeg_x","RightShin_x"],
                sample_hz=960,stiffness_reduction_fraction=.20 if self.case.get("weight_impedance") else 0.,
                damping_increase_fraction=self.case.get("loading_damping_fraction",0),
                damping_unchanged=not bool(self.case.get("loading_damping_fraction",0)),
                force_caps_unchanged=True,events=self.impedance_events,
                columns=["time","kp_left_hip","kp_left_knee","kp_right_hip","kp_right_knee","kd_left_hip","kd_left_knee","kd_right_hip","kd_right_knee","next_left_weight","next_right_weight"])
            (out/"impedance-plan.json").write_text(json.dumps(meta,indent=2),encoding="utf-8")
            result["runtime_impedance"]=meta
        return result

def main(output=None):
    from load_response import LoadAcceptance,energy_by_joint
    from heel_toe_suite import prepare
    from heel_toe import save_pack,run,analyse_rollover
    from heel_toe_model import HeelToeModel
    from landing_feedback import LandingFeedback
    root=Path(__file__).resolve().parents[2];out=experiment_output(root,"out/load-response/impedance-v1",output)
    b,_,_,gaze,meta=prepare(root,.5,6)
    save_pack([gaze],out/"references.pt")
    common=dict(meta,clip=0,upper_damping=True,leg_absorption=True,landing_feedback=True,
        landing_mode="replace",foot_contact_tau=.01,load_acceptance=True,yield_depth=.008)
    cases=[dict(common,id="yield_reference"),dict(common,id="yield_impedance",weight_impedance=True)]
    context={};model=HeelToeModel()
    def configure(case,m,d):
        context["case"]=case
        return model(case,m,d)
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    (out/"hypothesis.txt").write_text(
        "Reference-only8mm yield gave little reduction: knee/hip position tracking remains rigid. Compare a single20% position-stiffness reduction on receiving sagittal hip/knee, triggered by actual heel load after a real unload; ramp25ms, recover140-260ms. Damping, targets, references, effort/joint limits, feet, friction, contact and initial state remain unchanged. Save actual native stiffness schedule960Hz; no root-force or actual pose writes. Mechanical work is measured; this is a muscle-function analogue, not physiological muscle activation.",encoding="utf-8")
    run(root,out/"references.pt",out,cases,same_initial=True,
        planner_factory=lambda env,c:LoadAcceptance(env,c,b,[gaze],c["yield_depth"]),
        telemetry_factory=lambda m,d:WeightAcceptanceTelemetry(m,d,context["case"]),
        model_configurator=configure,action_filter_factory=LandingFeedback)
    for case in cases:
        p=out/case["id"];analyse_rollover(p)
        print("LOADING_ENERGY",case["id"],json.dumps(energy_by_joint(p)),flush=True)
def main_damping(output=None):
    from load_response import LoadAcceptance,energy_by_joint
    from heel_toe_suite import prepare
    from heel_toe import save_pack,run,analyse_rollover
    from heel_toe_model import HeelToeModel
    from landing_feedback import LandingFeedback
    root=Path(__file__).resolve().parents[2];out=experiment_output(root,"out/load-response/damping-v2",output)
    b,_,_,gaze,meta=prepare(root,.5,6);save_pack([gaze],out/"references.pt")
    common=dict(meta,clip=0,upper_damping=True,leg_absorption=True,landing_feedback=True,
        landing_mode="replace",foot_contact_tau=.01,qualified_lifts=True)
    cases=[dict(common,id="before"),
        dict(common,id="yield_reference",load_acceptance=True,yield_depth=.008),
        dict(common,id="yield_damped",load_acceptance=True,yield_depth=.008,loading_damping_fraction=.5)]
    context={};model=HeelToeModel()
    def configure(case,m,d):
        context["case"]=case
        return model(case,m,d)
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    (out/"hypothesis.txt").write_text(
        "20% stiffness release increased knee negative work but did not reduce chest vibration and increased total positive work. Keep support stiffness; test 50% extra implicit viscous damping only in sagittal receiving hip/knee, ramp25ms, hold until140ms, recover by260ms after a qualified supported full-sole swing >=3cm for30ms and actual heel load. Compare reference8mm yield alone against identical yield plus phase damping; nominal before is retained. Extra velocity torque is dissipative before native force saturation; original effort and joint limits remain. Reject lost heel rollovers, instability, worse vibration or work. Same initial state/contact/friction/geometry/cadence; actual body poses are never assigned during the run.",encoding="utf-8")
    run(root,out/"references.pt",out,cases,same_initial=True,
        planner_factory=lambda env,c:LoadAcceptance(env,c,b,[gaze],c.get("yield_depth",0.)),
        telemetry_factory=lambda m,d:WeightAcceptanceTelemetry(m,d,context["case"]),
        model_configurator=configure,action_filter_factory=LandingFeedback)
    for c in cases:
        p=out/c["id"];r=analyse_rollover(p);energy_by_joint(p)
        print("COMPACT",c["id"],r["heel_first_strikes"],r["segments"]["chest"],r["segments"]["head"],flush=True)

if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument("--output");parser.add_argument("--damping",action="store_true")
    args=parser.parse_args()
    main_damping(args.output) if args.damping else main(args.output)

