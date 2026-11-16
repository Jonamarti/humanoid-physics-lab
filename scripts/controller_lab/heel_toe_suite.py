"""Explicit opt-in natural-gait experiments; saved flat-foot suite is unchanged."""
from pathlib import Path
import argparse,json
from heel_toe import ReferenceBuilder,save_pack,run,rollover,lean_reference,gaze_reference,RolloverTelemetry,analyse_rollover
from heel_toe_model import HeelToeModel
from landing_feedback import LandingFeedback

def prepare(root,period,steps):
    b=ReferenceBuilder(root)
    flat,meta=b.walking(period,steps,lift=.18,support_center=True,forward_sync=True,settle=5.,dynamic_root=True,natural_swing=True)
    roll,rm=rollover(b,flat,meta,heel_strike=-.35)
    lean,lm=lean_reference(b,roll,rm)
    gaze,gm=gaze_reference(b,lean,lm)
    if max(meta["max_ik_error_m"],rm["max_ik_error_m"],lm["max_ik_error_m"])>.002:
        raise RuntimeError("Unreachable reference")
    return b,flat,meta,gaze,gm

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--variant",choices=("absorbed_gaze","heel_gaze"),default="absorbed_gaze")
    p.add_argument("--period",type=float,default=.5)
    p.add_argument("--steps",type=int,default=6)
    p.add_argument("--prepare-only",action="store_true")
    p.add_argument("--foot-contact-tau",type=float)
    args=p.parse_args();root=Path(__file__).resolve().parents[2];out=args.output.resolve()
    b,flat,meta,gaze,gm=prepare(root,args.period,args.steps)
    save_pack([flat,gaze],out/"references.pt")
    candidate=dict(gm,id=args.variant,clip=1,upper_damping=True,leg_absorption=True,landing_feedback=True)
    if args.variant=="heel_gaze":candidate["landing_mode"]="replace"
    if args.foot_contact_tau is not None:candidate["foot_contact_tau"]=args.foot_contact_tau
    cases=[dict(meta,id="flat",clip=0),candidate]
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    if args.prepare_only:
        print("References only; no physical execution or validation.",flush=True);return
    run(root,out/"references.pt",out,cases,same_initial=True,telemetry_factory=RolloverTelemetry,
        model_configurator=HeelToeModel(),action_filter_factory=LandingFeedback)
    for case in cases:analyse_rollover(out/case["id"])
if __name__=="__main__":main()
