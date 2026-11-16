"""Prepare load acceptance before heel strike; preserve world foot targets."""
from pathlib import Path
import json
import numpy as np
from cases import smooth
from load_response import lower_reference,energy_by_joint,experiment_output
from heel_toe_suite import prepare
from heel_toe import save_pack,run,analyse_rollover,RolloverTelemetry
from heel_toe_model import HeelToeModel
from landing_feedback import LandingFeedback

def prepared_depth(times,period,steps,depth=.03,lead=0.):
    value=np.zeros_like(times,dtype=float)
    for k in range(steps):
        u=(times-1-k*period)/period
        value+=depth*(smooth((u-(.72-lead))/.38)-smooth((u-1.14)/.66))
    return np.minimum(value,depth)

def main(lead=0.,output=None):
    root=Path(__file__).resolve().parents[2];out=experiment_output(root,"out/load-response/prepared-early-v4" if lead else "out/load-response/prepared-v3",output)
    b,_,_,gaze,meta=prepare(root,.5,6);times=np.arange(len(gaze["gts"]))/30
    depth=prepared_depth(times,.5,6,lead=lead)
    yielded,_,_,error=lower_reference(b,gaze,depth)
    if error>.002:raise RuntimeError("Prepared acceptance is unreachable")
    save_pack([gaze,yielded],out/"references.pt")
    common=dict(meta,upper_damping=True,leg_absorption=True,landing_feedback=True,
        landing_mode="replace",foot_contact_tau=.01)
    cases=[dict(common,id="before",clip=0),dict(common,id="prepared_early" if lead else "prepared",clip=1,preparation_lead_phase=lead)]
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    (out/"hypothesis.txt").write_text(
        ("Earlier prepared3cm reduced head HF13% and chest6% but increased worst chest/head peak. Hypothesis: continuing reference descent through heel contact increases landing velocity. Advance only the rising phase by.18 cycle(90ms), preserving depth, rise duration and recovery; arrive flexed before impact. Reject increased worst impact, lost heel roll or stability. " if lead else "")+"Chest/head peaks occur8ms after heel contact; reactive30Hz reference yield cannot change the immediate collision. Knees keep extending at reception. Phase damping increased absorbed work but not acceleration attenuation. Prepare coordinated root/leg IK flexion in late swing: max3cm reference descent, start phase.72, peak1.10, recover1.14-1.80. Preserve both world foot goals, heel/toe rotations, gains, contact, friction, cadence, horizontal path and full initial state. No actual-body pose assignments. Reject lost six alternating heel-first steps/rollovers, increased chest/head vibration/peak, work or artifacts. Planned load acceptance, not autonomous footstep selection.",encoding="utf-8")
    np.savez_compressed(out/"prepared-reference.npz",times=times,depth_m=depth)
    run(root,out/"references.pt",out,cases,same_initial=True,telemetry_factory=RolloverTelemetry,
        model_configurator=HeelToeModel(),action_filter_factory=LandingFeedback)
    for c in cases:
        p=out/c["id"];r=analyse_rollover(p);energy_by_joint(p)
        print("PREPARED",c["id"],r["heel_first_strikes"],r["stable"],
            {k:r["segments"][k] for k in ["chest","head"]},flush=True)
if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument("--output");parser.add_argument("--early",action="store_true")
    args=parser.parse_args();main(.18 if args.early else 0.,args.output)
