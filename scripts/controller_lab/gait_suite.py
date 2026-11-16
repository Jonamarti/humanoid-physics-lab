"""Reproducible gait and chest-pulse suites, using only references authored here."""
import argparse,json
from pathlib import Path
import numpy as np
from gait import ReferenceBuilder,save_pack,run
from recovery import RecoveryPlanner

def standing(builder,duration=12.):
    q=np.repeat(builder.q0[None,:],round(duration*30)+1,axis=0)
    contacts=np.zeros((len(q),53),dtype=bool)
    for side in ("Left","Right"):
        for part in ("Foot","ToeBase"):
            contacts[:,list(builder.robot.kinematic_info.body_names).index(side+part)]=True
    a=builder.arrays(q,contacts)
    for k in ("gvs","gavs","dvs"):a[k][:]=0.
    return a

def impact_cases():
    cases=[dict(id="no_hit_"+str(on),clip=0,duration_s=8.,kind="posture",reactive=on,steps_requested=0) for on in (False,True)]
    for profile,duration in (("punch",.08),("kick",.18)):
        for direction,label in (([1,0,0],"forward"),([-1,0,0],"backward"),([0,1,0],"left"),([0,-1,0],"right")):
            for impulse in (8.,20.,36.):
                for reactive in (False,True):
                    cases.append(dict(id=f"{profile}_{label}_{int(impulse)}_"+("reactive" if reactive else "baseline"),
                        clip=0,duration_s=8.,kind="recovery",impulse_ns=impulse,direction=direction,
                        hit_time=2.3,hit_duration=duration,reactive=reactive,steps_requested=1))
    return cases

def main():
    p=argparse.ArgumentParser();p.add_argument("suite",choices=("walk","impacts"))
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--noslip",type=int,default=0)
    p.add_argument("--cases",nargs="*")
    args=p.parse_args();root=Path(__file__).resolve().parents[2];out=args.output.resolve()
    b=ReferenceBuilder(root);chunks=[];cases=[]
    if args.suite=="walk":
        for period,label in ((2.4,"slow"),(1.2,"medium"),(.8,"fast"),(.5,"higher")):
            b.max_error=0.
            a,meta=b.walking(period,6,support_center=True,forward_sync=True,settle=5.,dynamic_root=True,natural_swing=True,lift=.18)
            if meta["max_ik_error_m"]>.002:raise RuntimeError("Unreachable reference: "+str(meta))
            chunks.append(a);cases.append(dict(meta,id=label,clip=len(chunks)-1,noslip_iterations=args.noslip))
    else:
        chunks=[standing(b)];cases=impact_cases()
        for c in cases:c["noslip_iterations"]=args.noslip
    save_pack(chunks,out/"references.pt")
    (out/"scenarios.json").write_text(json.dumps(cases,indent=2),encoding="utf-8")
    if args.cases:cases=[c for c in cases if c["id"] in args.cases]
    factory=(lambda env,c:RecoveryPlanner(env,c,b)) if args.suite=="impacts" else None
    run(root,out/"references.pt",out,cases,factory,same_initial=True,warmup=3. if factory else 0.)
if __name__=="__main__":main()
