"""Existing chest pulses and public takedown tracks with frozen ProtoMotions."""
from pathlib import Path
import argparse,hashlib,json,time,os
import numpy as np
import mujoco
import torch
from gait import ReferenceBuilder,save_pack,analyse,pulse_force
from gait_suite import standing,impact_cases
from proto_adapter import create_agent,reset_clip,action
from recovery import RecoveryPlanner
from heel_toe import RolloverTelemetry
from heel_toe_model import HeelToeModel
from metrics import physical_fingerprint
from load_response import experiment_output
from scene_forces import ToriScene

def run_group(root,pack,out,name,scene_file=None,pulse=None,smoke=False,warmup=3.):
    builder=ReferenceBuilder(root)
    agent=create_agent(root,pack,scene_file=scene_file);env=agent.env;sim=env.simulator;m,d=sim.model,sim.data
    original=mujoco.mj_step
    lib=env.reference_motion.motion_lib
    frozen={k:getattr(lib,k).clone() for k in ("gts","grs","gvs","gavs","dps","dvs","contacts")}
    template=json.loads(scene_file.read_text(encoding="utf-8")) if scene_file else None
    initial=None;snapshot=None;spawn=None
    configure=HeelToeModel();results=[]
    try:
        for reactive in ([False] if smoke else [False,True]):
            for key,value in frozen.items():getattr(lib,key).copy_(value)
            mujoco.mj_resetData(m,d);obs=reset_clip(agent,0);d.time=0.
            c=dict(id=name+("_reactive" if reactive else "_tracker"),clip=0,kind="recovery",duration_s=2. if smoke else 8.,
                reactive=reactive,steps_requested=0 if pulse is None and template is None else 1,
                upper_damping=True,leg_absorption=True,foot_contact_tau=.01,period_s=.5)
            if pulse:c.update(pulse,id=c["id"],duration_s=c["duration_s"],reactive=reactive)
            config=configure(c,m,d) if initial is None else config
            scene=ToriScene(m,d,template,env._spawn.respawn_root_offset[0].numpy()) if template else None
            if scene:scene.initialize()
            if initial is None:
                mujoco.mj_forward(m,d);env.observe(torch.tensor([0]));obs=env.get_obs()
                if warmup and not smoke:
                    prep=RolloverTelemetry(m,d)
                    def prepare_step(model,data,*args,**kwargs):
                        if model is m:
                            data.xfrc_applied[:]=0;data.qfrc_applied[:]=0
                            mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);mujoco.mj_comVel(m,d)
                            if scene:scene.apply(0.)
                        original(model,data,*args,**kwargs)
                        if model is m:prep.sample()
                    mujoco.mj_step=prepare_step
                    try:
                        for k in range(round(warmup/env.dt)):
                            with torch.no_grad():obs,*_=env.step(action(agent,obs,k))
                    finally:mujoco.mj_step=original
                    prep.finish(dict(id=name+"_preparation",kind="posture"),"protomotions",out/(name+"_preparation"),warmup)
                    d.time=0.
                    env.reference.assign(torch.tensor([0]),ids=torch.tensor([0]),times=torch.zeros(1))
                    env.observe(torch.tensor([0]));obs=env.get_obs()
                    # Reset measurement-only scene state, preserve physically settled poses.
                    if scene:scene=ToriScene(m,d,template,env._spawn.respawn_root_offset[0].numpy())
                initial=np.zeros(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_INTEGRATION))
                mujoco.mj_getState(m,d,initial,mujoco.mjtState.mjSTATE_INTEGRATION)
                snapshot=env.save_state();spawn=env._spawn.respawn_root_offset.clone()
            else:
                env.restore_state(snapshot);env._spawn.respawn_root_offset.copy_(spawn)
                mujoco.mj_setState(m,d,initial,mujoco.mjtState.mjSTATE_INTEGRATION);mujoco.mj_forward(m,d)
                mujoco.mj_setState(m,d,initial,mujoco.mjtState.mjSTATE_INTEGRATION)
                obs=env.get_obs()
            tel=RolloverTelemetry(m,d);planner=RecoveryPlanner(env,c,builder)
            velocities=[];forces=[];torques=[];robot_wrenches=[];started=time.perf_counter()
            observation0={k:v.detach().numpy().copy() for k,v in obs.items()}
            robot_ids=np.flatnonzero(m.body_rootid==m.body("Hips").id)
            max_direct_root_wrench=0.
            def measured(model,data,*args,**kwargs):
                nonlocal max_direct_root_wrench
                if model is not m:return original(model,data,*args,**kwargs)
                data.xfrc_applied[:]=0;data.qfrc_applied[:]=0
                mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);mujoco.mj_comVel(m,d)
                if scene:scene.apply(data.time)
                if pulse:
                    data.xfrc_applied[m.body("Chest").id,:3]=pulse_force(data.time,m.opt.timestep,
                        c["hit_time"],c["hit_duration"],c["impulse_ns"],c["direction"])
                max_direct_root_wrench=max(max_direct_root_wrench,float(np.abs(data.xfrc_applied[m.body("Hips").id]).max()))
                original(model,data,*args,**kwargs);tel.sample()
                if scene:scene.solved()
                if (tel.idx-1)%8==0:
                    velocities.append(data.qvel.copy());forces.append(data.xfrc_applied[m.body("Chest").id].copy())
                    torques.append(data.actuator_force.copy());robot_wrenches.append(data.xfrc_applied[robot_ids].copy())
            mujoco.mj_step=measured
            for k in range(round(c["duration_s"]/env.dt)):
                tick=time.perf_counter()
                if planner.update(tel):env.observe(torch.tensor([0]));obs=env.get_obs()
                a=action(agent,obs,k);tel.control_latencies.append(time.perf_counter()-tick)
                with torch.no_grad():obs,*_=env.step(a)
                if not np.isfinite(d.qpos).all():raise RuntimeError("Nonfinite physical state")
            dest=out/c["id"]
            result=tel.finish(c,"protomotions",dest,time.perf_counter()-started)
            np.savez_compressed(dest/"initial-observations.npz",**observation0)
            np.savez_compressed(dest/"dynamics.npz",qvel=np.array(velocities),chest_force=np.array(forces),
                joint_torque=np.array(torques),robot_wrenches=np.array(robot_wrenches),robot_body_ids=robot_ids)
            result.update(scenario=c,model_intervention=config,planner_events=planner.events,
                physical_model_sha256=physical_fingerprint(m),initial_state_sha256=hashlib.sha256(initial.tobytes()).hexdigest(),
                reference_sha256=hashlib.sha256(pack.read_bytes()).hexdigest(),max_direct_root_wrench=max_direct_root_wrench,
                auxiliary_bodies_excluded_from_policy=sum(m.body(i).name.startswith("projectile_scene_") for i in range(m.nbody)),policy_actuators=m.nu,physical_preparation_s=0. if smoke else warmup,
                source="existing gait_suite.impact_cases" if pulse else ("scenarios/physical_refs/"+name+".json" if scene else "no attack control"))
            if scene:result["attacker"]=scene.save(dest)
            (dest/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
            review=analyse(dest);results.append(dict(summary=result,review=review))
            print("SCENE_RESULT",c["id"],"fell",review["fell"],"stable",review["stable_final_two_seconds"],
                "steps",review["steps"],"artifacts",result["artifacts"],
                "Tori_contact",scene.max_contact_force if scene else None,flush=True)
            mujoco.mj_step=original
    finally:
        mujoco.mj_step=original;sim.shutdown();os.chdir(root)
    return results

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--output",default="out/existing-scenarios/trials-v1")
    parser.add_argument("--exports",default="out/existing-scenarios/exports-native");parser.add_argument("--warmup",type=float,default=3.);parser.add_argument("--smoke",action="store_true");parser.add_argument("--groups",nargs="*")
    args=parser.parse_args();root=Path(__file__).resolve().parents[2]
    out=experiment_output(root,args.output);out.mkdir(parents=True)
    b=ReferenceBuilder(root);save_pack([standing(b)],out/"references.pt")
    hypothesis="Transfer existing chest-pulse bank and two unchanged public Tori tracks to the same published SOMA42.47kg tracker. Do not claim equivalence to old Uke75kg. Compare frozen tracker and observed-state recovery planner on identical full initial state, model, reference, forces and attack tracks. Physical Tori segments retain75kg and finite drive caps; all grips require physical proximity and equal/opposite forces. No runtime actual pose writes or robot root force. Report misses, falls, cap saturation, penetration and failure to recover; no retuning or relaxed gates."
    (out/"hypothesis.txt").write_text(hypothesis,encoding="utf-8")
    pulses={c["id"]:c for c in impact_cases() if not c["reactive"] and c.get("impulse_ns")}
    groups=[("no_attack",None,None)]
    for group,source in [("punch_forward20","punch_forward_20_baseline"),("punch_backward20","punch_backward_20_baseline"),
            ("kick_left36","kick_left_36_baseline"),("kick_backward36","kick_backward_36_baseline")]:
        groups.append((group,None,pulses[source]))
    for group in ["outer_reap","leg_drive"]:groups.append((group,root/args.exports/(group+".json"),None))
    if args.groups:groups=[g for g in groups if g[0] in args.groups]
    if args.smoke:groups=[g for g in groups if g[0]=="outer_reap"]
    results=[]
    for name,scene,pulse in groups:
        results.extend(run_group(root,out/"references.pt",out,name,scene,pulse,args.smoke,args.warmup))
        (out/"results.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
if __name__=="__main__":main()
