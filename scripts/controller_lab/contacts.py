"""Two dynamic humanoids. Only bounded joint actuators move them after t=0."""
from __future__ import annotations
import copy
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from metrics import Telemetry

def make_pair(base: Path, enabled: bool):
    tree=ET.parse(base); root=tree.getroot(); world=root.find("worldbody")
    actor=world.find("body[@name='Hips']")
    if actor is None: raise ValueError("Missing humanoid")
    for body in list(world.findall("body")): world.remove(body)
    act=root.find("actuator"); original=list(act);act.clear()
    contact=root.find("contact"); original_contacts=list(contact) if contact is not None else []
    if contact is None:contact=ET.SubElement(root,"contact")
    contact.clear()
    for keyframe in root.findall("keyframe"):root.remove(keyframe)
    # All names and references internal to a copied actor get its own namespace.
    names={e.get("name") for e in actor.iter() if e.get("name")}
    names.update(e.get("name") for e in original if e.get("name"))
    for prefix in ("a_","b_"):
        def namespace(e):
            for item in e.iter():
                for k,v in list(item.attrib.items()):
                    if v in names:item.set(k,prefix+v)
            return e
        world.append(namespace(copy.deepcopy(actor)))
        for a in original:act.append(namespace(copy.deepcopy(a)))
        for e in original_contacts:contact.append(namespace(copy.deepcopy(e)))
    # The single inert sensor is sufficient for replay telemetry.
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"))
    for g in range(m.ngeom):
        b=m.geom_bodyid[g]
        if b==0:
            m.geom_contype[g]=1;m.geom_conaffinity[g]=6
        elif m.geom_contype[g] or m.geom_conaffinity[g]:
            is_a=(m.body(b).name or "").startswith("a_")
            bit=2 if is_a else 4
            m.geom_contype[g]=bit
            m.geom_conaffinity[g]=7 if enabled else 1|bit
    return m

def contact_wrenches(m,d):
    force=np.zeros(3);impulse_pairs={}
    for i,c in enumerate(d.contact):
        if c.efc_address<0:continue
        n1=m.body(m.geom_bodyid[c.geom1]).name or ""
        n2=m.body(m.geom_bodyid[c.geom2]).name or ""
        if not ((n1.startswith("a_") and n2.startswith("b_")) or
                (n1.startswith("b_") and n2.startswith("a_"))):continue
        f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
        world=np.asarray(c.frame).reshape(3,3).T@f[:3]
        force+=world if n2.startswith("b_") else -world
        pair=" / ".join(sorted([n1,n2]))
        impulse_pairs[pair]=impulse_pairs.get(pair,0.)+max(0.,float(f[0]))*m.opt.timestep
    return force,impulse_pairs

def planned_controls(m,initial,base_targets,kind):
    scratch=mujoco.MjData(m);scratch.qpos[:]=initial
    mujoco.mj_forward(m,scratch)
    side="a_Right"
    if kind=="arm_push":
        body=m.body(side+"Hand").id
        receiver=m.body("b_Chest").id
        target=scratch.xpos[receiver].copy()+[-.05,0,0]
        include=("a_RightShoulder_","a_RightArm_","a_RightForeArm_","a_RightHand_")
    elif kind=="leg_push":
        body=m.body(side+"Foot").id
        receiver=m.body("b_LeftShin").id
        target=scratch.xpos[receiver].copy()+[-.03,0,-.05]
        include=("a_RightLeg_","a_RightShin_","a_RightFoot_")
    else:
        body=m.body("b_Chest").id
        receiver=m.body("a_Hips").id
        target=scratch.xpos[receiver].copy()+[.02,0,.10]
        include=("b_Spine1_","b_Spine2_","b_Chest_","b_LeftLeg_","b_RightLeg_",
            "b_LeftShin_","b_RightShin_","b_LeftFoot_","b_RightFoot_")
    start=scratch.xpos[body].copy()
    joints=[j for j in range(m.njnt) if (m.joint(j).name or "").startswith(include)]
    cols=np.array([m.jnt_dofadr[j] for j in joints]);qcols=m.jnt_qposadr[joints]
    q=initial.copy();controls=[];max_error=0.
    for t in np.arange(90)/30:
        u=np.clip((t-.35)/1.0,0,1);u=u*u*(3-2*u)
        goal=start+(target-start)*u
        for _ in range(35):
            scratch.qpos[:]=q;mujoco.mj_forward(m,scratch)
            error=goal-scratch.xpos[body]
            if np.linalg.norm(error)<1e-4:break
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jacBody(m,scratch,jp,jr,body)
            J=jp[:,cols]
            delta=J.T@np.linalg.solve(J@J.T+np.eye(3)*1e-4,error)
            q[qcols]=np.clip(q[qcols]+np.clip(delta,-.12,.12),m.jnt_range[joints,0],m.jnt_range[joints,1])
        c=base_targets.copy()
        for a in range(m.nu):
            j=m.actuator_trnid[a,0]
            if j in joints:c[a]=q[m.jnt_qposadr[j]]
        max_error=max(max_error,float(np.linalg.norm(error)))
        controls.append(c)
    return np.array(controls),max_error

def run(root):
    """Learned balance observes each actor through a rigidly transformed mirror.
    Only the shared two-body model integrates physics. The negative control
    replays precisely the same actuator targets with cross-collisions disabled.
    """
    import torch
    from proto_adapter import create_agent,reset_clip,action
    from benchmark import inverse_pd_action
    base=root/"out/controller-comparison/protomotions/stand"
    brains=[create_agent(root),create_agent(root)]
    results=[]
    try:
        for kind,distance in (("arm_push",.55),("leg_push",.55),("body_support",.38)):
            obs=[];observer_origins=[];initial_parts=[];base_targets=[]
            rotations=[Rotation.identity(),Rotation.from_rotvec([0,0,np.pi])]
            for i,brain in enumerate(brains):
                sim=brain.env.simulator
                mujoco.mj_resetData(sim.model,sim.data)
                obs.append(reset_clip(brain,0))
                sim.data.time=0.
                observer_origins.append(sim.data.qpos[:3].copy())
                q=sim.data.qpos[:59].copy()
                q[:2]=[-distance/2 if i==0 else distance/2,0]
                q[3:7]=(rotations[i]*Rotation.from_quat(q[3:7],scalar_first=True)).as_quat(scalar_first=True)
                initial_parts.append(q)
                raw=action(brain,obs[-1],0)
                targets=brain.env.config.action_config.apply(raw)["processed_action"][0].detach().numpy()
                ordered=targets[sim.data_conversion.dof_convert_to_sim.numpy()]
                base_targets.append(ordered)
            initial=np.concatenate(initial_parts)
            planned,error=planned_controls(make_pair(base/"model.xml",True),initial,np.concatenate(base_targets),kind)
            changed=np.any(np.abs(planned-np.concatenate(base_targets)[None,:])>1e-8,axis=0)
            executed=[]
            for enabled in (True,False):
                m=make_pair(base/"model.xml",enabled);d=mujoco.MjData(m)
                d.qpos[:]=initial;mujoco.mj_forward(m,d)
                for g in range(m.ngeom):
                    name=m.body(m.geom_bodyid[g]).name or ""
                    if name.startswith("a_"):m.geom_rgba[g]=[.15,.45,.85,1]
                    elif name.startswith("b_"):m.geom_rgba[g]=[.85,.3,.18,1]
                start_pen=max([max(0.,-float(c.dist)) for c in d.contact if
                    m.body_rootid[m.geom_bodyid[c.geom1]]!=m.body_rootid[m.geom_bodyid[c.geom2]]
                    and m.geom_bodyid[c.geom1]!=0 and m.geom_bodyid[c.geom2]!=0] or [0])
                if start_pen>.001:raise ValueError("Initial inter-humanoid overlap")
                telemetry=Telemetry(m,d,prefix="a_")
                root_b=m.body("b_Hips").id;com0=d.subtree_com[root_b].copy()
                cross=np.zeros(3);pairs={};started=time.perf_counter()
                for step in range(90):
                    if enabled:
                        targets=[]
                        for i,brain in enumerate(brains):
                            raw=action(brain,obs[i],step)
                            target=brain.env.config.action_config.apply(raw)["processed_action"][0].detach().numpy()
                            targets.append(target[brain.env.simulator.data_conversion.dof_convert_to_sim.numpy()])
                        target=np.concatenate(targets)
                        target[changed]=planned[step,changed]
                        executed.append(target.copy())
                    else:target=executed[step]
                    for _ in range(32):
                        d.ctrl[:]=target
                        mujoco.mj_step(m,d)
                        f,p=contact_wrenches(m,d);cross+=f*m.opt.timestep
                        for key,value in p.items():pairs[key]=pairs.get(key,0.)+value
                        telemetry.sample()
                    if enabled:
                        for i,brain in enumerate(brains):
                            sim=brain.env.simulator
                            original=sim._physics_step
                            def observe_only(i=i,sim=sim):
                                # Observation transform only. Never write shared roots.
                                sim._apply_control()
                                q=d.qpos[i*59:(i+1)*59].copy()
                                v=d.qvel[i*58:(i+1)*58].copy()
                                q[:3]=rotations[i].inv().apply(q[:3]-initial_parts[i][:3])+observer_origins[i]
                                q[3:7]=(rotations[i].inv()*Rotation.from_quat(q[3:7],scalar_first=True)).as_quat(scalar_first=True)
                                v[:3]=rotations[i].inv().apply(v[:3])
                                sim.data.qpos[:59]=q;sim.data.qvel[:58]=v;sim.data.time=d.time
                                mujoco.mj_forward(sim.model,sim.data)
                                sim._capture_applied_dof_forces()
                            sim._physics_step=observe_only
                            try:
                                with torch.no_grad():
                                    obs[i],*_=brain.env.step(inverse_pd_action(brain.env,target[i*52:(i+1)*52]))
                            finally:sim._physics_step=original
                out=root/"out/controller-comparison/contacts"/kind/("enabled" if enabled else "disabled")
                summary=telemetry.finish({"id":kind,"kind":"contact"},"contacts",out,time.perf_counter()-started)
                summary.update(contacts_enabled=enabled,cross_impulse_ns=cross.tolist(),
                    contact_pairs_normal_impulse_ns=pairs,receiver_com_delta_m=(d.subtree_com[root_b]-com0).tolist(),
                    initial_cross_penetration_m=start_pen,planned_hand_or_foot_error_m=error,
                    intent_only=True,controller="hybrid_learned_balance_and_limb_IK" if enabled else "identical_target_replay",
                    note="Shared dynamic bodies; learned balance plus limb goals. Not a validated technique.",
                    geometry_goal_reachable=bool(error<.005))
                (out/"summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
                np.save(out/"command_targets.npy",np.array(executed))
                results.append(summary)
                print(json.dumps({k:summary[k] for k in ("case","contacts_enabled","cross_impulse_ns","receiver_com_delta_m","artifacts")}),flush=True)
    finally:
        for brain in brains:brain.env.simulator.shutdown()
    (root/"out/controller-comparison/contacts.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
    return results

if __name__=="__main__":run(Path(__file__).resolve().parents[2])
