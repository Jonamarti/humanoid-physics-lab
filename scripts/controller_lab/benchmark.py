"""CPU comparison on identical goals, initial states, contacts and perturbations."""
from __future__ import annotations
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
import mujoco
from cases import CASES, DURATION, PULSE_START, PULSE_DURATION, prepare_pack
from metrics import Telemetry, model_fingerprint
from proto_adapter import create_agent, reset_clip, action

class Mpc:
    def __init__(self, root, model, samples=32, horizon=48):
        dll=root/"out/native/mjpc-build/Release/hpl_mjpc.dll"
        self.dll=ctypes.CDLL(str(dll))
        self.dll.lab_create.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int]
        self.dll.lab_create.restype=ctypes.c_void_p
        self.dll.lab_plan.argtypes=[ctypes.c_void_p,ctypes.c_void_p,
            np.ctypeslib.ndpointer(dtype=np.float64,flags="C_CONTIGUOUS"),ctypes.c_int,
            ctypes.c_double,np.ctypeslib.ndpointer(dtype=np.float64,flags="C_CONTIGUOUS")]
        self.dll.lab_destroy.argtypes=[ctypes.c_void_p]
        self.dll.lab_error.restype=ctypes.c_char_p
        if self.dll.lab_version()!=mujoco.mj_version(): raise RuntimeError("MuJoCo ABI mismatch")
        self.handle=self.dll.lab_create(model._address,samples,horizon)
        if not self.handle: raise RuntimeError(self.dll.lab_error().decode())
        self.nu=model.nu
    def plan(self,data,refs):
        output=np.zeros(self.nu)
        if not self.dll.lab_plan(self.handle,data._address,np.ascontiguousarray(refs,dtype=np.float64),
            len(refs),1/30,output): raise RuntimeError(self.dll.lab_error().decode())
        if not np.isfinite(output).all(): raise RuntimeError("Nonfinite MPC control")
        return output
    def close(self):
        if self.handle:
            self.dll.lab_destroy(self.handle)
            self.handle=None

def references(agent):
    import torch
    from scipy.spatial.transform import Rotation
    env=agent.env; sim=env.simulator; m=sim.model; d=sim.data
    _,w=env.reference.current_and_window(None,torch.arange(16,dtype=torch.float32)/30)
    q=np.repeat(d.qpos[None,:],16,axis=0); v=np.zeros((16,m.nv))
    q[:,:3]=w.pos[0,:,0].numpy()
    quat=np.roll(w.rot[0,:,0].numpy(),1,axis=-1)
    q[:,3:7]=quat
    ds=sim.data_conversion.dof_convert_to_sim.numpy()
    q[:,7:59]=w.dof_pos[0].numpy()[:,ds]
    v[:,:3]=w.vel[0,:,0].numpy()
    v[:,3:6]=Rotation.from_quat(quat,scalar_first=True).inv().apply(w.ang_vel[0,:,0].numpy())
    v[:,6:58]=w.dof_vel[0].numpy()[:,ds]
    p=np.repeat(d.xpos[None,:,:],16,axis=0)
    p[:,1:54]=w.pos[0].numpy()[:,sim.data_conversion.body_convert_to_sim.numpy()]
    return np.concatenate([q,v,p.reshape(16,-1)],axis=1)

def inverse_pd_action(env,targets):
    import torch
    sim=env.simulator
    # Actuator order is checked against joint order explicitly.
    target_by_dof=np.empty(52)
    for a in range(sim.model.nu):
        j=sim.model.actuator_trnid[a,0]
        target_by_dof[sim.model.jnt_dofadr[j]-6]=targets[a]
    common=target_by_dof[sim.data_conversion.dof_convert_to_common.numpy()]
    params=env.config.action_config.params
    if params.get("action_transform") is not None:
        raise RuntimeError("Expected the published affine, unclipped action contract")
    offset=np.asarray(params["pd_action_offset"])
    scale=np.asarray(params["action_scale"])
    return torch.from_numpy(((common-offset)/scale).astype(np.float32))[None,:]

def run(root,controller,selected,samples=32,destination=None,reference_file=None):
    import torch
    root=root.resolve(); out=(destination or root/"out/controller-comparison").resolve()
    pack=reference_file or root/"out/controller-comparison/references.pt"
    if not pack.exists(): prepare_pack(root,pack)
    agent=create_agent(root,pack)
    env=agent.env; sim=env.simulator; m=sim.model; d=sim.data
    results=[]
    # Save actual compiled configuration, before modifying model data per case.
    expected_model=model_fingerprint(m)
    original_step=mujoco.mj_step
    try:
        for case in CASES:

            torch.manual_seed(20261116); np.random.seed(20261116)
            # Also reseed the env-owned generator; identical spawn and reset.
            generator=getattr(env,"_generator",None)
            if generator is not None: generator.manual_seed(20261116)
            mujoco.mj_resetData(m,d)
            obs=reset_clip(agent,case["clip"])
            # Physics clock is trial-local. This does not alter positions/velocities.
            d.time=0.
            if selected and case["id"] not in selected: continue
            telemetry=Telemetry(m,d)
            pulse_body=m.body("Chest").id
            mpc=Mpc(root,m,samples) if controller=="mjpc" else None
            def measured_step(model,data,*args,**kwargs):
                if model is not m: return original_step(model,data,*args,**kwargs)
                data.xfrc_applied[:]=0
                t=data.time-telemetry.start
                overlap=max(0.,min(t+m.opt.timestep,PULSE_START+PULSE_DURATION)-max(t,PULSE_START))
                if "impulse_ns" in case and overlap>0:
                    data.xfrc_applied[pulse_body,:3]=np.asarray(case["direction"])*case["impulse_ns"]/PULSE_DURATION*(overlap/m.opt.timestep)
                original_step(model,data,*args,**kwargs)
                telemetry.sample()
            mujoco.mj_step=measured_step
            started=time.perf_counter()
            try:
                for step in range(round(DURATION/env.dt)):
                    tick=time.perf_counter()
                    if controller=="protomotions":
                        a=action(agent,obs,step)
                    else:
                        refs=references(agent)
                        if controller=="mjpc": targets=mpc.plan(d,refs)
                        else:
                            q=refs[0,:m.nq]
                            targets=q[m.jnt_qposadr[m.actuator_trnid[:,0]]]
                        a=inverse_pd_action(env,targets)
                    telemetry.control_latencies.append(time.perf_counter()-tick)
                    with torch.no_grad():
                        obs,*_=env.step(a)
                    if not np.isfinite(d.qpos).all(): raise RuntimeError("Nonfinite executed state")
                result=telemetry.finish(case,controller,out/controller/case["id"],time.perf_counter()-started)
                if result["model_sha256"]!=expected_model: raise RuntimeError("Physical model changed")
                result["reference_sha256"]=hashlib.sha256(pack.read_bytes()).hexdigest()
                result["initial_state_sha256"]=hashlib.sha256(telemetry.initial_state.tobytes()).hexdigest()
                (out/controller/case["id"]/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
                results.append(result)
                print("LAB_CASE",json.dumps(result),flush=True)
            finally:
                mujoco.mj_step=original_step
                d.xfrc_applied[:]=0
                if mpc: mpc.close()
    finally:
        mujoco.mj_step=original_step
        sim.shutdown()
    (out/(controller+".json")).write_text(json.dumps(results,indent=2),encoding="utf-8")
    return results

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--controller",choices=["protomotions","mjpc","pd"],default="protomotions")
    p.add_argument("--cases",nargs="*")
    p.add_argument("--samples",type=int,default=32)
    p.add_argument("--prepare",action="store_true")
    p.add_argument("--output-dir",type=Path)
    p.add_argument("--reference-file",type=Path)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[2]
    if args.prepare: print(json.dumps(prepare_pack(root,root/"out/controller-comparison/references.pt"),indent=2))
    else: run(root,args.controller,args.cases,args.samples,args.output_dir,args.reference_file)
