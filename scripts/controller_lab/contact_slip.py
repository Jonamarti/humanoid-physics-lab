"""Contact-point slip diagnostic; retains the old foot-origin translation proxy."""
import argparse,json
from pathlib import Path
import numpy as np,mujoco

def tangent_velocity(m,d,body,point,normal):
    jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
    mujoco.mj_jac(m,d,jp,jr,point,body)
    velocity=jp@d.qvel
    return velocity-normal*np.dot(velocity,normal)

def inspect(path):
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"));d=mujoco.MjData(m)
    a=dict(np.load(path/"recording.npz"));dyn=dict(np.load(path/"dynamics.npz"))
    groups=[{m.body(side+p).id for p in ("Foot","ToeBase")} for side in ("Left","Right")]
    sums=np.zeros(2);maximum=np.zeros(2);speeds=[];last=a["frames"][0,0]
    max_pen=dict(depth_m=0.)
    for idx,f in enumerate(a["frames"]):
        d.qpos[:]=f[1:];d.qvel[:]=dyn["qvel"][idx];d.ctrl[:]=a["controls"][idx]
        mujoco.mj_forward(m,d);dt=f[0]-last;last=f[0];w=np.zeros(2);v=np.zeros(2)
        for ci,c in enumerate(d.contact):
            if c.efc_address<0:continue
            b1,b2=m.geom_bodyid[[c.geom1,c.geom2]]
            if b1!=0 and b2!=0:continue
            b=int(b2 if b1==0 else b1)
            if -c.dist>max_pen["depth_m"]:max_pen=dict(depth_m=float(-c.dist),body=m.body(b).name,time_s=float(f[0]))
            force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
            n=max(0.,force[0])
            if n<5:continue
            normal=np.asarray(c.frame).reshape(3,3)[0]
            tangent=tangent_velocity(m,d,b,c.pos,normal)
            for i,g in enumerate(groups):
                if b in g:w[i]+=n;v[i]+=n*np.linalg.norm(tangent)
        speed=v/np.maximum(w,1)
        for i in range(2):
            if w[i]>42:
                sums[i]+=speed[i]*dt
                maximum[i]=max(maximum[i],sums[i])
            else:sums[i]=0
        speeds.append([f[0],*speed,*w])
    report=dict(method="load-weighted material-point tangential velocity integrated per loaded bout, 120Hz",
        max_integrated_contact_slip_m=maximum.tolist(),
        mean_loaded_material_speed_m_s=np.mean(np.array(speeds)[:,1:3],axis=0).tolist(),
        max_floor_penetration_replay=max_pen,
        note="Additional diagnostic. Original 2cm foot-origin displacement gate is preserved, not reclassified.")
    np.savez_compressed(path/"contact-slip.npz",series=np.array(speeds))
    (path/"contact-slip.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(path.name,json.dumps(report),flush=True)
    return report
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("path",type=Path);a=p.parse_args();inspect(a.path)
