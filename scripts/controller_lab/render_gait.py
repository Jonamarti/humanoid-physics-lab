"""Replay real trajectories with two views; never integrate physics."""
import argparse,json,subprocess
from pathlib import Path
import mujoco,numpy as np,imageio_ffmpeg
from PIL import Image,ImageDraw
from render import font,add_marker,forbidden
mujoco.mj_step=forbidden

def render(path,output,title=None,slow=False,rate=1.):
    path=path.resolve();output=output.resolve();output.parent.mkdir(parents=True,exist_ok=True)
    a=dict(np.load(path/"recording.npz"));s=json.loads((path/"summary.json").read_text(encoding="utf-8"))
    review=json.loads((path/"review.json").read_text(encoding="utf-8"))
    dyn=dict(np.load(path/"dynamics.npz")) if (path/"dynamics.npz").exists() else None
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"));d=mujoco.MjData(m)
    renderer=mujoco.Renderer(m,height=420,width=480)
    cameras=[]
    for az in (0,90):
        c=mujoco.MjvCamera();mujoco.mjv_defaultCamera(c)
        c.distance=2.7;c.azimuth=az;c.elevation=-12;cameras.append(c)
    fps=30;n=round(s["duration_s"]*fps/rate)
    w=imageio_ffmpeg.write_frames(str(output),(960,540),fps=fps,codec="libx264",quality=7,
        pix_fmt_out="yuv420p",macro_block_size=1,ffmpeg_log_level="error");w.send(None)
    sheet=Image.new("RGB",(1440,540),"#111827")
    keys=np.linspace(0,n-1,6).astype(int).tolist()
    try:
        for k in range(n):
            t=k/fps*rate
            idx=np.argmin(abs(a["frames"][:,0]-t));r=a["rows"][np.argmin(abs(a["rows"][:,0]-t))]
            d.qpos[:]=a["frames"][idx,1:];d.ctrl[:]=a["controls"][idx]
            if dyn is not None:d.qvel[:]=dyn["qvel"][idx]
            mujoco.mj_forward(m,d)
            im=Image.new("RGB",(960,540),"#111827");draw=ImageDraw.Draw(im)
            for i,c in enumerate(cameras):
                c.lookat[:]=[d.qpos[0],d.qpos[1],.86]
                renderer.update_scene(d,c)
                add_marker(renderer.scene,np.r_[r[1:3],.01],mujoco.mjtGeom.mjGEOM_SPHERE,[1,.8,.05,1])
                for j,name in enumerate(("LeftFoot","RightFoot")):
                    p=d.xpos[m.body(name).id].copy()
                    if r[7+j]>1:add_marker(renderer.scene,p,mujoco.mjtGeom.mjGEOM_ARROW,[.1,.85,.4,1],p+[0,0,min(.5,r[7+j]*.001)])
                force=dyn["chest_force"][idx,:3] if dyn is not None else np.zeros(3)
                if np.linalg.norm(force)>1:
                    p=d.xpos[m.body("Chest").id].copy()
                    add_marker(renderer.scene,p,mujoco.mjtGeom.mjGEOM_ARROW,[1,.2,.2,1],p+force/max(np.linalg.norm(force),1)*.6)
                im.paste(Image.fromarray(renderer.render()),(i*480,40))
            draw.text((12,7),(title or s["case"])+(" | tiempo real x1" if rate==1 else " | CAMARA LENTA x0.25"),font=font(20),fill="white")
            count=sum(ev["land_s"]<=t for ev in review["events"])
            draw.text((12,470),f"t={t:.2f}s | pasos >=3cm: {count} | apoyos I/D {r[7]:.0f}/{r[8]:.0f} N",font=font(18),fill="white")
            status="CAIDA" if r[9]<.5*s["initial_root_height_m"] else "frente / perfil · COM amarillo · cargas verdes"
            draw.text((12,502),status,font=font(17),fill="#fbbf24")
            w.send(np.asarray(im))
            if k in keys:sheet.paste(im.resize((480,270)),((keys.index(k)%3)*480,(keys.index(k)//3)*270))
    finally:w.close();renderer.close()
    sheet.save(output.with_suffix(".png"))
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    subprocess.run([ff,"-v","error","-i",str(output),"-f","null","-"],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    if slow:
        render(path,output.with_name(output.stem+"-x025.mp4"),title,False,rate=.25)
    print(output,flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("case",type=Path);p.add_argument("output",type=Path)
    p.add_argument("--title");p.add_argument("--slow",action="store_true");a=p.parse_args()
    render(a.case,a.output,a.title,a.slow)
