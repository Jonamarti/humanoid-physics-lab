"""Render stored states only. mj_step is forbidden throughout this process."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
import mujoco
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
from compare import validate_pair

def forbidden(*args,**kwargs): raise RuntimeError("Replay must never integrate physics")
mujoco.mj_step=forbidden

def font(size):
    for p in ("C:/Windows/Fonts/arial.ttf","/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(p).exists(): return ImageFont.truetype(p,size)
    return ImageFont.load_default()

def add_marker(scene,pos,kind,color,end=None):
    if scene.ngeom>=scene.maxgeom: raise RuntimeError("Marker capacity exceeded")
    g=scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g,kind,np.array([.012]*3),np.asarray(pos),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
    if end is not None: mujoco.mjv_connector(g,kind,.009,np.asarray(pos),np.asarray(end))
    scene.ngeom+=1

def render_case(root,case,view,contact=False):
    out=root/"out/controller-comparison"
    dirs=([out/"contacts"/case/c for c in ("enabled","disabled")] if contact else [out/c/case for c in ("mjpc","protomotions")])
    summary=[json.loads((p/"summary.json").read_text()) for p in dirs]
    if not contact: validate_pair(*summary)
    records=[dict(np.load(p/"recording.npz")) for p in dirs]
    if not np.array_equal(records[0]["frames"][:,0],records[1]["frames"][:,0]):
        raise ValueError("Mismatched recording times")
    models=[mujoco.MjModel.from_binary_path(str(p/"model.mjb")) for p in dirs]
    data=[mujoco.MjData(m) for m in models]
    renderers=[mujoco.Renderer(m,height=420,width=480) for m in models]
    camera=mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:]=records[0]["initial_qpos"][:3]+[.25,0,-.1]
    camera.distance=3.2;camera.azimuth=0 if view=="front" else 90;camera.elevation=-15
    if contact:
        if not np.array_equal(records[0]["initial_qpos"],records[1]["initial_qpos"]): raise ValueError("Different contact initial pose")
        if not np.array_equal(np.load(dirs[0]/"command_targets.npy"),np.load(dirs[1]/"command_targets.npy")): raise ValueError("Different contact commands")
    delivery=out/"delivery";delivery.mkdir(exist_ok=True)
    name=("contact-" if contact else "")+case+"-"+view
    fast=delivery/(name+"-fast.mp4")
    writer=imageio_ffmpeg.write_frames(str(fast),(960,500),fps=30,codec="libx264",
        quality=7,pix_fmt_in="rgb24",pix_fmt_out="yuv420p",macro_block_size=1,
        ffmpeg_log_level="error")
    writer.send(None)
    sheet=Image.new("RGB",(1440,750),"#111827")
    frame_count=90 if contact else 180
    keys=[0,15,27,39,60,84] if contact else [0,39,57,69,90,174]
    labels=(["Contactos activados","CONTROL: contactos desactivados"] if contact else ["MPC · prototipo","ProtoMotions · controlador publicado"])
    try:
        for frame in range(frame_count):
            t=frame/30
            canvas=Image.new("RGB",(960,500),"#111827");draw=ImageDraw.Draw(canvas)
            for i,(m,d,renderer,record) in enumerate(zip(models,data,renderers,records)):
                j=int(np.argmin(abs(record["frames"][:,0]-t)))
                r=record["rows"][np.argmin(abs(record["rows"][:,0]-t))]
                d.qpos[:]=record["frames"][j,1:]
                d.ctrl[:]=record["controls"][j]
                mujoco.mj_forward(m,d)
                renderer.update_scene(d,camera)
                add_marker(renderer.scene,r[1:4],mujoco.mjtGeom.mjGEOM_SPHERE,[1.,.8,.05,1.])
                for k,n in enumerate(("LeftFoot","RightFoot")):
                    p=d.xpos[m.body(("a_" if contact else "")+n).id].copy()
                    if r[7+k]>1:
                        add_marker(renderer.scene,p,mujoco.mjtGeom.mjGEOM_ARROW,[.1,.9,.3,1.],
                            p+[0,0,min(.6,r[7+k]*.001)])
                canvas.paste(Image.fromarray(renderer.render()),(i*480,40))
                draw.text((12+i*480,7),labels[i],font=font(18),fill="white")
                text=f"t={t:4.2f}s | pies {r[7]:.0f}/{r[8]:.0f} N | potencia {r[12]:+.0f} W"
                draw.text((10+i*480,462),text,font=font(15),fill="#cbd5e1")
                draw.text((10+i*480,482),summary[i]["outcome"],font=font(13),fill="#fbbf24")
            writer.send(np.asarray(canvas))
            if frame in keys:
                k=keys.index(frame)
                sheet.paste(canvas.resize((480,250)),((k%3)*480,(k//3)*250))
        sheet.crop((0,0,1440,500)).save(delivery/(name+".png"))
    finally:
        writer.close()
        for renderer in renderers:renderer.close()
    final=delivery/(name+".mp4")
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    command=[ffmpeg,"-y","-loglevel","error","-i",str(fast),"-filter_complex",
        "[0:v]split=2[a][b];[a]setpts=PTS-STARTPTS[fast];[b]setpts=4*(PTS-STARTPTS)[slow];[fast][slow]concat=n=2:v=1:a=0,fps=30[v]",
        "-map","[v]","-c:v","libx264","-crf","21","-pix_fmt","yuv420p",str(final)]
    subprocess.run(command,check=True)
    subprocess.run([ffmpeg,"-v","error","-i",str(final),"-f","null","-"],check=True,
        stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    fast.unlink()
    return {"case":case,"view":view,"video":final.name,"sheet":name+".png","decoded":True,
        "physics_steps":0,"speed_sequence":[1,.25]}

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--contacts",action="store_true");p.add_argument("--cases",nargs="*");p.add_argument("--views",nargs="*",default=["front","side"])
    args=p.parse_args();root=Path(__file__).resolve().parents[2]
    cases=args.cases or (["arm_push","leg_push","body_support"] if args.contacts else [p.name for p in (root/"out/controller-comparison/mjpc").iterdir() if p.is_dir()])
    results=[]
    for case in cases:
        for view in args.views:
            r=render_case(root,case,view,args.contacts);results.append(r);print(json.dumps(r),flush=True)
    check=root/"out/controller-comparison/delivery/media-check.json"
    existing=json.loads(check.read_text()) if check.exists() else []
    updated={r["video"]:r for r in existing+results}
    check.write_text(json.dumps(list(updated.values()),indent=2),encoding="utf-8")
