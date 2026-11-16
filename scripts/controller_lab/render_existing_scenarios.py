"""Diagnostic videos of saved trials only, including failed attacks and falls."""
from pathlib import Path
import argparse,hashlib,json,subprocess
import numpy as np
import mujoco,imageio_ffmpeg
from PIL import Image,ImageDraw
from render import font,add_marker,forbidden
from evidence_existing_scenarios import case_path
mujoco.mj_step=forbidden

LABELS={"punch_forward20":"Puno: 20 N.s hacia delante","punch_backward20":"Puno: 20 N.s hacia atras",
    "kick_left36":"Patada: 36 N.s lateral","kick_backward36":"Patada: 36 N.s hacia atras",
    "outer_reap":"Siega exterior","leg_drive":"Entrada a las piernas"}
def load(path):
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"))
    return dict(path=path,m=m,d=mujoco.MjData(m),a=dict(np.load(path/"recording.npz")),
        dyn=dict(np.load(path/"dynamics.npz")),s=json.loads((path/"summary.json").read_text(encoding="utf-8")),
        review=json.loads((path/"review.json").read_text(encoding="utf-8")),
        renderer=mujoco.Renderer(m,height=360,width=320))
def panel(case,t,title,rate):
    m,d=case["m"],case["d"];a=case["a"];dyn=case["dyn"]
    i=int(np.argmin(abs(a["frames"][:,0]-t)))
    d.qpos[:]=a["frames"][i,1:];d.qvel[:]=dyn["qvel"][i];d.ctrl[:]=a["controls"][i];d.time=a["frames"][i,0]
    mujoco.mj_forward(m,d)
    row=a["rows"][np.argmin(abs(a["rows"][:,0]-t))]
    im=Image.new("RGB",(640,480),"#111827");draw=ImageDraw.Draw(im)
    draw.text((9,5),title,font=font(18),fill="white")
    draw.text((9,29),f"t={t:.2f}s | "+("tiempo real" if rate==1 else "camara lenta x0.25"),font=font(15),fill="#fbbf24")
    target=np.array([d.qpos[0],d.qpos[1],.8]);distance=3.
    if "attacker" in case["s"]:
        other=d.xpos[m.body("projectile_scene_tori_pelvis").id]
        target[:2]=(target[:2]+other[:2])/2;distance=3.6
    for col,az in enumerate((0,90)):
        camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
        camera.lookat[:]=target;camera.distance=distance;camera.azimuth=az;camera.elevation=-15
        renderer=case["renderer"];renderer.update_scene(d,camera)
        add_marker(renderer.scene,np.r_[row[1:3],.01],mujoco.mjtGeom.mjGEOM_SPHERE,[1,.85,.1,1])
        for foot,name in enumerate(("LeftFoot","RightFoot")):
            p=d.xpos[m.body(name).id].copy()
            if row[7+foot]>5:add_marker(renderer.scene,p,mujoco.mjtGeom.mjGEOM_ARROW,[.1,.9,.3,1],p+[0,0,min(.5,row[7+foot]*.001)])
        if "robot_wrenches" in dyn:
            for body,force in zip(dyn["robot_body_ids"],dyn["robot_wrenches"][i,:,:3]):
                n=np.linalg.norm(force)
                if n>2:
                    p=d.xipos[int(body)].copy()
                    add_marker(renderer.scene,p,mujoco.mjtGeom.mjGEOM_ARROW,[1,.25,.15,1],p+force/max(n,1)*min(.5,n/600))
        im.paste(Image.fromarray(renderer.render()),(col*320,53))
    steps=sum(e["land_s"]<=t for e in case["review"]["events"])
    draw.text((9,417),f"Pasos con despeje >=3cm: {steps} | apoyo I/D: {row[7]:.0f}/{row[8]:.0f} N",font=font(15),fill="white")
    fallen=row[9]<.5*case["s"]["initial_root_height_m"]
    if case["s"]["artifacts"]:
        draw.rectangle((0,445,639,479),fill="#7f1d1d")
        status="CAIDA · " if fallen else ""
        draw.text((9,448),status+"DIAGNOSTICO: "+", ".join(case["s"]["artifacts"][:2]),font=font(14),fill="white")
    else:draw.text((9,448),"Control sin artefactos detectados | COM amarillo",font=font(14),fill="#86efac")
    return im

def render(root,out,kind,rate):
    entries=([(n+"_reactive",LABELS[n]+" · planificador activo") for n in ("punch_forward20","punch_backward20","kick_left36","kick_backward36")]
        if kind=="hits" else [(n+"_"+c,LABELS[n]+(" · tracker" if c=="tracker" else " · planificador activo"))
            for n in ("outer_reap","leg_drive") for c in ("tracker","reactive")])
    cases=[load(case_path(root,name)) for name,_ in entries];duration=min(c["s"]["duration_s"] for c in cases)
    out.parent.mkdir(parents=True,exist_ok=True);n=round(duration*30/rate)
    writer=imageio_ffmpeg.write_frames(str(out),(1280,960),fps=30,codec="libx264",quality=7,
        pix_fmt_out="yuv420p",macro_block_size=1,ffmpeg_log_level="error");writer.send(None)
    sheet=Image.new("RGB",(1920,960),"#111827");keys=[round(x*30/rate) for x in (1.8,2.35,2.8,3.5,5.,7.5)]
    try:
        for frame in range(n):
            t=frame/30*rate;im=Image.new("RGB",(1280,960))
            for i,(case,(_,title)) in enumerate(zip(cases,entries)):
                im.paste(panel(case,t,title,rate),((i%2)*640,(i//2)*480))
            writer.send(np.asarray(im))
            if frame in keys:
                k=keys.index(frame);sheet.paste(im.resize((640,480)),((k%3)*640,(k//3)*480))
                if k==2:im.save(out.with_name(out.stem+"-detalle.png"))
    finally:
        writer.close()
        for case in cases:case["renderer"].close()
    sheet.save(out.with_suffix(".png"))
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),"-v","error","-i",str(out),"-f","null","-"],check=True,stderr=subprocess.PIPE)
    print("DECODED",out,flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument("root",type=Path);p.add_argument("output",type=Path);p.add_argument("--kind",choices=["hits","throws"],required=True);a=p.parse_args()
    fast=a.output.with_name(a.output.stem+"-x1.mp4");slow=a.output.with_name(a.output.stem+"-x025.mp4")
    render(a.root.resolve(),fast.resolve(),a.kind,1.);render(a.root.resolve(),slow.resolve(),a.kind,.25)
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),"-y","-v","error","-i",str(fast),"-i",str(slow),
        "-filter_complex","[0:v][1:v]concat=n=2:v=1:a=0[v]","-map","[v]","-c:v","libx264","-crf","21","-pix_fmt","yuv420p",str(a.output)],check=True)
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),"-v","error","-i",str(a.output),"-f","null","-"],check=True,stderr=subprocess.PIPE)
    files=[fast,slow,a.output]
    a.output.with_suffix(".json").write_text(json.dumps(dict(physics_steps=0,decoded=True,kind=a.kind,
        files={f.name:dict(bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in files}),indent=2),encoding="utf-8")
if __name__=="__main__":main()
