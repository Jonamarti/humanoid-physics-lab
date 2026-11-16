"""Front/profile replay of ProtoMotions; no physics integration."""
from pathlib import Path
import json
import subprocess
import numpy as np
import mujoco
import imageio_ffmpeg
from PIL import Image,ImageDraw
from render import font,add_marker,forbidden
mujoco.mj_step=forbidden

def render(root):
    source=root/"out/protomotions-natural-v3/protomotions/natural_step_left"
    review=json.loads((source/"physical-review.json").read_text())
    a=np.load(source/"recording.npz")
    soles=np.load(source/"sole-clearance.npz")["series"]
    m=mujoco.MjModel.from_binary_path(str(source/"model.mjb"))
    d=mujoco.MjData(m)
    renderer=mujoco.Renderer(m,height=460,width=480)
    cameras=[]
    for azimuth in (0,90):
        c=mujoco.MjvCamera();mujoco.mjv_defaultCamera(c)
        c.lookat[:]=a["initial_qpos"][:3]+[.12,0,-.12]
        c.distance=2.8;c.azimuth=azimuth;c.elevation=-12
        cameras.append(c)
    out=root/"out/protomotions-natural-v3/delivery";out.mkdir(exist_ok=True)
    fast=out/"paso-natural-x1.mp4"
    writer=imageio_ffmpeg.write_frames(str(fast),(960,600),fps=30,codec="libx264",
        quality=7,pix_fmt_in="rgb24",pix_fmt_out="yuv420p",macro_block_size=1,ffmpeg_log_level="error")
    writer.send(None)
    sheet=Image.new("RGB",(1440,600),"#111827")
    keys=[24,45,69,70,75,150]
    try:
        for frame in range(180):
            t=frame/30;j=int(np.argmin(abs(a["frames"][:,0]-t)))
            d.qpos[:]=a["frames"][j,1:];d.ctrl[:]=a["controls"][j];mujoco.mj_forward(m,d)
            row=a["rows"][np.argmin(abs(a["rows"][:,0]-t))]
            sole=soles[np.argmin(abs(soles[:,0]-t))]
            image=Image.new("RGB",(960,600),"#111827");draw=ImageDraw.Draw(image)
            for i,camera in enumerate(cameras):
                renderer.update_scene(d,camera)
                add_marker(renderer.scene,row[1:4],mujoco.mjtGeom.mjGEOM_SPHERE,[1,.8,.05,1])
                for k,name in enumerate(("LeftFoot","RightFoot")):
                    p=d.xpos[m.body(name).id]
                    if row[7+k]>1:add_marker(renderer.scene,p,mujoco.mjtGeom.mjGEOM_ARROW,[.1,.85,.4,1],p+[0,0,min(.45,row[7+k]*.001)])
                image.paste(Image.fromarray(renderer.render()),(i*480,40))
            draw.text((14,8),"ProtoMotions · frente",font=font(20),fill="white")
            draw.text((494,8),"Mismo movimiento · perfil",font=font(20),fill="white")
            fraction=row[8]/max(1,row[7]+row[8])
            label="Transferencia de peso" if t<1.7 else "Intento de avance"
            if review["supported_lift_time_s"][0] is not None and t>=review["supported_lift_time_s"][0]:
                label="Pie izquierdo en el aire"
            if review["loaded_landing_time_s"][0] is not None and t>=review["loaded_landing_time_s"][0]:
                label="Aterrizaje y estabilizacion"
            draw.text((14,545),f"t={t:.2f}s · {label} · carga derecha {fraction*100:.0f}%",font=font(17),fill="white")
            draw.text((14,573),f"Despeje real del pie izquierdo: {max(0,sole[1])*100:.1f} cm · avance final: {review['actual_foot_advance_m']*100:.1f} cm",font=font(16),fill="#cbd5e1")
            writer.send(np.asarray(image))
            if frame in keys:
                k=keys.index(frame);sheet.paste(image.resize((480,300)),((k%3)*480,(k//3)*300))
            if frame in (30,54,60,90):image.save(out/("captura-"+str(frame)+".png"))
        sheet.save(out/"paso-natural-fotogramas.png")
    finally:writer.close();renderer.close()
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    final=out/"paso-natural.mp4"
    subprocess.run([ffmpeg,"-y","-loglevel","error","-i",str(fast),"-filter_complex",
        "[0:v]split=2[a][b];[a]setpts=PTS-STARTPTS[f];[b]setpts=4*(PTS-STARTPTS)[s];[f][s]concat=n=2:v=1:a=0,fps=30[v]",
        "-map","[v]","-c:v","libx264","-crf","20","-pix_fmt","yuv420p",str(final)],check=True)
    subprocess.run([ffmpeg,"-v","error","-i",str(final),"-f","null","-"],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    (out/"media-check.json").write_text(json.dumps({"decoded":True,"physics_steps":0,
        "views":["front","side"],"speed_sequence":[1,.25],"review":review},indent=2),encoding="utf-8")
    print(str(final))

if __name__=="__main__":render(Path(__file__).resolve().parents[2])
