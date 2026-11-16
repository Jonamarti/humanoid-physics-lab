"""Paired physical replay with front/profile and sagittal foot close-ups."""
from pathlib import Path
import argparse,json,hashlib,subprocess
import mujoco,numpy as np,imageio_ffmpeg
from PIL import Image,ImageDraw
from render import font,add_marker,forbidden
mujoco.mj_step=forbidden

def load(path):
    m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"))
    return dict(path=path,m=m,d=mujoco.MjData(m),a=dict(np.load(path/"recording.npz")),
        dyn=dict(np.load(path/"dynamics.npz")),c=np.load(path/"rollover.npz")["contacts"],
        s=json.loads((path/"summary.json").read_text(encoding="utf-8")),
        events=json.loads((path/"review.json").read_text(encoding="utf-8"))["events"],
        r=json.loads((path/"rollover-review.json").read_text(encoding="utf-8")),
        body=mujoco.Renderer(m,height=420,width=300),foot=mujoco.Renderer(m,height=220,width=600))

def draw_case(case,t,title,rate):
    m,d=case["m"],case["d"];a=case["a"]
    j=int(np.argmin(abs(a["frames"][:,0]-t)))
    d.qpos[:]=a["frames"][j,1:];d.qvel[:]=case["dyn"]["qvel"][j];d.ctrl[:]=a["controls"][j]
    mujoco.mj_forward(m,d)
    row=a["rows"][np.argmin(abs(a["rows"][:,0]-t))]
    contact=case["c"][np.argmin(abs(case["c"][:,0]-t))]
    canvas=Image.new("RGB",(600,800),"#111827");draw=ImageDraw.Draw(canvas)
    draw.text((10,6),title,font=font(21),fill="white")
    draw.text((10,32),f"t={t:.2f}s | "+("tiempo real" if rate==1 else "camara lenta x0.25"),font=font(17),fill="#fbbf24")
    for col,az in enumerate((0,90)):
        cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam)
        cam.lookat[:]=[d.qpos[0],d.qpos[1],.88];cam.distance=2.9;cam.azimuth=az;cam.elevation=-12
        case["body"].update_scene(d,cam)
        add_marker(case["body"].scene,np.r_[row[1:3],.01],mujoco.mjtGeom.mjGEOM_SPHERE,[1,.85,.1,1])
        canvas.paste(Image.fromarray(case["body"].render()),(col*300,58))
    cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam)
    cam.lookat[:]=[d.qpos[0]+.02,d.qpos[1],.13];cam.distance=1.25;cam.azimuth=90;cam.elevation=-7
    case["foot"].update_scene(d,cam)
    canvas.paste(Image.fromarray(case["foot"].render()),(0,478))
    draw.text((10,699),"Carga real: talon (naranja) / antepie (azul)",font=font(16),fill="white")
    for i,label in enumerate(("I","D")):
        heel=contact[3+i];fore=contact[5+i];total=heel+fore;y=725+27*i
        draw.text((10,y-2),f"{label}: {total:4.0f} N",font=font(16),fill="white")
        x=115;width=235;fraction=heel/max(1,total)
        draw.rectangle((x,y,x+width,y+13),fill="#374151")
        if total>5:
            draw.rectangle((x,y,x+int(width*fraction),y+13),fill="#fb923c")
            draw.rectangle((x+int(width*fraction),y,x+width,y+13),fill="#38bdf8")
        cop=contact[7+i]
        text=f"COPx {cop*100:4.1f} cm" if np.isfinite(cop) and total>5 else "sin apoyo"
        draw.text((365,y-2),text,font=font(16),fill="#d1d5db")
    count=sum(ev["land_s"]<=t for ev in case["events"])
    draw.text((10,779),f"{count} pasos con despeje >=3cm | COM amarillo",font=font(15),fill="#fbbf24")
    return canvas

def render_pair(before,after,output,rate=1.):
    pair=[load(before),load(after)]
    duration=min(x["s"]["duration_s"] for x in pair)
    n=round(duration*30/rate);output.parent.mkdir(parents=True,exist_ok=True)
    writer=imageio_ffmpeg.write_frames(str(output),(1200,800),fps=30,codec="libx264",quality=7,
        pix_fmt_out="yuv420p",macro_block_size=1,ffmpeg_log_level="error");writer.send(None)
    sheet=Image.new("RGB",(1800,800),"#111827")
    keytimes=[.9,1.45,2.45,3.48,4.,duration-.1]
    keys=[round(t*30/rate) for t in keytimes]
    try:
        for frame in range(n):
            t=frame/30*rate;canvas=Image.new("RGB",(1200,800))
            for i,(case,title) in enumerate(zip(pair,("ANTES · version guardada","DESPUES · coordinacion y absorcion"))):
                canvas.paste(draw_case(case,t,title,rate),(i*600,0))
            writer.send(np.asarray(canvas))
            if frame in keys:
                k=keys.index(frame);sheet.paste(canvas.resize((600,400)),((k%3)*600,(k//3)*400))
                if k==2:canvas.save(output.with_name(output.stem+"-contacto.png"))
    finally:
        writer.close()
        for p in pair:p["body"].close();p["foot"].close()
    sheet.save(output.with_suffix(".png"))
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),"-v","error","-i",str(output),"-f","null","-"],
        check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    print("DECODED",output,flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("before",type=Path);p.add_argument("after",type=Path);p.add_argument("output",type=Path)
    a=p.parse_args();before=a.before.resolve();after=a.after.resolve();out=a.output.resolve()
    fast=out.with_name(out.stem+"-x1.mp4");slow=out.with_name(out.stem+"-x025.mp4")
    render_pair(before,after,fast);render_pair(before,after,slow,rate=.25)
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    subprocess.run([ff,"-y","-v","error","-i",str(fast),"-i",str(slow),"-filter_complex",
        "[0:v][1:v]concat=n=2:v=1:a=0[v]","-map","[v]","-c:v","libx264","-crf","21","-pix_fmt","yuv420p",str(out)],check=True)
    subprocess.run([ff,"-v","error","-i",str(out),"-f","null","-"],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    files=[fast,slow,out]
    out.with_suffix(".json").write_text(json.dumps(dict(physics_steps=0,views=["front","profile","foot_profile"],
        files={p.name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),decoded=True) for p in files}),indent=2),encoding="utf-8")
