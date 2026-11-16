"""Build review videos from saved states only; original wall-clock speed is preserved."""
from pathlib import Path
import json,hashlib,subprocess
import imageio_ffmpeg
from render_gait import render

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/"out/gait/delivery"
FF=imageio_ffmpeg.get_ffmpeg_exe()

def encode(inputs,filters,output):
    args=[FF,"-y","-v","error"]
    for i in inputs:args+=["-i",str(i)]
    subprocess.run(args+["-filter_complex",filters,"-map","[v]","-c:v","libx264","-crf","21","-pix_fmt","yuv420p",str(output)],check=True)
    check(output)
def check(path):
    subprocess.run([FF,"-v","error","-i",str(path),"-f","null","-"],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    cases=ROOT/"out/gait/reachable-lift"
    labels=("Muy lenta","Lenta","Media","Mayor cadencia")
    inputs=[];durations=[]
    for name,label in zip(("slow","medium","fast","higher"),labels):
        source=cases/name;s=json.loads((source/"review.json").read_text(encoding="utf-8"))
        file=OUT/("marcha-"+name+".mp4")
        render(source,file,f"{label} · {s['mean_forward_speed_m_s']:.2f} m/s",slow=(name=="higher"))
        inputs.append(file);durations.append(json.loads((source/"summary.json").read_text(encoding="utf-8"))["duration_s"])
    parts=[]
    for i,duration in enumerate(durations):
        parts.append(f"[{i}:v]scale=640:360,tpad=stop_mode=clone:stop_duration={max(durations)-duration+.05}[p{i}]")
    parts.append("[p0][p1][p2][p3]xstack=inputs=4:layout=0_0|640_0|0_360|640_360:shortest=1[v]")
    encode(inputs,";".join(parts),OUT/"cuatro-velocidades.mp4")
    baseline=ROOT/"out/gait/lateral-ab/fast_baseline"
    render(baseline,OUT/"marcha-anterior.mp4","ANTES · misma cadencia 0.8 s/paso")
    encode([OUT/"marcha-anterior.mp4",OUT/"marcha-fast.mp4"],"[0:v][1:v]vstack=inputs=2[v]",OUT/"marcha-antes-despues.mp4")
    selections=(("punch_forward_8","Frente · 8 Ns · 80 ms"),
        ("punch_left_20","Lateral · 20 Ns · 80 ms"),
        ("kick_forward_36","Frente · 36 Ns · 180 ms"),
        ("punch_backward_36","Atras · 36 Ns · 80 ms · caso fallido"))
    pairs=[]
    for key,title in selections:
        files=[]
        for variant,label in (("baseline","Tracker"),("reactive","Planificador experimental")):
            file=OUT/(key+"_"+variant+".mp4")
            render(ROOT/"out/gait/impacts-final"/(key+"_"+variant),file,title+" · "+label)
            files.append(file)
        output=OUT/(key+"-pareja.mp4")
        encode(files,"[0:v][1:v]vstack=inputs=2[v]",output);pairs.append(output)
    filters="".join(f"[{i}:v]" for i in range(len(pairs)))+f"concat=n={len(pairs)}:v=1:a=0[v]"
    encode(pairs,filters,OUT/"impactos-antes-despues.mp4")
    artifacts={}
    for p in OUT.glob("*.mp4"):
        check(p)
        artifacts[p.name]=dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),decoded=True)
    (OUT/"media-audit.json").write_text(json.dumps(dict(physics_steps=0,real_time_fps=30,
        slow_motion_source_sampling_hz=120,files=artifacts),indent=2),encoding="utf-8")
    print("DELIVERY_COMPLETE",OUT,flush=True)
if __name__=="__main__":main()
