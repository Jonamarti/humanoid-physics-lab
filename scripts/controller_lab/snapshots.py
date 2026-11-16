"""Preserve runtime actuator enums: mj_saveLastXML alone omits edited biastype."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from metrics import model_fingerprint

def save_model(m,path):
    path=Path(path)
    mujoco.mj_saveModel(m,str(path/"model.mjb"))
    mujoco.mj_saveLastXML(str(path/"model.xml"),m)
    tree=ET.parse(path/"model.xml")
    for a,element in enumerate(tree.getroot().find("actuator")):
        element.set("gaintype",["fixed","affine","muscle","user"][m.actuator_gaintype[a]])
        element.set("biastype",["none","affine","muscle","user"][m.actuator_biastype[a]])
        element.set("ctrllimited","true" if m.actuator_ctrllimited[a] else "false")
        element.set("forcelimited","true" if m.actuator_forcelimited[a] else "false")
    tree.write(path/"model.xml",encoding="unicode")
    binary=mujoco.MjModel.from_binary_path(str(path/"model.mjb"))
    if model_fingerprint(binary)!=model_fingerprint(m):
        raise RuntimeError("Binary model reload changed physics")
    xml=mujoco.MjModel.from_xml_path(str(path/"model.xml"))
    if not np.array_equal(xml.actuator_biastype,m.actuator_biastype):
        raise RuntimeError("Actuator bias type was lost in XML export")

if __name__=="__main__":
    import json
    from proto_adapter import create_agent
    root=Path(__file__).resolve().parents[2]
    agent=create_agent(root)
    m=agent.env.simulator.model
    for p in (root/"out/controller-comparison").glob("*/*/summary.json"):
        summary=json.loads(p.read_text())
        if summary.get("controller") not in ("pd","mjpc","protomotions"):continue
        if summary["model_sha256"]!=model_fingerprint(m):raise RuntimeError("Cannot reconstruct model")
        save_model(m,p.parent)
    agent.env.simulator.shutdown()
    print("All 27 model snapshots restored and verified against recorded fingerprints")
