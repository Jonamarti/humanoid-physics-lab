"""Isolated CPU adapter. Upstream code and checkpoint remain unchanged."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET
import tempfile

import numpy as np

PROTO_SHA = "13de0cccd06f53ce5c9cf9ec7d1932c7da73610f"
MPC_SHA = "ff572a21e7c2bf9fda62e1862a758da7e9a8719b"
CHECKPOINT_SHA = "c13e56e85494f8c93fe9eed5b07cadef7281c2eb1188231755f858e1d0f340f0"
SEED = 20261116

def create_agent(root: Path, motion_file: Path | None = None, scene_file: Path | None = None):
    """Use the published inference view, with single-device Fabric (no DDP)."""
    root = root.resolve()
    source = root / "out/external/protomotions"
    os.chdir(source)
    sys.path.insert(0, str(source))
    os.environ["WANDB_MODE"] = "disabled"
    os.environ["TORCHDYNAMO_DISABLE"] = "1"
    import torch
    import mujoco
    from lightning.fabric import Fabric
    from protomotions.utils.experiment_configs import load_experiment_configs_from_checkpoint
    from protomotions.simulator.factory import update_simulator_config_for_test
    from protomotions.utils.inference_utils import derive_inference_configs, recipe_hooks
    from protomotions.utils.component_builder import build_env_from_configs
    from protomotions.utils.hydra_replacement import get_class
    from protomotions.simulator.mujoco.simulator import MujocoSimulator

    torch.set_num_threads(2)
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    checkpoint = source / "data/pretrained_models/motion_tracker/anatomic_soma_bones_fsq/inference_last.ckpt"
    motion_file = motion_file or root / "out/controller-comparison/references.pt"
    if not motion_file.exists():
        raise FileNotFoundError("Generate our own references with benchmark.py --prepare first")
    args = argparse.Namespace(checkpoint=str(checkpoint), simulator="mujoco", num_envs=1,
        headless=True, full_eval=True, motion_file=str(motion_file), scenes_file=None,
        overrides=[], inference_config=None)
    training = load_experiment_configs_from_checkpoint(checkpoint)
    training.simulator = update_simulator_config_for_test(
        current_simulator_config=training.simulator, new_simulator="mujoco",
        robot_config=training.robot)
    view = derive_inference_configs(training, args, {
        "simulator.num_envs": 1, "simulator.headless": True,
        "motion_lib.motion_file": str(motion_file),
        "motion_lib.motion_file_shard_indices": None,
    }, recipe_hooks(checkpoint.parent, None))
    # One diagnostic scalar is inert for the policy, and supplies MJPC's residual
    # storage. Both policies use exactly this augmented physical model.
    original_loader = MujocoSimulator._load_mjcf_stripped
    def load_with_residual(*args, **kwargs):
        model = original_loader(*args, **kwargs)
        with tempfile.TemporaryDirectory(prefix="hpl-model-") as tmp:
            path = Path(tmp) / "model.xml"
            mujoco.mj_saveLastXML(str(path), model)
            tree = ET.parse(path)
            if scene_file is not None:
                import json
                scene=json.loads(scene_file.read_text(encoding="utf-8"))
                fragment=ET.fromstring("<worldbody>"+scene["mjcf"]+"</worldbody>")
                for body in fragment:
                    # Adapter-owned auxiliary bodies are excluded from the published
                    # robot ordering, just like the existing parked projectile.
                    for element in body.iter():
                        if "name" in element.attrib:element.set("name","projectile_scene_"+element.get("name"))
                    tree.getroot().find("worldbody").append(body)
            sensor = ET.SubElement(tree.getroot(), "sensor")
            ET.SubElement(sensor, "user", name="lab_cost", dim="1",
                needstage="acc", user="-1 1 0 1")
            tree.write(path, encoding="unicode")
            return mujoco.MjModel.from_xml_path(str(path))
    MujocoSimulator._load_mjcf_stripped = staticmethod(load_with_residual)
    try:
        fabric = Fabric(accelerator="cpu", devices=1, strategy="auto", loggers=[])
        fabric.launch()
        env = build_env_from_configs(view, fabric.device)
        agent = get_class(view.agent._target_)(config=view.agent, env=env, fabric=fabric,
            root_dir=checkpoint.parent)
        agent.setup(training=False)
        agent.load(str(checkpoint), load_env=False, load_training_state=False, for_inference=True)
        agent.eval()
        env.set_perturbations_enabled(False)
        return agent
    finally:
        MujocoSimulator._load_mjcf_stripped = staticmethod(original_loader)

def reset_clip(agent, motion_id: int):
    import torch
    ids = torch.tensor([motion_id], dtype=torch.long)
    manager = agent.env.reference_motion.motion_manager
    manager.set_assignment(ids, torch.zeros(1))
    obs, _ = agent.env.reset(sample_flat=True, disable_motion_resample=True)
    agent.model.reset_rollout_context(env_ids=torch.tensor([0]))
    return obs

def action(agent, obs, step):
    import torch
    with torch.no_grad():
        agent.pre_collect_step(step)
        obs = agent.add_agent_info_to_obs(obs)
        td = agent.obs_dict_to_tensordict(obs)
        return agent.inference_step(td)[agent.model.deployable_action_key]

if __name__ == "__main__":
    import json
    root = Path(__file__).resolve().parents[2]
    agent = create_agent(root)
    lib = agent.motion_lib
    print("LAB_INVENTORY", json.dumps({
        "motions": list(lib.motion_files), "frames": lib.motion_num_frames.tolist(),
        "duration": lib.motion_lengths.tolist(),
        "bodies": agent.env.simulator.model.nbody,
        "joints": agent.env.simulator.model.njnt,
        "actuators": agent.env.simulator.model.nu,
        "physics_dt": agent.env.simulator.model.opt.timestep,
    }))
    result = agent.evaluator.evaluate()
    print("LAB_UPSTREAM_RESULT", json.dumps({"logs": result.logs, "score": result.score,
        "items": result.num_items}, default=float))
    agent.env.simulator.shutdown()
