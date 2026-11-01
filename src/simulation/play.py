import numpy as np
import torch

from common.logging import get_logger
from iteh.policy import PolicyConfig, RlController
from simulation.mjpython import ensure_mjpython
from simulation.mujoco import MujocoSimulator

log = get_logger("sim.play")


def run_play(config_name: str = "biped_25hz.yaml", control_mode: str = "keyboard") -> None:
    """Run the MuJoCo simulation environment with the specified policy and control mode."""
    ensure_mjpython()
    cfg = PolicyConfig.load(config_name=config_name, control_mode=control_mode)
    if not cfg:
        raise ValueError("Failed to load config.")

    robot = MujocoSimulator(cfg)  # type: ignore
    obs = robot.reset()

    controller = RlController(cfg)
    controller.load_policy()

    default_actions = np.array(cfg.default_joint_positions, dtype=np.float32)[robot.cfg.action_indices]

    log.info("Starting MuJoCo simulation main loop (control=%s)...", control_mode)
    try:
        while robot.mj_viewer.is_running():
            actions = robot.get_remote_actions()
            if actions is None:
                actions = controller.update(obs.numpy())

            if actions is None:
                actions = default_actions

            actions_tensor = torch.tensor(actions, dtype=torch.float32)
            obs = robot.step(actions_tensor)
    except KeyboardInterrupt:
        log.info("Simulation stopped by user.")
    finally:
        robot.command_controller.stop()
        log.info("Simulation finished cleanly.")
