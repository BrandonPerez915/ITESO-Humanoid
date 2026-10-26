import os
import sys
from pathlib import Path

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import numpy as np
import torch

from iteh.common.paths import REPO_ROOT, SRC_DIR, get_config_path
from iteh.control.policy import RlController, Cfg
from iteh.envs.mujoco import MujocoSimulator


def main():
    """Main execution function for the MuJoCo simulation environment."""
    # Load configuration
    cfg = Cfg.from_arguments()
    if not cfg:
        raise ValueError("Failed to load config.")

    # Initialize environment
    robot = MujocoSimulator(cfg)
    obs = robot.reset()

    # Initialize and start policy controller
    controller = RlController(cfg)
    controller.load_policy()

    # Default actions for fallback
    default_actions = np.array(cfg.default_joint_positions, dtype=np.float32)[robot.cfg.action_indices]

    # Main control loop
    try:
        while robot.mj_viewer.is_running():
            # Send observations and receive actions
            actions = controller.update(obs.numpy())

            # Use default actions if no actions received
            if actions is None:
                actions = default_actions

            # Execute step
            actions = torch.tensor(actions)
            obs = robot.step(actions)
    except KeyboardInterrupt:
        print("\n[INFO] Simulación detenida por el usuario.")
    finally:
        robot.command_controller.stop()
        print("[INFO] Simulación finalizada limpiamente.")


if __name__ == "__main__":
    main()
