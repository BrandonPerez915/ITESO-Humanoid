import os
import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import numpy as np
import torch

try:
    from bhl.control.policy import RlController, Cfg
    from bhl.envs.mujoco import MujocoSimulator
except ImportError:
    from src.iteh.control.policy.rl_controller import RlController
    from src.environments.mujoco import MujocoSimulator, Cfg


# Main execution block
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
