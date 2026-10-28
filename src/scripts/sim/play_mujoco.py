import numpy as np
import torch

from iteh.policy import RlController, PolicyConfig
from train.enviroments.mujoco_env import MujocoSimulator

# Main execution block
def main():
    """Main execution function for the MuJoCo simulation environment."""
    # Load configuration
    cfg = PolicyConfig.from_arguments()
    if not cfg:
        raise ValueError("Failed to load config.")

    # Initialize environment
    robot = MujocoSimulator(cfg) # type: ignore
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
