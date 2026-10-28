import os
import sys
from pathlib import Path
import threading

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[3]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import numpy as np
from cc.udp import UDP

from iteh.policy.config import PolicyConfig
from train.enviroments.mujoco_env import MujocoVisualizer


if __name__ == "__main__":
    """Main execution function for the MuJoCo simulation environment."""
    # Initialize environment
    visualizer = MujocoVisualizer(PolicyConfig(
        {
            "num_joints": 12,
            "physics_dt": 0.001,
        })
    )

    def receive_udp_data(robot_observation_buffer):
        # Setup UDP communication
        udp = UDP(("0.0.0.0", 11000), ("127.0.0.1", 11000))

        """Thread function to receive UDP data."""
        while True:
            robot_observations = udp.recv_numpy(dtype=np.float32)
            if robot_observations is not None:
                robot_observation_buffer[:] = robot_observations

    robot_observation_buffer = np.zeros((35,), dtype=np.float32)

    udp_receive_thread = threading.Thread(target=receive_udp_data, args=(robot_observation_buffer,))
    udp_receive_thread.daemon = True
    udp_receive_thread.start()

    try:
        while visualizer.mj_viewer.is_running():
            visualizer.step(robot_observation_buffer)
    except KeyboardInterrupt:
        print("\n[INFO] Visualizador detenido.")
