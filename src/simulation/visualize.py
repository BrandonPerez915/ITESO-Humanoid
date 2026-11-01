import threading
import numpy as np
from cc.udp import UDP

from common.logging import get_logger
from iteh.policy.config import PolicyConfig
from simulation.mjpython import ensure_mjpython
from simulation.mujoco import MujocoVisualizer

log = get_logger("sim.visualize")


def run_visualizer(num_joints: int = 12, physics_dt: float = 0.001, udp_port: int = 11000) -> None:
    """Run the MuJoCo UDP state visualizer."""
    ensure_mjpython()
    visualizer = MujocoVisualizer(
        PolicyConfig(
            {
                "num_joints": num_joints,
                "physics_dt": physics_dt,
            }
        )
    )

    robot_observation_buffer = np.zeros((35,), dtype=np.float32)

    def receive_udp_data(buffer: np.ndarray) -> None:
        udp = UDP(("0.0.0.0", udp_port), ("127.0.0.1", udp_port))
        while True:
            robot_observations = udp.recv_numpy(dtype=np.float32)
            if robot_observations is not None:
                buffer[:] = robot_observations

    udp_receive_thread = threading.Thread(
        target=receive_udp_data, args=(robot_observation_buffer,), name="udp-visualizer", daemon=True
    )
    udp_receive_thread.start()
    log.info("MuJoCo visualizer listening for UDP observations on port %d...", udp_port)

    try:
        while visualizer.mj_viewer.is_running():
            visualizer.step(robot_observation_buffer)
    except KeyboardInterrupt:
        log.info("Visualizer stopped.")
