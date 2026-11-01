"""Teleoperation IK solver and bimanual control operations."""

from __future__ import annotations

import os
import threading
import time
from typing import Optional
import numpy as np

from common.logging import get_logger
from common.paths import get_urdf_path

log = get_logger("ops.teleop")


class TeleopIkSolver:
    """Differential IK solver for bimanual teleoperation using Pinocchio and Pink."""

    def __init__(self, urdf_path: Optional[str] = None) -> None:
        import meshcat_shapes
        from pink.tasks import FrameTask
        from pink.visualization import start_meshcat_visualizer
        import pinocchio as pin
        import qpsolvers

        self._pin = pin
        resolved_urdf = urdf_path or str(get_urdf_path(biped=False))
        urdf_package_path = os.path.dirname(resolved_urdf)
        self.robot = pin.RobotWrapper.BuildFromURDF(
            resolved_urdf,
            package_dirs=[urdf_package_path],
            root_joint=pin.JointModelFreeFlyer(),
        )

        self.init_base_position = np.array([0.0, 0.0, 0.0])
        self.init_base_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        self.visualizer = start_meshcat_visualizer(self.robot)
        self.viewer = self.visualizer.viewer

        self.end_effectors: list[str] = ["arm_left_elbow_roll", "arm_right_elbow_roll"]
        self.end_effector_ids = [self.robot.model.getFrameId(name) for name in self.end_effectors]
        self.num_end_effectors = len(self.end_effectors)

        for name in self.end_effectors:
            meshcat_shapes.frame(self.viewer[name + "_target"], opacity=0.5)
            meshcat_shapes.frame(self.viewer[name + "_vive"], opacity=0.25)
            meshcat_shapes.frame(self.viewer[name], opacity=1.0)

        self.end_effector_tasks = [
            FrameTask(name, position_cost=1.0, orientation_cost=0.1, lm_damping=1.0)
            for name in self.end_effectors
        ]
        self.base_tasks = [
            FrameTask("base", position_cost=1000.0, orientation_cost=1000.0, lm_damping=1.0)
        ]

        if any(ee_id < 0 or ee_id >= len(self.robot.model.frames) for ee_id in self.end_effector_ids):
            raise ValueError("End effector name not found in model")
        log.info("End effector frame IDs: %s", self.end_effector_ids)

        self.solver = "quadprog" if "quadprog" in qpsolvers.available_solvers else qpsolvers.available_solvers[0]
        self.last_update_time = time.perf_counter()
        self.first_run = True

        self.vive_last_poses = [pin.SE3.Identity() for _ in self.end_effector_ids]
        self.robot_last_poses = [pin.SE3.Identity() for _ in self.end_effector_ids]

        self.pose_data = (np.eye(4), np.eye(4))
        self.button_data = (False, False)
        self.trigger_data = (0.0, 0.0)

    def update_controller(self, bridge_data: dict) -> None:
        self.pose_data[0][:, :] = np.array(bridge_data["left"]["pose"])
        self.pose_data[1][:, :] = np.array(bridge_data["right"]["pose"])
        self.button_data = (bridge_data["left"]["button_pressed"], bridge_data["right"]["button_pressed"])
        self.trigger_data = (bridge_data["left"]["trigger"], bridge_data["right"]["trigger"])

    def update(self, obs: np.ndarray) -> tuple[np.ndarray, tuple[float, float]]:
        import pink
        from pink import solve_ik

        pin = self._pin
        vive_poses = [
            pin.SE3(
                quat=pin.Quaternion(R=self.pose_data[i][0:3, 0:3]),
                translation=self.pose_data[i][0:3, -1],
            )
            for i in range(self.num_end_effectors)
        ]

        button_data = self.button_data
        trigger_data = self.trigger_data
        assert obs.shape == (self.robot.model.nq,)

        data = self.robot.data.copy()
        q = obs.copy()
        q[3:7] = q[[4, 5, 6, 3]]
        pin.forwardKinematics(self.robot.model, data, q)
        pin.framesForwardKinematics(self.robot.model, data, q)
        pin.updateFramePlacements(self.robot.model, data)

        delta_poses = [pin.SE3.Identity() for _ in range(self.num_end_effectors)]

        if self.first_run:
            self.first_run = False
            self.last_update_time = time.perf_counter()
            self.last_button_data = button_data

        for i in range(self.num_end_effectors):
            if button_data[i] and not self.last_button_data[i]:
                self.vive_last_poses[i] = vive_poses[i].copy()
            if not button_data[i] and self.last_button_data[i]:
                self.robot_last_poses[i] = data.oMf[self.end_effector_ids[i]].copy()
            if button_data[i]:
                delta_poses[i] = pin.SE3.Identity()
                delta_poses[i].rotation = (vive_poses[i] * self.vive_last_poses[i].inverse()).rotation
                delta_poses[i].translation = vive_poses[i].translation - self.vive_last_poses[i].translation
        self.last_button_data = button_data

        desired_poses = [
            pin.SE3(
                delta_poses[i].rotation @ self.robot_last_poses[i].rotation,
                self.robot_last_poses[i].translation + delta_poses[i].translation,
            )
            for i in range(self.num_end_effectors)
        ]

        configuration = pink.Configuration(self.robot.model, self.robot.data, q)
        for i, name in enumerate(self.end_effectors):
            self.viewer[name + "_target"].set_transform(desired_poses[i].np)
            self.viewer[name + "_vive"].set_transform(vive_poses[i].np)
            self.viewer[name].set_transform(
                configuration.get_transform_frame_to_world(self.end_effector_tasks[i].frame).np
            )
        self.visualizer.display(configuration.q)

        for i in range(self.num_end_effectors):
            self.end_effector_tasks[i].transform_target_to_world = desired_poses[i]

        self.base_tasks[0].transform_target_to_world = pin.SE3(
            pin.Quaternion.Identity(), np.array([0.0, 0.0, 0.5])
        )
        tasks = self.end_effector_tasks + self.base_tasks

        dt = time.perf_counter() - self.last_update_time
        velocity = solve_ik(configuration, tasks, dt, solver=self.solver, safety_break=False)
        configuration.integrate_inplace(velocity, dt)

        return configuration.q[7:17], trigger_data


def run_teleop(urdf_path: Optional[str] = None, rate_hz: float = 30.0, udp_port: int = 11005) -> None:
    """Run bimanual teleoperation loop with UDP pose input."""
    from cc.udp import UDP
    from loop_rate_limiters import RateLimiter
    from iteh.bimanual import Bimanual

    solver = TeleopIkSolver(urdf_path=urdf_path)
    rate = RateLimiter(rate_hz)
    robot = Bimanual()
    bridge_udp = UDP(recv_addr=("0.0.0.0", udp_port), send_addr=("127.0.0.1", udp_port))

    obs = np.zeros(solver.robot.model.nq)
    obs[0:3] = [0, 0, 0.5]
    obs[3:7] = [1, 0, 0, 0]

    def controller_update() -> None:
        while True:
            bridge_data = bridge_udp.recv_dict()
            if bridge_data is not None:
                solver.update_controller(bridge_data)

    controller_thread = threading.Thread(target=controller_update, name="teleop-udp", daemon=True)
    controller_thread.start()

    robot.start(kp=30, kd=2, torque_limit=2)
    robot_actions = np.zeros((12,), dtype=np.float32)
    log.info("Bimanual teleoperation started (UDP port=%d, rate=%.1f Hz)...", udp_port, rate_hz)

    try:
        while True:
            robot_obs = robot.step(robot_actions * robot.joint_axis_directions) * robot.joint_axis_directions
            obs[7:17] = robot_obs[0:10]
            joint_actions, gripper_actions = solver.update(obs)
            robot_actions[0:10] = joint_actions
            robot_actions[10] = gripper_actions[0]
            robot_actions[11] = gripper_actions[1]
            rate.sleep()
    except KeyboardInterrupt:
        log.info("Teleoperation interrupted by user.")
    finally:
        robot.stop()
        log.info("Bimanual robot stopped.")


def run_teleop_idle(rate_hz: float = 100.0) -> None:
    """Run bimanual arms in zero-position hold / idle mode."""
    from loop_rate_limiters import RateLimiter
    from iteh.bimanual import Bimanual

    np.set_printoptions(precision=3, suppress=True)
    rate = RateLimiter(rate_hz)
    robot = Bimanual()
    robot.start(kp=20, kd=2, torque_limit=0.5)
    log.info("Bimanual arms in IDLE mode (%.1f Hz)...", rate_hz)

    try:
        while True:
            obs = robot.step(np.zeros((12,), dtype=np.float32))
            log.debug("Bimanual obs: %s", obs)
            rate.sleep()
    except KeyboardInterrupt:
        log.info("Stopping bimanual arms due to user interruption.")
    finally:
        robot.stop()
