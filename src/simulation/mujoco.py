# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import threading
import time
import mujoco
import mujoco.viewer
import numpy as np
import torch

from common.logging import get_logger
from common.math import quat_rotate_inverse
from common.paths import get_mjcf_path
from iteh.inputs.gamepad import Se2Gamepad, Se2Keyboard
from iteh.policy.config import PolicyConfig
from simulation.mjpython import ensure_mjpython

log = get_logger("sim.mujoco")


class MujocoEnv:
    def __init__(self, cfg: PolicyConfig, key_callback=None):
        self.cfg = cfg
        ensure_mjpython()

        # Load appropriate MJCF model based on robot configuration using centralized paths
        scene_xml = get_mjcf_path(biped=(cfg.num_joints != 22), scene=True)
        self.mj_model = mujoco.MjModel.from_xml_path(str(scene_xml))

        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = self.cfg.physics_dt
        if key_callback is not None:
            self.mj_viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data, key_callback=key_callback)
        else:
            self.mj_viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)


class MujocoVisualizer(MujocoEnv):
    """MuJoCo visualization environment for the Berkeley Humanoid Lite robot."""
    def __init__(self, cfg: PolicyConfig):
        super().__init__(cfg)

        self.num_dofs = self.mj_model.nu
        log.info("Number of DOFs: %d", self.num_dofs)

    def reset(self) -> None:
        """Reset the simulation environment to initial state."""
        self.mj_data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
        self.mj_data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        if hasattr(self.cfg, "default_joint_positions") and self.cfg.default_joint_positions:
            self.mj_data.qpos[7:7 + len(self.cfg.default_joint_positions)] = self.cfg.default_joint_positions
        else:
            self.mj_data.qpos[7:7 + self.num_dofs] = 0
        self.mj_data.qvel[:] = 0

    def step(self, robot_observations: np.ndarray) -> None:
        """Execute one visualization step with the given state observations."""
        robot_base_quat = robot_observations[0:4]
        robot_base_ang_vel = robot_observations[4:7]
        robot_joint_pos = robot_observations[7:7 + self.num_dofs]
        robot_joint_vel = robot_observations[7 + self.num_dofs:7 + self.num_dofs * 2]

        self.mj_data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
        self.mj_data.qpos[3:7] = robot_base_quat
        self.mj_data.qvel[0:3] = np.array([0.0, 0.0, 0.0])
        self.mj_data.qvel[3:6] = robot_base_ang_vel
        self.mj_data.qpos[7:] = robot_joint_pos
        self.mj_data.qvel[6:] = robot_joint_vel

        mujoco.mj_step(self.mj_model, self.mj_data)
        self.mj_viewer.sync()


class MujocoSimulator(MujocoEnv):
    """MuJoCo physics simulation environment for the Berkeley Humanoid Lite robot."""
    def __init__(self, cfg: PolicyConfig):
        control_mode = getattr(cfg, "control_mode", "keyboard")
        if control_mode == "ros2":
            from iteh.ros2.bridge import RosBridge
            self.command_controller = RosBridge()
        elif control_mode == "gamepad":
            self.command_controller = Se2Gamepad()
        else:
            self.command_controller = Se2Keyboard()

        self.command_controller.run()

        super().__init__(cfg, key_callback=self._key_callback)
        self.physics_substeps = int(np.round(self.cfg.policy_dt / self.cfg.physics_dt))

        # Initialize simulation parameters
        self.sensordata_dof_size = 3 * self.mj_model.nu
        self.gravity_vector = torch.tensor([0.0, 0.0, -1.0])

        # Initialize control parameters
        self.joint_kp = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.joint_kd = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.effort_limits = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)

        self.joint_kp[:] = torch.tensor(self.cfg.joint_kp)
        self.joint_kd[:] = torch.tensor(self.cfg.joint_kd)
        self.effort_limits[:] = torch.tensor(self.cfg.effort_limits)

        self.n_steps = 0

        log.info(
            "Simulador MuJoCo listo | Policy freq: %.1f Hz | Physics freq: %.1f Hz | Substeps: %d",
            1.0 / self.cfg.policy_dt,
            1.0 / self.cfg.physics_dt,
            self.physics_substeps,
        )

        # Initialize control mode and command variables
        self.is_killed = threading.Event()
        self.mode = 3.0  # Default to RL control mode
        self.command_velocity_x = 0.0
        self.command_velocity_y = 0.0
        self.command_velocity_yaw = 0.0

    def _key_callback(self, keycode: int) -> None:
        """Forward MuJoCo viewer keyboard events to controller if supported."""
        if hasattr(self.command_controller, "handle_key"):
            self.command_controller.handle_key(keycode)

    def reset(self) -> torch.Tensor:
        """Reset the simulation environment to initial state."""
        self.mj_data.qpos[0:3] = self.cfg.default_base_position
        self.mj_data.qpos[3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0])
        self.mj_data.qpos[7:] = self.cfg.default_joint_positions
        self.mj_data.qvel[:] = 0

        observations = self._get_observations()
        return observations

    def step(self, actions: torch.Tensor) -> torch.Tensor:
        """Execute one simulation step with the given actions."""
        step_start_time = time.perf_counter()

        for _ in range(self.physics_substeps):
            self._apply_actions(actions)
            mujoco.mj_step(self.mj_model, self.mj_data)

        self.mj_viewer.sync()
        observations = self._get_observations()

        # Maintain real-time simulation
        time_until_next_step = self.cfg.policy_dt - (time.perf_counter() - step_start_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        self.n_steps += 1
        return observations

    def _apply_actions(self, actions: torch.Tensor):
        """Apply PD control actions with torque limits."""
        target_positions = torch.tensor(self.cfg.default_joint_positions, dtype=torch.float32)
        if isinstance(actions, np.ndarray):
            actions = torch.from_numpy(actions).float()
        target_positions[self.cfg.action_indices] = actions

        output_torques = (
            self.joint_kp * (target_positions - self._get_joint_pos())
            + self.joint_kd * (-self._get_joint_vel())
        )
        output_torques_clipped = torch.clip(output_torques, -self.effort_limits, self.effort_limits)
        self.mj_data.ctrl[:] = output_torques_clipped.numpy()

    def _get_base_pos(self) -> torch.Tensor:
        return torch.tensor(self.mj_data.qpos[:3], dtype=torch.float32)

    def _get_base_quat(self) -> torch.Tensor:
        return torch.tensor(
            self.mj_data.sensordata[self.sensordata_dof_size + 0 : self.sensordata_dof_size + 4],
            dtype=torch.float32,
        )

    def _get_base_ang_vel(self) -> torch.Tensor:
        return torch.tensor(
            self.mj_data.sensordata[self.sensordata_dof_size + 4 : self.sensordata_dof_size + 7],
            dtype=torch.float32,
        )

    def _get_projected_gravity(self) -> torch.Tensor:
        base_quat = self._get_base_quat()
        return quat_rotate_inverse(base_quat, self.gravity_vector)

    def _get_joint_pos(self) -> torch.Tensor:
        return torch.tensor(self.mj_data.sensordata[0:self.cfg.num_joints], dtype=torch.float32)

    def _get_joint_vel(self) -> torch.Tensor:
        return torch.tensor(
            self.mj_data.sensordata[self.cfg.num_joints : 2 * self.cfg.num_joints],
            dtype=torch.float32,
        )

    def get_remote_actions(self) -> np.ndarray | None:
        """Returns joint target actions computed by the ROS 2 Policy Node if active."""
        if hasattr(self.command_controller, "get_latest_actions"):
            return self.command_controller.get_latest_actions()
        return None

    def _get_observations(self) -> torch.Tensor:
        """Get complete observation vector for the policy."""
        base_quat = self._get_base_quat()
        base_ang_vel = self._get_base_ang_vel()
        controlled_joint_pos = self._get_joint_pos()[self.cfg.action_indices]
        controlled_joint_vel = self._get_joint_vel()[self.cfg.action_indices]

        if hasattr(self.command_controller, "publish_observations"):
            all_names = list(getattr(self.cfg, "joints", []))
            controlled_names = (
                [all_names[i] for i in self.cfg.action_indices]
                if len(all_names) >= self.cfg.num_joints
                else None
            )
            self.command_controller.publish_observations(
                base_quat=base_quat.tolist(),
                base_ang_vel=base_ang_vel.tolist(),
                joint_pos=controlled_joint_pos.tolist(),
                joint_vel=controlled_joint_vel.tolist(),
                joint_names=controlled_names,
            )

        command_mode_switch = self.command_controller.commands["mode_switch"]
        command_velocity_x = self.command_controller.commands["velocity_x"]
        command_velocity_y = self.command_controller.commands["velocity_y"]
        command_velocity_yaw = self.command_controller.commands["velocity_yaw"]

        if command_mode_switch != 0:
            self.mode = command_mode_switch
        self.command_velocity_x = command_velocity_x
        self.command_velocity_y = command_velocity_y * 0.5
        self.command_velocity_yaw = command_velocity_yaw

        return torch.cat(
            [
                base_quat,
                base_ang_vel,
                controlled_joint_pos,
                controlled_joint_vel,
                torch.tensor(
                    [self.mode, self.command_velocity_x, self.command_velocity_y, self.command_velocity_yaw],
                    dtype=torch.float32,
                ),
            ],
            dim=-1,
        )
