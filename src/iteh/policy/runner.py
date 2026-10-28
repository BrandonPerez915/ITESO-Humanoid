# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

"""
RL Runner for ITESO Humanoid

This module implements a policy runner that executes trained policies (PyTorch or ONNX)
for controlling the ITESO Humanoid robot. It handles UDP communication for
receiving observations and sending actions to the robot.
"""

from typing import Union
from abc import ABC, abstractmethod
import numpy as np
from omegaconf import DictConfig, ListConfig
import torch
import onnxruntime as ort

try:
    from common import math
except ImportError:
    from src.common import math


class Policy(ABC):
    """
    Abstract base class for all policies.
    """
    def __init__(self, checkpoint_path: str):
        pass

    @abstractmethod
    def forward(self, observations: np.ndarray) -> np.ndarray:
        pass


class TorchPolicy(Policy):
    """
    PyTorch policy inference runner.

    Loads and executes PyTorch models for robot control policies.
    """
    def __init__(self, checkpoint_path: str, device: str = "cpu"):
        self.device = device
        self.model: torch.nn.Module = torch.load(checkpoint_path, map_location=self.device)
        self.model.eval()

    def forward(self, observations: np.ndarray) -> np.ndarray:
        observations_tensor = torch.from_numpy(observations).unsqueeze(0).to(self.device)
        actions_tensor = self.model(observations_tensor)
        return actions_tensor.detach().cpu().squeeze(0).numpy()


class OnnxPolicy(Policy):
    """
    ONNX policy inference runner

    Loads and executes ONNX models for robot control policies.
    """
    def __init__(self, checkpoint_path: str):
        self.model: ort.InferenceSession = ort.InferenceSession(checkpoint_path)

        inputs = self.model.get_inputs()
        if inputs:
            self.key = inputs[0].name
        else:
            self.key = "obs"

    def forward(self, observations: np.ndarray) -> np.ndarray:
        return np.array(self.model.run(None, {self.key: observations})[0])


class RlController:
    """
    A class to run trained policies for the ITESO Humanoid robot.

    This class handles the execution of trained policies (PyTorch or ONNX format),
    processes robot observations, and sends control commands via UDP communication.
    """

    @staticmethod
    def quat_rotate_inverse(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
        return math.quat_rotate_inverse_np(quaternion, vector)

    def __init__(self, config: Union[DictConfig, ListConfig]):
        """
        Initialize the PolicyRunner.

        Args:
            config (Union[DictConfig, ListConfig]): Configuration object containing policy parameters
        """
        self.config = config

        # for data logging
        self.data_log = []
        self.counter = 0

        # Initialize robot state buffers
        self.command_velocity = np.array(self.config.command_velocity, dtype=np.float32)

        all_defaults = np.array(self.config.default_joint_positions, dtype=np.float32)
        if hasattr(self.config, "action_indices") and self.config.action_indices is not None and len(self.config.action_indices) == self.config.num_actions:
            self.default_joint_positions = all_defaults[self.config.action_indices]
        elif self.config.num_actions == self.config.num_joints:
            self.default_joint_positions = all_defaults
        else:
            self.default_joint_positions = all_defaults[-self.config.num_actions:]

        self.gravity_vector = np.array([0., 0., -1.], dtype=np.float32)

        # Initialize observation and action buffers
        self.num_frames = int(self.config.history_length) + 1
        self._term_dims = [
            3,  # command_velocity
            3,  # base_ang_vel
            3,  # projected_gravity
            int(self.config.num_actions),  # joint_pos
            int(self.config.num_actions),  # joint_vel
            int(self.config.num_actions),  # prev_actions
        ]
        self._term_histories = [
            np.zeros((self.num_frames, dim), dtype=np.float32)
            for dim in self._term_dims
        ]
        self.policy_observations = np.zeros((1, self.config.num_observations * self.num_frames), dtype=np.float32)
        self.policy_actions = np.zeros((1, self.config.num_actions), dtype=np.float32)
        self.prev_actions = np.zeros((self.config.num_actions,), dtype=np.float32)

    def reset(self) -> None:
        """Reset observation history and previous action buffers."""
        for buf in self._term_histories:
            buf.fill(0.0)
        self.policy_observations.fill(0.0)
        self.policy_actions.fill(0.0)
        self.prev_actions.fill(0.0)

    def load_policy(self) -> None:
        """
        Load the policy model (PyTorch or ONNX)
        """
        from common.paths import get_model_path
        model_checkpoint_path = str(get_model_path(self.config.policy_checkpoint_path))

        # Determine policy format and load appropriate model
        if ".pt" in model_checkpoint_path:
            torch.set_printoptions(precision=2)
            self.policy = TorchPolicy(model_checkpoint_path)
            print("Using Torch runner")

        elif ".onnx" in model_checkpoint_path:
            self.policy = OnnxPolicy(model_checkpoint_path)
            print("Using ONNX runner")

        else:
            raise ValueError("Unrecognized policy format")

    def update(self, robot_observations: np.ndarray) -> np.ndarray:
        """
        Run the policy execution loop.

        This method:
        1. Loads the policy model (PyTorch or ONNX)
        2. Sets up UDP communication
        3. Runs the main control loop
        4. Logs experiment data

        Args:
            robot_observations (np.ndarray): Observations from the robot low-level controller

        Returns:
            np.ndarray: Actions to send to the robot
        """

        # Parse UDP observations
        robot_base_quat = robot_observations[0:4]
        robot_base_ang_vel = robot_observations[4:7]
        robot_joint_pos = robot_observations[7:7 + self.config.num_actions] - self.default_joint_positions
        robot_joint_vel = robot_observations[7 + self.config.num_actions:7 + self.config.num_actions * 2]
        # robot_mode = robot_observations[7 + self.config.num_actions * 2]
        command_velocity = robot_observations[7 + self.config.num_actions * 2 + 1:7 + self.config.num_actions * 2 + 4]

        # Process observations
        base_ang_vel = robot_base_ang_vel
        projected_gravity = self.quat_rotate_inverse(robot_base_quat, self.gravity_vector)
        joint_pos = robot_joint_pos
        joint_vel = robot_joint_vel

        # Update per-term observation history (matches Isaac Lab ObservationManager layout)
        current_terms = (
            command_velocity,
            base_ang_vel,
            projected_gravity,
            joint_pos,
            joint_vel,
            self.prev_actions,
        )
        for buf, val in zip(self._term_histories, current_terms):
            if self.num_frames > 1:
                buf[:-1] = buf[1:]
            buf[-1] = val

        self.policy_observations[0] = np.concatenate(
            [buf.reshape(-1) for buf in self._term_histories], axis=0
        )

        # Execute policy
        self.policy_actions[:] = self.policy.forward(self.policy_observations)

        # Process and scale actions
        policy_actions_clipped = np.clip(self.policy_actions[0],
                                         self.config.action_limit_lower,
                                         self.config.action_limit_upper)
        self.prev_actions[:] = policy_actions_clipped

        policy_actions_scaled = policy_actions_clipped * self.config.action_scale + self.default_joint_positions

        # # Log data
        # data_log.append(np.concatenate([[time.time()], policy_observations.flatten()]).tolist())
        # counter += 1

        # # Save experiment data
        # with open("data_log.json", "w") as f:
        #     json.dump(data_log, f)
        # print("Written experiment data to ./data_log.json")

        return policy_actions_scaled
