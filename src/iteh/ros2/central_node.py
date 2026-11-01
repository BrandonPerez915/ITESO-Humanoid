# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

"""
Native ROS 2 Policy, Mode & Command Node for ITESO Humanoid.
Intended to run inside the ROS 2 Humble container where `rclpy` is installed.
"""

from __future__ import annotations

from enum import IntEnum
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger

from common.logging import get_logger
from iteh.policy.config import PolicyConfig
from iteh.policy.runner import RlController

log = get_logger("ros2.node")


class RobotMode(IntEnum):
    """Control states managed by the ROS 2 Policy Node."""
    INVALID = 0
    IDLE = 1
    RL_INIT = 2
    RL_RUNNING = 3


def linear_interpolate(start: np.ndarray, end: np.ndarray, percentage: float) -> np.ndarray:
    """Linearly interpolates between start and end joint configurations."""
    alpha = min(max(percentage, 0.0), 1.0)
    return start * (1.0 - alpha) + end * alpha


class CentralNode(Node):
    """
    Central ROS 2 Node that encapsulates:
      1. Velocity command handling & safety watchdog
      2. Mode state machine (IDLE -> RL_INIT -> RL_RUNNING)
      3. Neural network policy inference (RlController)
      4. Joint target publication & control services
    """

    def __init__(
        self,
        node_name: str = "humanoid_policy_node",
        config_name: Optional[str] = None,
        policy_rate: float = 0.0,
        watchdog_timeout: float = 0.5,
        init_duration: float = 2.0,
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter("config", config_name or "biped_25hz.yaml")
        self.declare_parameter("policy_rate", float(policy_rate))
        self.declare_parameter("watchdog_timeout", float(watchdog_timeout))
        self.declare_parameter("init_duration", float(init_duration))
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("publish_forward_commands", True)

        cfg_file = str(self.get_parameter("config").value)
        param_rate = float(self.get_parameter("policy_rate").value)
        self.watchdog_timeout = float(self.get_parameter("watchdog_timeout").value)
        self.init_duration = max(float(self.get_parameter("init_duration").value), 0.1)
        self.publish_joint_states_flag = bool(self.get_parameter("publish_joint_states").value)
        self.publish_forward_commands_flag = bool(self.get_parameter("publish_forward_commands").value)

        self.config = PolicyConfig.load(cfg_file, control_mode="ros2")
        self.policy_dt = float(self.config.policy_dt) if param_rate <= 0.0 else (1.0 / param_rate)
        self.policy_rate = 1.0 / self.policy_dt

        self.controller = RlController(self.config)
        self.controller.load_policy()

        self.num_actions = int(self.config.num_actions)
        self.num_joints = int(self.config.num_joints)
        self.action_indices: List[int] = list(self.config.action_indices)
        self.all_joint_names: List[str] = list(
            getattr(self.config, "joints", [f"joint_{i}" for i in range(self.num_joints)])
        )
        self.controlled_joint_names: List[str] = [self.all_joint_names[i] for i in self.action_indices]

        all_defaults = np.array(self.config.default_joint_positions, dtype=np.float32)
        self.default_actions = all_defaults[self.action_indices].copy()

        self.state = RobotMode.RL_RUNNING
        self.next_state = RobotMode.RL_RUNNING
        self.init_percentage = 0.0
        self.starting_positions = self.default_actions.copy()

        self.commands: Dict[str, float] = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": float(RobotMode.RL_RUNNING),
        }

        self.base_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.joint_pos_measured = self.default_actions.copy()
        self.joint_vel_measured = np.zeros(self.num_actions, dtype=np.float32)
        self.target_joint_positions = self.default_actions.copy()

        self._has_sensor_data = False
        self._last_cmd_time = 0.0
        self._stopped = threading.Event()
        self._thread: Optional[threading.Thread] = None

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.cmd_vel_topic = "/cmd_vel"
        self.mode_topic = "/robot_mode"
        self.obs_topic = "/robot_observations"
        self.imu_topic = "/imu/data"
        self.joint_states_topic = "/joint_states"

        self.sub_vel = self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_cmd_vel, sensor_qos
        )
        self.sub_mode = self.create_subscription(
            Int32, self.mode_topic, self._on_mode, reliable_qos
        )
        self.sub_obs = self.create_subscription(
            Float32MultiArray, self.obs_topic, self._on_packed_observations, sensor_qos
        )
        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, sensor_qos
        )
        self.sub_joint_states = self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_states, sensor_qos
        )

        self.forward_cmd_topic = "/forward_position_controller/commands"
        self.joint_cmd_topic = "/joint_command"
        self.state_mode_topic = "/robot_state/mode"

        self.pub_forward_cmd = self.create_publisher(
            Float64MultiArray, self.forward_cmd_topic, reliable_qos
        )
        self.pub_joint_cmd = self.create_publisher(
            JointState, self.joint_cmd_topic, reliable_qos
        )
        self.pub_state_mode = self.create_publisher(
            String, self.state_mode_topic, reliable_qos
        )

        self.srv_reset = self.create_service(
            Trigger, "/iteh/reset_policy", self._handle_reset_policy
        )
        self.srv_estop = self.create_service(
            Trigger, "/iteh/emergency_stop", self._handle_emergency_stop
        )
        self.srv_mode_idle = self.create_service(
            Trigger, "/iteh/set_mode_idle", self._handle_set_mode_idle
        )
        self.srv_mode_init = self.create_service(
            Trigger, "/iteh/set_mode_init", self._handle_set_mode_init
        )
        self.srv_mode_run = self.create_service(
            Trigger, "/iteh/set_mode_run", self._handle_set_mode_run
        )

        self.policy_timer = self.create_timer(self.policy_dt, self._policy_step_callback)
        self.watchdog_timer = self.create_timer(0.05, self._watchdog_callback)

        msg = (
            f"HumanoidPolicyNode inicializado | Config: {cfg_file} | "
            f"Rate: {self.policy_rate:.1f} Hz | DOFs controlados: {self.num_actions}"
        )
        self.get_logger().info(msg)
        log.info(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.commands["velocity_x"] = float(np.clip(msg.linear.x, -1.0, 1.0))
        self.commands["velocity_y"] = float(np.clip(msg.linear.y, -0.5, 0.5))
        self.commands["velocity_yaw"] = float(np.clip(msg.angular.z, -1.0, 1.0))
        self._last_cmd_time = time.time()

    def _on_mode(self, msg: Int32) -> None:
        requested = int(msg.data)
        if requested in (RobotMode.IDLE, RobotMode.RL_INIT, RobotMode.RL_RUNNING):
            self._request_mode_transition(RobotMode(requested))
        else:
            self.get_logger().warning(f"Ignored invalid mode: {requested}")
            log.warning("Ignored invalid mode: %d", requested)

    def _on_packed_observations(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float32)
        expected_min_len = 7 + 2 * self.num_actions
        if data.size < expected_min_len:
            self.get_logger().warning(
                f"Received observation with length {data.size}, expected >= {expected_min_len}"
            )
            return

        self.base_quat[:] = data[0:4]
        self.base_ang_vel[:] = data[4:7]
        self.joint_pos_measured[:] = data[7 : 7 + self.num_actions]
        self.joint_vel_measured[:] = data[7 + self.num_actions : 7 + 2 * self.num_actions]
        self._has_sensor_data = True

    def _on_imu(self, msg: Imu) -> None:
        self.base_quat[0] = float(msg.orientation.w)
        self.base_quat[1] = float(msg.orientation.x)
        self.base_quat[2] = float(msg.orientation.y)
        self.base_quat[3] = float(msg.orientation.z)
        self.base_ang_vel[0] = float(msg.angular_velocity.x)
        self.base_ang_vel[1] = float(msg.angular_velocity.y)
        self.base_ang_vel[2] = float(msg.angular_velocity.z)
        self._has_sensor_data = True

    def _on_joint_states(self, msg: JointState) -> None:
        pos = np.asarray(msg.position, dtype=np.float32)
        vel = np.asarray(msg.velocity, dtype=np.float32)

        if pos.size == self.num_actions:
            self.joint_pos_measured[:] = pos
            if vel.size == self.num_actions:
                self.joint_vel_measured[:] = vel
            self._has_sensor_data = True
        elif pos.size >= self.num_joints:
            self.joint_pos_measured[:] = pos[self.action_indices]
            if vel.size >= self.num_joints:
                self.joint_vel_measured[:] = vel[self.action_indices]
            self._has_sensor_data = True

    def _request_mode_transition(self, target_mode: RobotMode) -> None:
        self.next_state = target_mode
        self.commands["mode_switch"] = float(target_mode)

        if target_mode == RobotMode.IDLE:
            self.state = RobotMode.IDLE
            self.commands["velocity_x"] = 0.0
            self.commands["velocity_y"] = 0.0
            self.commands["velocity_yaw"] = 0.0
            log.info("Transition to Mode 1: IDLE (holding position)")

        elif target_mode == RobotMode.RL_INIT:
            self.state = RobotMode.RL_INIT
            self.starting_positions = self.joint_pos_measured.copy()
            self.init_percentage = 0.0
            self.controller.reset()
            log.info("Transition to Mode 2: RL_INIT (interpolating to nominal posture)")

        elif target_mode == RobotMode.RL_RUNNING:
            self.state = RobotMode.RL_RUNNING
            log.info("Transition to Mode 3: RL_RUNNING (active policy inference)")

    def _handle_reset_policy(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.controller.reset()
        self.commands["velocity_x"] = 0.0
        self.commands["velocity_y"] = 0.0
        self.commands["velocity_yaw"] = 0.0
        response.success = True
        response.message = "Policy observation history and velocity commands reset."
        log.info(response.message)
        return response

    def _handle_emergency_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._request_mode_transition(RobotMode.IDLE)
        self.controller.reset()
        response.success = True
        response.message = "EMERGENCY STOP activated: transitioned to IDLE mode and zeroed velocities."
        log.warning(response.message)
        return response

    def _handle_set_mode_idle(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._request_mode_transition(RobotMode.IDLE)
        response.success = True
        response.message = "Transitioned to IDLE mode (1)."
        return response

    def _handle_set_mode_init(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._request_mode_transition(RobotMode.RL_INIT)
        response.success = True
        response.message = "Transitioned to RL_INIT mode (2)."
        return response

    def _handle_set_mode_run(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._request_mode_transition(RobotMode.RL_RUNNING)
        response.success = True
        response.message = "Transitioned to RL_RUNNING mode (3)."
        return response

    def _policy_step_callback(self) -> None:
        if not self._has_sensor_data:
            return

        if self.state == RobotMode.IDLE:
            self.target_joint_positions[:] = self.joint_pos_measured[:]

        elif self.state == RobotMode.RL_INIT:
            step_increment = self.policy_dt / self.init_duration
            self.init_percentage = min(self.init_percentage + step_increment, 1.0)
            self.target_joint_positions[:] = linear_interpolate(
                self.starting_positions, self.default_actions, self.init_percentage
            )
            if self.init_percentage >= 1.0 and self.next_state == RobotMode.RL_RUNNING:
                self.state = RobotMode.RL_RUNNING

        elif self.state == RobotMode.RL_RUNNING:
            obs_vec = np.concatenate(
                [
                    self.base_quat,
                    self.base_ang_vel,
                    self.joint_pos_measured,
                    self.joint_vel_measured,
                    np.array(
                        [
                            float(self.state),
                            self.commands["velocity_x"],
                            self.commands["velocity_y"] * 0.5,
                            self.commands["velocity_yaw"],
                        ],
                        dtype=np.float32,
                    ),
                ],
                axis=0,
            )
            actions = self.controller.update(obs_vec)
            if actions is not None:
                self.target_joint_positions[:] = np.asarray(actions, dtype=np.float32)

        self._publish_joint_targets()

    def _publish_joint_targets(self) -> None:
        if self.publish_forward_commands_flag:
            fwd_msg = Float64MultiArray()
            fwd_msg.data = [float(val) for val in self.target_joint_positions]
            self.pub_forward_cmd.publish(fwd_msg)

        if self.publish_joint_states_flag:
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = self.controlled_joint_names
            js_msg.position = [float(val) for val in self.target_joint_positions]
            self.pub_joint_cmd.publish(js_msg)

    def _watchdog_callback(self) -> None:
        if self._last_cmd_time > 0 and (time.time() - self._last_cmd_time > self.watchdog_timeout):
            if (
                abs(self.commands["velocity_x"]) > 1e-4
                or abs(self.commands["velocity_y"]) > 1e-4
                or abs(self.commands["velocity_yaw"]) > 1e-4
            ):
                self.commands["velocity_x"] = 0.0
                self.commands["velocity_y"] = 0.0
                self.commands["velocity_yaw"] = 0.0
                log.warning("Watchdog timeout en /cmd_vel -> velocidades puestas a 0.0")

        state_msg = String()
        if self.state == RobotMode.RL_INIT:
            state_msg.data = f"RL_INIT ({self.init_percentage * 100.0:.0f}%)"
        else:
            state_msg.data = self.state.name
        self.pub_state_mode.publish(state_msg)

    def start_in_background(self) -> None:
        self._thread = threading.Thread(target=lambda: rclpy.spin(self), daemon=True)
        self._thread.start()

    def destroy(self) -> None:
        self._stopped.set()
        self.destroy_node()


# Backwards-compatible alias
ItehNode = CentralNode


def run_node(
    config_name: str = "biped_25hz.yaml",
    policy_rate: float = 0.0,
    watchdog_timeout: float = 0.5,
    init_duration: float = 2.0,
    ros_args: Optional[List[str]] = None,
) -> None:
    """Initialize and spin the native ROS 2 HumanoidPolicyNode."""
    if not rclpy.ok():
        rclpy.init(args=ros_args)

    node = CentralNode(
        config_name=config_name,
        policy_rate=policy_rate,
        watchdog_timeout=watchdog_timeout,
        init_duration=init_duration,
    )
    log.info(
        "ROS 2 node active: /%s | Subscriptions: %s, %s, %s, %s, %s",
        node.get_name(),
        node.cmd_vel_topic,
        node.mode_topic,
        node.obs_topic,
        node.imu_topic,
        node.joint_states_topic,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        log.info("ROS 2 policy node stopped by user.")
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()
