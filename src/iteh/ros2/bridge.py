# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

"""
ROS Bridge Bidirectional Client Module for ITESO Humanoid.
Intended to run on the host machine (macOS / MuJoCo simulator / hardware bridge)
and communicate with the ROS 2 container over WebSocket via `rosbridge_server` (`roslibpy`).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional, Sequence
import numpy as np

from common.logging import get_logger, status_line

try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False

log = get_logger("ros2.bridge")


class RosBridge:
    """
    Bidirectional ROS Bridge WebSocket client for ITESO Humanoid.
    Bridges the host simulator/hardware with the ROS 2 Policy & Control Node.
    """

    def __init__(
        self,
        watchdog_timeout: float = 0.5,
        host: str = "localhost",
        bridge_port: int = 9090,
        cmd_vel_topic: str = "/cmd_vel",
        mode_topic: str = "/robot_mode",
        obs_topic: str = "/robot_observations",
        imu_topic: str = "/imu/data",
        joint_states_topic: str = "/joint_states",
        forward_cmd_topic: str = "/forward_position_controller/commands",
        state_mode_topic: str = "/robot_state/mode",
        publish_standard_sensors: bool = True,
    ) -> None:
        self.cmd_vel_topic = cmd_vel_topic
        self.mode_topic = mode_topic
        self.obs_topic = obs_topic
        self.imu_topic = imu_topic
        self.joint_states_topic = joint_states_topic
        self.forward_cmd_topic = forward_cmd_topic
        self.state_mode_topic = state_mode_topic
        self.publish_standard_sensors = publish_standard_sensors

        self.watchdog_timeout = watchdog_timeout
        self.host = host or "localhost"
        self.port = bridge_port

        self.commands: Dict[str, float] = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 3,
        }

        self._latest_ros_actions: Optional[np.ndarray] = None
        self._last_action_time = 0.0
        self.remote_state_str: str = "RL_RUNNING"

        self._last_cmd_time = 0.0
        self._stopped = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None

        self._client: Optional[Any] = None
        self._sub_vel: Optional[Any] = None
        self._sub_mode: Optional[Any] = None
        self._sub_forward_cmd: Optional[Any] = None
        self._sub_state_mode: Optional[Any] = None

        self._pub_obs: Optional[Any] = None
        self._pub_imu: Optional[Any] = None
        self._pub_joint_states: Optional[Any] = None
        self._pub_cmd_vel: Optional[Any] = None
        self._pub_mode: Optional[Any] = None

    @property
    def is_connected(self) -> bool:
        return bool(self._client is not None and getattr(self._client, "is_connected", False))

    @property
    def has_remote_policy_actions(self) -> bool:
        return (
            self._latest_ros_actions is not None
            and (time.time() - self._last_action_time) < max(self.watchdog_timeout * 2.0, 1.0)
        )

    def get_latest_actions(self) -> Optional[np.ndarray]:
        if self.has_remote_policy_actions and self._latest_ros_actions is not None:
            return self._latest_ros_actions.copy()
        return None

    def run(self) -> None:
        if not ROSLIBPY_AVAILABLE:
            log.error(
                "'roslibpy' not found. Install with `pip install roslibpy` "
                "and launch rosbridge in container (`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`)."
            )
            return

        self._start_rosbridge()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="rosbridge-watchdog", daemon=True
        )
        self._watchdog_thread.start()

    def _start_rosbridge(self) -> None:
        log.info(
            "Starting rosbridge client -> ws://%s:%d (cmd_vel=%s, mode=%s, obs=%s)",
            self.host,
            self.port,
            self.cmd_vel_topic,
            self.mode_topic,
            self.obs_topic,
        )

        self._client = roslibpy.Ros(host=self.host, port=self.port)
        try:
            self._client.run()
            log.info("rosbridge connection established on ws://%s:%d.", self.host, self.port)
        except Exception as e:
            log.error("Could not connect to rosbridge at ws://%s:%d: %s", self.host, self.port, e)
            return

        self._sub_vel = roslibpy.Topic(self._client, self.cmd_vel_topic, "geometry_msgs/Twist")
        self._sub_vel.subscribe(self._on_cmd_vel)

        self._sub_mode = roslibpy.Topic(self._client, self.mode_topic, "std_msgs/Int32")
        self._sub_mode.subscribe(self._on_mode)

        self._sub_forward_cmd = roslibpy.Topic(
            self._client, self.forward_cmd_topic, "std_msgs/Float64MultiArray"
        )
        self._sub_forward_cmd.subscribe(self._on_forward_commands)

        self._sub_state_mode = roslibpy.Topic(self._client, self.state_mode_topic, "std_msgs/String")
        self._sub_state_mode.subscribe(self._on_state_mode)

        self._pub_obs = roslibpy.Topic(self._client, self.obs_topic, "std_msgs/Float32MultiArray")
        self._pub_obs.advertise()

        if self.publish_standard_sensors:
            self._pub_imu = roslibpy.Topic(self._client, self.imu_topic, "sensor_msgs/Imu")
            self._pub_imu.advertise()

            self._pub_joint_states = roslibpy.Topic(
                self._client, self.joint_states_topic, "sensor_msgs/JointState"
            )
            self._pub_joint_states.advertise()

        self._pub_cmd_vel = roslibpy.Topic(self._client, self.cmd_vel_topic, "geometry_msgs/Twist")
        self._pub_cmd_vel.advertise()

        self._pub_mode = roslibpy.Topic(self._client, self.mode_topic, "std_msgs/Int32")
        self._pub_mode.advertise()

    def _on_cmd_vel(self, message: Dict[str, Any]) -> None:
        lin = message.get("linear", {})
        ang = message.get("angular", {})
        self.commands["velocity_x"] = float(lin.get("x", 0.0))
        self.commands["velocity_y"] = float(lin.get("y", 0.0))
        self.commands["velocity_yaw"] = float(ang.get("z", 0.0))
        self._last_cmd_time = time.time()
        status_line(
            f"[ROS2 | {self.remote_state_str}] "
            f"Vx: {self.commands['velocity_x']:+.2f} | "
            f"Vy: {self.commands['velocity_y']:+.2f} | "
            f"Vyaw: {self.commands['velocity_yaw']:+.2f}"
        )

    def _on_mode(self, message: Dict[str, Any]) -> None:
        mode = int(message.get("data", 3))
        if self.commands["mode_switch"] != mode:
            log.info("Modo actualizado vía /robot_mode -> %d", mode)
        self.commands["mode_switch"] = mode

    def _on_state_mode(self, message: Dict[str, Any]) -> None:
        new_state = str(message.get("data", "RL_RUNNING"))
        if new_state != self.remote_state_str:
            log.debug("Estado remoto ROS 2: %s", new_state)
        self.remote_state_str = new_state

    def _on_forward_commands(self, message: Dict[str, Any]) -> None:
        data = message.get("data", [])
        if data:
            self._latest_ros_actions = np.asarray(data, dtype=np.float32)
            self._last_action_time = time.time()

    def publish_observations(
        self,
        base_quat: Sequence[float],
        base_ang_vel: Sequence[float],
        joint_pos: Sequence[float],
        joint_vel: Sequence[float],
        joint_names: Optional[List[str]] = None,
    ) -> None:
        if not self.is_connected:
            return

        q = [float(x) for x in base_quat]
        w = [float(x) for x in base_ang_vel]
        jp = [float(x) for x in joint_pos]
        jv = [float(x) for x in joint_vel]

        if self._pub_obs is not None:
            packed = q + w + jp + jv
            self._pub_obs.publish(roslibpy.Message({"data": packed}))

        if self.publish_standard_sensors:
            if self._pub_imu is not None:
                imu_msg = {
                    "orientation": {"w": q[0], "x": q[1], "y": q[2], "z": q[3]},
                    "angular_velocity": {"x": w[0], "y": w[1], "z": w[2]},
                }
                self._pub_imu.publish(roslibpy.Message(imu_msg))

            if self._pub_joint_states is not None:
                js_msg: Dict[str, Any] = {
                    "position": jp,
                    "velocity": jv,
                }
                if joint_names is not None:
                    js_msg["name"] = list(joint_names)
                self._pub_joint_states.publish(roslibpy.Message(js_msg))

    def publish_cmd_vel(self, vx: float, vy: float, vyaw: float) -> None:
        self.commands["velocity_x"] = float(np.clip(vx, -1.0, 1.0))
        self.commands["velocity_y"] = float(np.clip(vy, -0.5, 0.5))
        self.commands["velocity_yaw"] = float(np.clip(vyaw, -1.0, 1.0))
        self._last_cmd_time = time.time()

        if self.is_connected and self._pub_cmd_vel is not None:
            twist_msg = {
                "linear": {
                    "x": self.commands["velocity_x"],
                    "y": self.commands["velocity_y"],
                    "z": 0.0,
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": self.commands["velocity_yaw"],
                },
            }
            self._pub_cmd_vel.publish(roslibpy.Message(twist_msg))

    def publish_mode(self, mode: int) -> None:
        self.commands["mode_switch"] = int(mode)
        if self.is_connected and self._pub_mode is not None:
            self._pub_mode.publish(roslibpy.Message({"data": int(mode)}))

    def call_trigger_service(self, service_name: str) -> None:
        if not self.is_connected or self._client is None:
            return

        service = roslibpy.Service(self._client, service_name, "std_srvs/Trigger")
        request = roslibpy.ServiceRequest()
        service.call(
            request,
            callback=lambda res: log.info("[Service %s] %s", service_name, res.get("message", "OK")),
            errback=lambda err: log.error("[Service Error %s] %s", service_name, err),
        )

    def handle_key(self, keycode: int) -> None:
        step_x = 0.1
        step_y = 0.1
        step_yaw = 0.15

        vx = self.commands["velocity_x"]
        vy = self.commands["velocity_y"]
        vyaw = self.commands["velocity_yaw"]

        if keycode in (ord("W"), ord("w"), 265):
            self.publish_cmd_vel(vx + step_x, vy, vyaw)
        elif keycode in (ord("S"), ord("s"), 264):
            self.publish_cmd_vel(vx - step_x, vy, vyaw)
        elif keycode in (ord("A"), ord("a"), 263):
            self.publish_cmd_vel(vx, vy + step_y, vyaw)
        elif keycode in (ord("D"), ord("d"), 262):
            self.publish_cmd_vel(vx, vy - step_y, vyaw)
        elif keycode in (ord("Q"), ord("q")):
            self.publish_cmd_vel(vx, vy, vyaw + step_yaw)
        elif keycode in (ord("E"), ord("e")):
            self.publish_cmd_vel(vx, vy, vyaw - step_yaw)
        elif keycode == 32:
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            self.call_trigger_service("/iteh/emergency_stop")
        elif keycode in (ord("R"), ord("r")):
            self.call_trigger_service("/iteh/reset_policy")
        elif keycode == ord("1"):
            self.publish_mode(1)
            self.call_trigger_service("/iteh/set_mode_idle")
        elif keycode == ord("2"):
            self.publish_mode(2)
            self.call_trigger_service("/iteh/set_mode_init")
        elif keycode == ord("3"):
            self.publish_mode(3)
            self.call_trigger_service("/iteh/set_mode_run")

    def _watchdog_loop(self) -> None:
        while not self._stopped.is_set():
            time.sleep(0.05)
            if self._last_cmd_time > 0 and (time.time() - self._last_cmd_time > self.watchdog_timeout):
                if (
                    abs(self.commands["velocity_x"]) > 1e-4
                    or abs(self.commands["velocity_y"]) > 1e-4
                    or abs(self.commands["velocity_yaw"]) > 1e-4
                ):
                    self.commands["velocity_x"] = 0.0
                    self.commands["velocity_y"] = 0.0
                    self.commands["velocity_yaw"] = 0.0
                    log.warning("Watchdog timeout on /cmd_vel -> velocities set to 0.0")

    def stop(self) -> None:
        self._stopped.set()
        for sub in (self._sub_vel, self._sub_mode, self._sub_forward_cmd, self._sub_state_mode):
            if sub is not None:
                try:
                    sub.unsubscribe()
                except Exception:
                    pass
        for pub in (self._pub_obs, self._pub_imu, self._pub_joint_states, self._pub_cmd_vel, self._pub_mode):
            if pub is not None:
                try:
                    pub.unadvertise()
                except Exception:
                    pass
        if self._client is not None and ROSLIBPY_AVAILABLE:
            try:
                self._client.terminate()
            except Exception:
                pass
        log.info("rosbridge client stopped.")


def run_bridge(host: str = "localhost", port: int = 9090, watchdog_timeout: float = 0.5) -> None:
    """Run the ROS bridge client standalone until interrupted."""
    bridge = RosBridge(watchdog_timeout=watchdog_timeout, host=host, bridge_port=port)
    bridge.run()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        log.info("Stopping rosbridge due to user interruption...")
    finally:
        bridge.stop()
