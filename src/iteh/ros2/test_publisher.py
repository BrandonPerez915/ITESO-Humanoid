"""
ROS 2 Commander & Test Node for ITESO Humanoid.
Demonstrates interacting with `HumanoidPolicyNode` inside the ROS 2 container via services and topics.
"""

from __future__ import annotations

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from common.logging import get_logger

log = get_logger("ros2.test")


class CmdVelTestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("iteh_commander_node")
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_mode = self.create_publisher(Int32, "/robot_mode", 10)

        self.cli_init = self.create_client(Trigger, "/iteh/set_mode_init")
        self.cli_run = self.create_client(Trigger, "/iteh/set_mode_run")
        self.cli_idle = self.create_client(Trigger, "/iteh/set_mode_idle")
        self.cli_reset = self.create_client(Trigger, "/iteh/reset_policy")
        self.cli_estop = self.create_client(Trigger, "/iteh/emergency_stop")

        log.info("CmdVelTestPublisher initialized (automated test sequence towards HumanoidPolicyNode).")

    def call_trigger(self, client, timeout_sec: float = 1.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            log.warning("Service %s unavailable, falling back to /robot_mode topic.", client.srv_name)
            return False
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.result() is not None:
            res = future.result()
            log.info("[%s] %s", client.srv_name, res.message)
            return bool(res.success)
        return False

    def set_mode(self, mode: int) -> None:
        msg = Int32()
        msg.data = int(mode)
        self.pub_mode.publish(msg)

    def send_velocity(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        self.pub_vel.publish(msg)


def run_test_publisher() -> None:
    if not rclpy.ok():
        rclpy.init()
    node = CmdVelTestPublisher()

    try:
        log.info("[1/5] Initializing posture (RL_INIT)...")
        if not node.call_trigger(node.cli_init):
            node.set_mode(2)
        time.sleep(2.2)

        log.info("[2/5] Activating RL policy (RL_RUNNING)...")
        if not node.call_trigger(node.cli_run):
            node.set_mode(3)
        time.sleep(0.5)

        log.info("[3/5] Walking forward (Vx = 0.35 m/s)...")
        start = time.time()
        while time.time() - start < 4.0:
            node.send_velocity(vx=0.35)
            time.sleep(0.1)

        log.info("[4/5] Turning left (Vyaw = 0.4 rad/s)...")
        start = time.time()
        while time.time() - start < 4.0:
            node.send_velocity(vx=0.2, vyaw=0.4)
            time.sleep(0.1)

        log.info("[5/5] Stopping robot and switching to IDLE...")
        for _ in range(5):
            node.send_velocity(vx=0.0, vy=0.0, vyaw=0.0)
            time.sleep(0.1)
        if not node.call_trigger(node.cli_idle):
            node.set_mode(1)

        log.info("Test sequence completed successfully.")

    except KeyboardInterrupt:
        log.warning("Test cancelled by user. Triggering emergency stop.")
        node.call_trigger(node.cli_estop)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
