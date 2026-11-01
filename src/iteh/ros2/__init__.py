"""
ROS 2 integration package for ITESO Humanoid.
- `bridge.RosBridge` (`Se2Ros2`): Bidirectional WebSocket client via `rosbridge` (`roslibpy`) for the host machine.
- `node.HumanoidPolicyNode` (`ItehNode`): Central native ROS 2 node (`rclpy`) for the ROS 2 container.
"""

from iteh.ros2.bridge import RosBridge, run_bridge

__all__ = ["RosBridge", "run_bridge", "CentralNode", "RobotMode"] # type: ignore


def __getattr__(name: str):
    if name in ("CentralNode", "RobotMode"):
        from iteh.ros2 import central_node
        return getattr(central_node, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
