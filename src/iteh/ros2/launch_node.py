"""
ROS 2 Launch file for ITESO Humanoid Policy & Control Node.
Usage inside ROS 2 Humble container:
  ros2 launch src/iteh/ros2/launch_node.py config:=biped_25hz.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="biped_25hz.yaml",
        description="Policy configuration YAML file name (in configs/policies/)",
    )

    policy_rate_arg = DeclareLaunchArgument(
        "policy_rate",
        default_value="0.0",
        description="Execution rate in Hz (0.0 auto-detects from configuration file)",
    )

    watchdog_timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout",
        default_value="0.5",
        description="Watchdog timeout in seconds for /cmd_vel",
    )

    init_duration_arg = DeclareLaunchArgument(
        "init_duration",
        default_value="2.0",
        description="Interpolation duration in seconds in RL_INIT mode",
    )

    policy_node_proc = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "iteh.cli",
            "ros2",
            "node",
            "--config",
            LaunchConfiguration("config"),
            "--policy-rate",
            LaunchConfiguration("policy_rate"),
            "--watchdog-timeout",
            LaunchConfiguration("watchdog_timeout"),
            "--init-duration",
            LaunchConfiguration("init_duration"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            config_arg,
            policy_rate_arg,
            watchdog_timeout_arg,
            init_duration_arg,
            policy_node_proc,
        ]
    )
