from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="biped_25hz.yaml",
        description="Nombre del archivo YAML de configuración de la política (en configs/policies/)",
    )

    policy_rate_arg = DeclareLaunchArgument(
        "policy_rate",
        default_value="0.0",
        description="Frecuencia de ejecución en Hz (0.0 auto-detecta del archivo de configuración)",
    )

    watchdog_timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout",
        default_value="0.5",
        description="Tiempo de espera en segundos para el watchdog de /cmd_vel",
    )

    policy_node = Node(
        package="iteh",  # O ejecutable directo vía python script
        executable="ros2_policy_node",
        name="humanoid_policy_node",
        output="screen",
        parameters=[
            {
                "config": LaunchConfiguration("config"),
                "policy_rate": LaunchConfiguration("policy_rate"),
                "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
                "publish_joint_states": True,
                "publish_forward_commands": True,
            }
        ],
    )

    return LaunchDescription([
        config_arg,
        policy_rate_arg,
        watchdog_timeout_arg,
        policy_node,
    ])
