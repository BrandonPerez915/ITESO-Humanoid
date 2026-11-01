"""`iteh ros2` subcommands: `node`, `bridge`, and `test`."""

from __future__ import annotations

import argparse



def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_ros2 = subparsers.add_parser(
        "ros2",
        parents=[common_parent],
        help="Native ROS 2 nodes and rosbridge WebSocket client.",
        description="Commands to run the ROS 2 policy node, rosbridge client, or test publishers.",
    )
    ros2_sub = p_ros2.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # iteh ros2 node (inside ROS 2 container)
    p_node = ros2_sub.add_parser(
        "node",
        parents=[common_parent],
        help="Start the central ROS 2 policy, mode, and service node (requires rclpy).",
    )
    p_node.add_argument(
        "--config",
        type=str,
        default="biped_25hz.yaml",
        help="Policy configuration YAML file (in configs/policies/).",
    )
    p_node.add_argument(
        "--policy-rate",
        type=float,
        default=0.0,
        help="Policy loop execution rate in Hz (0.0 auto-detects from YAML).",
    )
    p_node.add_argument(
        "--watchdog-timeout",
        type=float,
        default=0.5,
        help="Watchdog timeout in seconds for /cmd_vel.",
    )
    p_node.add_argument(
        "--init-duration",
        type=float,
        default=2.0,
        help="Duration in seconds of smooth interpolation in RL_INIT mode.",
    )
    p_node.set_defaults(func=_cmd_ros2_node, command_id="ros2.node", allow_unknown_args=True)

    # iteh ros2 bridge (host machine)
    p_bridge = ros2_sub.add_parser(
        "bridge",
        parents=[common_parent],
        help="Start standalone rosbridge WebSocket client on host machine.",
    )
    p_bridge.add_argument("--host", type=str, default="localhost", help="rosbridge server hostname/IP.")
    p_bridge.add_argument("--port", type=int, default=9090, help="rosbridge WebSocket port.")
    p_bridge.add_argument("--watchdog-timeout", type=float, default=0.5, help="Watchdog timeout in seconds.")
    p_bridge.set_defaults(func=_cmd_ros2_bridge, command_id="ros2.bridge", allow_unknown_args=False)

    # iteh ros2 test (inside ROS 2 container)
    p_test = ros2_sub.add_parser(
        "test",
        parents=[common_parent],
        help="Run the automated test sequence for ROS 2 services and velocity commands.",
    )
    p_test.set_defaults(func=_cmd_ros2_test, command_id="ros2.test", allow_unknown_args=False)


def _cmd_ros2_node(args: argparse.Namespace, extra_args: list[str]) -> int:
    from iteh.ros2.central_node import run_node

    run_node(
        config_name=args.config,
        policy_rate=args.policy_rate,
        watchdog_timeout=args.watchdog_timeout,
        init_duration=args.init_duration,
        ros_args=extra_args,
    )
    return 0


def _cmd_ros2_bridge(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.ros2.bridge import run_bridge

    run_bridge(host=args.host, port=args.port, watchdog_timeout=args.watchdog_timeout)
    return 0


def _cmd_ros2_test(args: argparse.Namespace, extra_args: list[str]) -> int:
    del args, extra_args
    from iteh.ros2.test_publisher import run_test_publisher

    run_test_publisher()
    return 0
