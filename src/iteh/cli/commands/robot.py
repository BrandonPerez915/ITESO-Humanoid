"""`iteh robot` subcommands: `locomotion`, `idle`, `inference`, `kinematic`, `get-config`, `set-config`."""

from __future__ import annotations

import argparse


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_robot = subparsers.add_parser(
        "robot",
        parents=[common_parent],
        help="Control, inference, kinematics, and configuration on the physical robot.",
        description="Direct operation commands for the ITESO Humanoid robot.",
    )
    robot_sub = p_robot.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # 1. iteh robot locomotion
    p_loco = robot_sub.add_parser(
        "locomotion",
        parents=[common_parent],
        help="Run the RL locomotion loop on the physical robot.",
    )
    p_loco.add_argument("--config", type=str, default="biped_25hz.yaml", help="Policy configuration YAML file.")
    p_loco.add_argument("--remote-ip", type=str, default="172.28.0.5", help="Target destination IP for UDP telemetry.")
    p_loco.add_argument("--port", type=int, default=11000, help="Target UDP port for telemetry.")
    p_loco.set_defaults(func=_cmd_locomotion, command_id="robot.locomotion", allow_unknown_args=False)

    # 2. iteh robot idle
    p_idle = robot_sub.add_parser(
        "idle",
        parents=[common_parent],
        help="Keep robot in IDLE mode while streaming observations via UDP.",
    )
    p_idle.add_argument("--remote-ip", type=str, default="172.28.0.5", help="Target destination IP for UDP telemetry.")
    p_idle.add_argument("--port", type=int, default=11000, help="Target UDP port for telemetry.")
    p_idle.set_defaults(func=_cmd_idle, command_id="robot.idle", allow_unknown_args=False)

    # 3. iteh robot inference
    p_inf = robot_sub.add_parser(
        "inference",
        parents=[common_parent],
        help="Run a quick policy inference dry-run test.",
    )
    p_inf.add_argument("--config", type=str, default="humanoid.yaml", help="Policy configuration YAML file.")
    p_inf.set_defaults(func=_cmd_inference, command_id="robot.inference", allow_unknown_args=False)

    # 4. iteh robot kinematic
    p_kin = robot_sub.add_parser(
        "kinematic",
        parents=[common_parent],
        help="Launch inverse kinematics solver (Pink/Pinocchio) with MeshCat.",
    )
    p_kin.add_argument("--urdf", type=str, default=None, help="Optional path to URDF file.")
    p_kin.add_argument("--rate", type=float, default=100.0, help="Control loop rate in Hz.")
    p_kin.set_defaults(func=_cmd_kinematic, command_id="robot.kinematic", allow_unknown_args=False)

    # 5. iteh robot get-config
    p_get = robot_sub.add_parser(
        "get-config",
        parents=[common_parent],
        help="Read configuration from all robot actuators and save to JSON.",
    )
    p_get.add_argument("--output", type=str, default=None, help="Output JSON file path.")
    p_get.set_defaults(func=_cmd_get_config, command_id="robot.get_config", allow_unknown_args=False)

    # 6. iteh robot set-config
    p_set = robot_sub.add_parser(
        "set-config",
        parents=[common_parent],
        help="Write JSON configuration to all robot actuators.",
    )
    p_set.add_argument("--config", type=str, default="motor.json", help="Motor configuration JSON file.")
    p_set.add_argument("--no-flash", action="store_true", default=False, help="Do not save changes to flash memory.")
    p_set.set_defaults(func=_cmd_set_config, command_id="robot.set_config", allow_unknown_args=False)


def _cmd_locomotion(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.robot import run_locomotion

    run_locomotion(config_name=args.config, remote_ip=args.remote_ip, udp_port=args.port)
    return 0


def _cmd_idle(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.robot import run_robot_idle

    run_robot_idle(remote_ip=args.remote_ip, udp_port=args.port)
    return 0


def _cmd_inference(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.robot import run_inference_test

    run_inference_test(config_name=args.config)
    return 0


def _cmd_kinematic(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.robot import run_kinematic_solver

    run_kinematic_solver(urdf_path=args.urdf, rate_hz=args.rate)
    return 0


def _cmd_get_config(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.robot import read_robot_config

    read_robot_config(output_path=args.output)
    return 0


def _cmd_set_config(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.robot import write_robot_config

    write_robot_config(config_name=args.config, store_to_flash=not args.no_flash)
    return 0
