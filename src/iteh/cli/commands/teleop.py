"""`iteh teleop` subcommands: `run` and `idle`."""

from __future__ import annotations

import argparse


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_teleop = subparsers.add_parser(
        "teleop",
        parents=[common_parent],
        help="Bimanual teleoperation with inverse kinematics (Pink/Pinocchio).",
        description="Bimanual robot teleoperation commands.",
    )
    teleop_sub = p_teleop.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # iteh teleop run
    p_run = teleop_sub.add_parser(
        "run",
        parents=[common_parent],
        help="Run the bimanual teleoperation loop with IK.",
    )
    p_run.add_argument("--urdf", type=str, default=None, help="Optional path to URDF file.")
    p_run.add_argument("--rate", type=float, default=30.0, help="Control loop rate in Hz.")
    p_run.add_argument("--port", type=int, default=11005, help="UDP port of teleoperation bridge.")
    p_run.set_defaults(func=_cmd_teleop_run, command_id="teleop.run", allow_unknown_args=False)

    # iteh teleop idle
    p_idle = teleop_sub.add_parser(
        "idle",
        parents=[common_parent],
        help="Hold bimanual arms in neutral (IDLE) position.",
    )
    p_idle.add_argument("--rate", type=float, default=100.0, help="Control loop rate in Hz.")
    p_idle.set_defaults(func=_cmd_teleop_idle, command_id="teleop.idle", allow_unknown_args=False)


def _cmd_teleop_run(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.teleop import run_teleop

    run_teleop(urdf_path=args.urdf, rate_hz=args.rate, udp_port=args.port)
    return 0


def _cmd_teleop_idle(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.teleop import run_teleop_idle

    run_teleop_idle(rate_hz=args.rate)
    return 0
