"""`iteh calibrate` subcommands: `joints` and `offset`."""

from __future__ import annotations

import argparse


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_cal = subparsers.add_parser(
        "calibrate",
        parents=[common_parent],
        help="Calibration of joint limits and motor electrical offsets.",
        description="Encoder and CAN actuator calibration commands.",
    )
    cal_sub = p_cal.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # 1. iteh calibrate joints
    p_joints = cal_sub.add_parser(
        "joints",
        parents=[common_parent],
        help="Calibrate position offsets for all joints and save to calibration.yaml.",
    )
    p_joints.add_argument("--output", type=str, default=None, help="Optional path to save calibration.yaml.")
    p_joints.set_defaults(func=_cmd_cal_joints, command_id="calibrate.joints", allow_unknown_args=False)

    # 2. iteh calibrate offset
    p_offset = cal_sub.add_parser(
        "offset",
        parents=[common_parent],
        help="Run electrical offset calibration sequence on a CAN actuator.",
    )
    p_offset.add_argument("channel", type=str, help="CAN channel (e.g. can0, can1).")
    p_offset.add_argument("device_id", type=int, help="CAN actuator ID.")
    p_offset.add_argument("--wait", type=float, default=20.0, help="Wait time in seconds (default: 20s).")
    p_offset.set_defaults(func=_cmd_cal_offset, command_id="calibrate.offset", allow_unknown_args=False)


def _cmd_cal_joints(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.calibrate import calibrate_joints

    calibrate_joints(output_path=args.output)
    return 0


def _cmd_cal_offset(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.calibrate import calibrate_electrical_offset

    calibrate_electrical_offset(channel=args.channel, device_id=args.device_id, wait_sec=args.wait)
    return 0
