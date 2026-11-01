"""`iteh net` subcommands: `ping`, `connection`, `can-up`, `can-down`."""

from __future__ import annotations

import argparse


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_net = subparsers.add_parser(
        "net",
        parents=[common_parent],
        help="SocketCAN interface management and CAN connectivity diagnostics.",
        description="CAN bus diagnostic and configuration commands.",
    )
    net_sub = p_net.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # 1. iteh net ping
    p_ping = net_sub.add_parser(
        "ping",
        parents=[common_parent],
        help="Ping a specific actuator on the CAN bus.",
    )
    p_ping.add_argument("channel", type=str, help="CAN channel (e.g. can0).")
    p_ping.add_argument("device_id", type=int, help="CAN actuator ID.")
    p_ping.set_defaults(func=_cmd_net_ping, command_id="net.ping", allow_unknown_args=False)

    # 2. iteh net connection
    p_conn = net_sub.add_parser(
        "connection",
        parents=[common_parent],
        help="Continuously check connectivity of all robot joints.",
    )
    p_conn.add_argument(
        "--humanoid",
        action="store_true",
        default=False,
        help="Check leg joints (Humanoid) instead of arm joints (Bimanual).",
    )
    p_conn.set_defaults(func=_cmd_net_connection, command_id="net.connection", allow_unknown_args=False)

    # 3. iteh net can-up
    p_up = net_sub.add_parser(
        "can-up",
        parents=[common_parent],
        help="Bring up SocketCAN interfaces (can0..can3) at 1 Mbps.",
    )
    p_up.add_argument(
        "--channels",
        nargs="+",
        default=["can0", "can1", "can2", "can3"],
        help="CAN interfaces to bring up.",
    )
    p_up.add_argument("--bitrate", type=int, default=1000000, help="CAN bus bitrate.")
    p_up.set_defaults(func=_cmd_net_can_up, command_id="net.can_up", allow_unknown_args=False)

    # 4. iteh net can-down
    p_down = net_sub.add_parser(
        "can-down",
        parents=[common_parent],
        help="Bring down SocketCAN interfaces (can0..can3).",
    )
    p_down.add_argument(
        "--channels",
        nargs="+",
        default=["can0", "can1", "can2", "can3"],
        help="CAN interfaces to bring down.",
    )
    p_down.set_defaults(func=_cmd_net_can_down, command_id="net.can_down", allow_unknown_args=False)


def _cmd_net_ping(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.network import ping_motor

    ok = ping_motor(channel=args.channel, device_id=args.device_id)
    return 0 if ok else 1


def _cmd_net_connection(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.network import check_connection

    check_connection(humanoid=args.humanoid)
    return 0


def _cmd_net_can_up(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.network import setup_can_interfaces

    return setup_can_interfaces(up=True, channels=args.channels, bitrate=args.bitrate)


def _cmd_net_can_down(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.network import setup_can_interfaces

    return setup_can_interfaces(up=False, channels=args.channels)
