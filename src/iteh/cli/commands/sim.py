"""`iteh sim` subcommands: `play` and `visualize`."""

from __future__ import annotations

import argparse


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_sim = subparsers.add_parser(
        "sim",
        parents=[common_parent],
        help="Simulation and visualization in MuJoCo (auto-handles macOS mjpython).",
        description=(
            "Commands to run the physics simulator or MuJoCo visualizer.\n"
            "Note: On macOS, MuJoCo requires the `mjpython` launcher. The CLI automatically\n"
            "detects macOS and relaunches via `mjpython` seamlessly."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sim_sub = p_sim.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # iteh sim play
    p_play = sim_sub.add_parser(
        "play",
        parents=[common_parent],
        help="Execute a trained policy in the MuJoCo simulator.",
    )
    p_play.add_argument(
        "--config",
        type=str,
        default="biped_25hz.yaml",
        help="Policy configuration YAML file (in configs/policies/).",
    )
    p_play.add_argument(
        "--control",
        type=str,
        default="keyboard",
        choices=["keyboard", "ros2", "gamepad"],
        help="Input controller mode: 'keyboard', 'ros2', or 'gamepad'.",
    )
    p_play.set_defaults(func=_cmd_sim_play, command_id="sim.play", allow_unknown_args=False)

    # iteh sim visualize
    p_vis = sim_sub.add_parser(
        "visualize",
        parents=[common_parent],
        help="Launch the passive MuJoCo visualizer listening for observations via UDP.",
    )
    p_vis.add_argument("--num-joints", type=int, default=12, help="Number of joints (12 biped, 22 humanoid).")
    p_vis.add_argument("--physics-dt", type=float, default=0.001, help="Physics simulation timestep in seconds.")
    p_vis.add_argument("--port", type=int, default=11000, help="UDP port to receive observations.")
    p_vis.set_defaults(func=_cmd_sim_visualize, command_id="sim.visualize", allow_unknown_args=False)


def _cmd_sim_play(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from simulation.mjpython import ensure_mjpython

    ensure_mjpython()
    from simulation.play import run_play

    run_play(config_name=args.config, control_mode=args.control)
    return 0


def _cmd_sim_visualize(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from simulation.mjpython import ensure_mjpython

    ensure_mjpython()
    from simulation.visualize import run_visualizer

    run_visualizer(num_joints=args.num_joints, physics_dt=args.physics_dt, udp_port=args.port)
    return 0
