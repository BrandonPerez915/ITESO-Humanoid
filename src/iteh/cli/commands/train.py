"""`iteh train` subcommands: `run`, `play`, and `list`."""

from __future__ import annotations

import argparse
from typing import Any

from common.logging import get_logger

log = get_logger("cli.train")


def _add_rsl_rl_args(parser: argparse.ArgumentParser) -> None:
    arg_group = parser.add_argument_group("rsl_rl", description="Arguments for the RSL-RL agent.")
    arg_group.add_argument(
        "--experiment_name",
        type=str,
        default=None,
        help="Name of the experiment folder where checkpoints will be saved.",
    )
    arg_group.add_argument("--run_name", type=str, default=None, help="Run name suffix for the run directory.")
    arg_group.add_argument(
        "--resume",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Resume training from a previous checkpoint.",
    )
    arg_group.add_argument("--load_run", type=str, default=None, help="Name of the run folder to load.")
    arg_group.add_argument("--checkpoint", type=str, default=None, help="Checkpoint file to load.")
    arg_group.add_argument(
        "--logger",
        type=str,
        default=None,
        choices=["wandb", "tensorboard", "neptune"],
        help="Metrics logging backend for RSL-RL.",
    )
    arg_group.add_argument(
        "--log_project_name",
        type=str,
        default=None,
        help="Project name in WandB or Neptune.",
    )


def _add_app_launcher_args(parser: argparse.ArgumentParser) -> None:
    """Attach Isaac Lab AppLauncher arguments if available, or compatible fallback flags."""
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(parser)
    except ImportError:
        sim_group = parser.add_argument_group(
            "app_launcher",
            description="Isaac Sim AppLauncher arguments (requires Linux environment with Isaac Sim).",
        )
        sim_group.add_argument("--headless", action="store_true", default=False, help="Run without graphical interface.")
        sim_group.add_argument("--livestream", type=int, default=-1, choices=[-1, 0, 1, 2], help="Livestream mode.")
        sim_group.add_argument("--enable_cameras", action="store_true", default=False, help="Enable cameras.")
        sim_group.add_argument("--device", type=str, default="cuda:0", help="Compute device (e.g. cuda:0, cpu).")
        sim_group.add_argument("--experience", type=str, default="", help="Omniverse experience .kit file.")
        sim_group.add_argument("--kit_args", type=str, default="", help="Extra arguments for Omniverse Kit.")


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_train = subparsers.add_parser(
        "train",
        parents=[common_parent],
        help="Train and evaluate RL policies in Isaac Lab (RSL-RL).",
        description="RL policy training and evaluation commands in Isaac Lab.",
    )
    train_sub = p_train.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # 1. iteh train run
    p_run = train_sub.add_parser(
        "run",
        parents=[common_parent],
        help="Train an RL agent with RSL-RL in Isaac Lab.",
    )
    p_run.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
    p_run.add_argument("--video_length", type=int, default=200, help="Duration of recorded video (in steps).")
    p_run.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
    p_run.add_argument("--num_envs", type=int, default=None, help="Number of parallel environments.")
    p_run.add_argument(
        "--task",
        type=str,
        default="Velocity-Berkeley-Humanoid-Lite-Robust-v0",
        help="Registered Gymnasium task name.",
    )
    p_run.add_argument("--seed", type=int, default=None, help="Random seed for the environment.")
    p_run.add_argument("--max_iterations", type=int, default=None, help="Maximum training iterations.")
    _add_rsl_rl_args(p_run)
    _add_app_launcher_args(p_run)
    p_run.set_defaults(func=_cmd_train_run, command_id="train.run", allow_unknown_args=True)

    # 2. iteh train play
    p_play = train_sub.add_parser(
        "play",
        parents=[common_parent],
        help="Evaluate a trained checkpoint in Isaac Lab and export policy.pt / policy.onnx.",
    )
    p_play.add_argument("--video", action="store_true", default=False, help="Record video during evaluation.")
    p_play.add_argument("--video_length", type=int, default=200, help="Duration of recorded video (in steps).")
    p_play.add_argument(
        "--disable_fabric",
        action="store_true",
        default=False,
        help="Disable Fabric and use USD I/O operations.",
    )
    p_play.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
    p_play.add_argument(
        "--task",
        type=str,
        default="Velocity-Berkeley-Humanoid-Lite-Robust-v0",
        help="Registered Gymnasium task name.",
    )
    _add_rsl_rl_args(p_play)
    _add_app_launcher_args(p_play)
    p_play.set_defaults(func=_cmd_train_play, command_id="train.play", allow_unknown_args=False)

    # 3. iteh train list
    p_list = train_sub.add_parser(
        "list",
        parents=[common_parent],
        help="List registered training tasks for ITESO Humanoid.",
    )
    p_list.set_defaults(func=_cmd_train_list, command_id="train.list", allow_unknown_args=False)


def _cmd_train_run(args: argparse.Namespace, extra_args: list[str]) -> int:
    try:
        from isaaclab.app import AppLauncher
    except ImportError as exc:
        log.error("Isaac Lab is not installed in this environment. Run `iteh train run` on the Linux GPU VM.")
        raise SystemExit(1) from exc

    if args.video:
        args.enable_cameras = True

    log.info("Starting Isaac Sim AppLauncher for training (task=%s)...", args.task)
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    from trainning.train import run_train
    run_train(args, extra_args, simulation_app)
    return 0


def _cmd_train_play(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    try:
        from isaaclab.app import AppLauncher
    except ImportError as exc:
        log.error("Isaac Lab is not installed in this environment. Run `iteh train play` on the Linux GPU VM.")
        raise SystemExit(1) from exc

    if args.video:
        args.enable_cameras = True

    log.info("Starting Isaac Sim AppLauncher for evaluation (task=%s)...", args.task)
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    from trainning.play import run_play_rl
    run_play_rl(args, simulation_app)
    return 0


def _cmd_train_list(args: argparse.Namespace, extra_args: list[str]) -> int:
    del args, extra_args
    from trainning.train import REGISTERED_TASKS

    log.info("Registered tasks in ITESO Humanoid:")
    for task_id, description in REGISTERED_TASKS:
        log.info("  • %-46s : %s", task_id, description)
    return 0
