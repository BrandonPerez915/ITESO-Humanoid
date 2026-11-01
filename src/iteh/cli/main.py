"""
Main entry point for the unified `iteh` CLI tool.

Usage examples:
  iteh train run  --task Velocity-Berkeley-Humanoid-Lite-Robust-v0 --num_envs 4096 --headless --log
  iteh train play --task Velocity-Berkeley-Humanoid-Lite-Robust-v0 --checkpoint model_3000.pt --log
  iteh sim play   --config biped_25hz.yaml --control ros2 --log
  iteh ros2 node  --config biped_25hz.yaml --log
  iteh robot locomotion --config biped_25hz.yaml --log
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Optional, Sequence

# Ensure src/ and repo root are importable before loading submodules
_SRC_DIR = Path(__file__).resolve().parents[2]
_REPO_ROOT = _SRC_DIR.parent
for _p in (str(_SRC_DIR), str(_REPO_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from common.logging import get_logger, set_exit_code, setup_logging, shutdown_logging
from iteh.cli.commands import assets, calibrate, hw, net, robot, ros2, sim, teleop, train


def build_common_parser() -> argparse.ArgumentParser:
    """Shared parser inherited by the root parser and all subparsers so `--log` works anywhere."""
    common = argparse.ArgumentParser(add_help=False)
    log_group = common.add_argument_group("logging", "Centralized logging system options")
    log_group.add_argument(
        "--log",
        action="store_true",
        default=argparse.SUPPRESS,
        help="Save execution logs to <repo>/logs/<group>/<command>/<timestamp>/.",
    )
    log_group.add_argument(
        "--log-level",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default=argparse.SUPPRESS,
        help="Logger verbosity level (default: INFO).",
    )
    log_group.add_argument(
        "--log-dir",
        type=str,
        default=argparse.SUPPRESS,
        help="Custom root directory for logs (default: <repo>/logs).",
    )
    log_group.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        default=argparse.SUPPRESS,
        help="Show detailed messages in console (equivalent to --log-level DEBUG).",
    )
    log_group.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        default=argparse.SUPPRESS,
        help="Show only warnings and errors in console (equivalent to --log-level WARNING).",
    )
    return common


def build_parser() -> argparse.ArgumentParser:
    """Construct the complete `iteh` argument parser hierarchy."""
    common = build_common_parser()
    parser = argparse.ArgumentParser(
        prog="iteh",
        parents=[common],
        description="Unified CLI for training, simulation, ROS 2, and ITESO Humanoid robot control.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  iteh train run --task Velocity-Berkeley-Humanoid-Lite-Robust-v0 --headless --log\n"
            "  iteh train play --task Velocity-Berkeley-Humanoid-Lite-Robust-v0 --log\n"
            "  iteh sim play --config biped_25hz.yaml --control ros2 --log\n"
            "  iteh ros2 node --config biped_25hz.yaml --log\n"
            "  iteh robot locomotion --config biped_25hz.yaml --log\n"
        ),
    )
    parser.add_argument("--version", action="version", version="iteh 1.1.0")

    subparsers = parser.add_subparsers(dest="group", metavar="<group>", required=True)

    for module in (train, sim, ros2, robot, teleop, calibrate, hw, net, assets):
        module.register(subparsers, common)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Parse CLI arguments, initialize centralized logging, and dispatch the selected command."""
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    parser = build_parser()
    args, extra_args = parser.parse_known_args(raw_argv)

    if not getattr(args, "allow_unknown_args", False) and extra_args:
        parser.error(f"Unrecognized arguments: {' '.join(extra_args)}")

    log_enabled = bool(getattr(args, "log", False))
    verbose = bool(getattr(args, "verbose", False))
    quiet = bool(getattr(args, "quiet", False))
    level = getattr(args, "log_level", None)
    if level is None:
        if verbose:
            level = "DEBUG"
        elif quiet:
            level = "WARNING"
        else:
            level = "INFO"

    log_dir = getattr(args, "log_dir", None)
    command_id = getattr(args, "command_id", f"{getattr(args, 'group', 'cli')}.{getattr(args, 'subcommand', 'run')}")

    setup_logging(
        command=command_id,
        enabled=log_enabled,
        level=level,
        log_dir=log_dir,
        argv=["iteh"] + raw_argv,
    )
    log = get_logger("cli")

    exit_code = 0
    try:
        func = getattr(args, "func", None)
        if func is None:
            parser.print_help()
            exit_code = 1
        else:
            res = func(args, extra_args)
            exit_code = int(res) if isinstance(res, int) else 0
    except KeyboardInterrupt:
        log.info("Execution interrupted by user (Ctrl+C).")
        exit_code = 130
    except SystemExit as exc:
        exit_code = int(exc.code) if isinstance(exc.code, int) else 1
    except Exception as exc:
        log.exception("Error during execution of `iteh %s`: %s", command_id.replace(".", " "), exc)
        exit_code = 1
    finally:
        set_exit_code(exit_code)
        shutdown_logging()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
