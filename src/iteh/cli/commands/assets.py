"""`iteh assets` subcommands: `onshape-urdf`, `onshape-mjcf`, `post-process`, `urdf-to-usd`."""

from __future__ import annotations

import argparse

from common.logging import get_logger
from iteh.cli.commands.train import _add_app_launcher_args # type: ignore

log = get_logger("cli.assets")


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_assets = subparsers.add_parser(
        "assets",
        parents=[common_parent],
        help="Export and conversion of CAD models (Onshape, URDF, MJCF, USD).",
        description="Tools for robot model generation and conversion.",
    )
    assets_sub = p_assets.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # 1. iteh assets onshape-urdf
    p_urdf = assets_sub.add_parser(
        "onshape-urdf",
        parents=[common_parent],
        help="Export Onshape assembly to URDF format.",
    )
    p_urdf.add_argument("--config", type=str, default="assets/urdf/config.json", help="Path to config.json file.")
    p_urdf.set_defaults(func=_cmd_onshape_urdf, command_id="assets.onshape_urdf", allow_unknown_args=False)

    # 2. iteh assets onshape-mjcf
    p_mjcf = assets_sub.add_parser(
        "onshape-mjcf",
        parents=[common_parent],
        help="Export Onshape assembly to MJCF XML format.",
    )
    p_mjcf.add_argument("--config", type=str, default="assets/mjcf/config.json", help="Path to config.json file.")
    p_mjcf.set_defaults(func=_cmd_onshape_mjcf, command_id="assets.onshape_mjcf", allow_unknown_args=False)

    # 3. iteh assets post-process
    p_post = assets_sub.add_parser(
        "post-process",
        parents=[common_parent],
        help="Run post-processing of mesh paths and sensors on URDF or MJCF.",
    )
    p_post.add_argument("--mode", type=str, default="urdf", choices=["urdf", "mjcf"], help="Format to post-process.")
    p_post.add_argument("--name", type=str, default="berkeley-humanoid-lite", help="Base file name.")
    p_post.set_defaults(func=_cmd_post_process, command_id="assets.post_process", allow_unknown_args=False)

    # 4. iteh assets urdf-to-usd
    p_usd = assets_sub.add_parser(
        "urdf-to-usd",
        parents=[common_parent],
        help="Convert a URDF file to USD format using Isaac Sim.",
    )
    p_usd.add_argument("input", type=str, help="Input URDF file path.")
    p_usd.add_argument("--fix-base", action="store_true", default=False, help="Fix the base link.")
    p_usd.add_argument("--merge-joints", action="store_true", default=False, help="Merge fixed joints.")
    p_usd.add_argument("--joint-stiffness", type=float, default=100.0, help="Joint drive stiffness.")
    p_usd.add_argument("--joint-damping", type=float, default=1.0, help="Joint drive damping.")
    p_usd.add_argument(
        "--joint-target-type",
        type=str,
        default="position",
        choices=["position", "velocity", "none"],
        help="Joint drive control type.",
    )
    p_usd.add_argument("--import-sites", action="store_true", default=True, help="Import <site> tags.")
    p_usd.add_argument(
        "--make-instanceable",
        action="store_true",
        default=True,
        help="Make asset instanceable for efficient cloning.",
    )
    _add_app_launcher_args(p_usd)
    p_usd.set_defaults(func=_cmd_urdf_to_usd, command_id="assets.urdf_to_usd", allow_unknown_args=False)


def _cmd_onshape_urdf(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from utils.onshape_to_urdf import export_onshape_to_urdf

    export_onshape_to_urdf(config_path=args.config)
    return 0


def _cmd_onshape_mjcf(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from utils.onshape_to_mjcf import export_onshape_to_mjcf

    export_onshape_to_mjcf(config_path=args.config)
    return 0


def _cmd_post_process(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from utils.post_process import postprocess_mjcf, postprocess_urdf

    if args.mode == "urdf":
        postprocess_urdf(name=args.name)
    else:
        postprocess_mjcf(name=args.name)
    return 0


def _cmd_urdf_to_usd(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    try:
        from isaaclab.app import AppLauncher
    except ImportError as exc:
        log.error("Isaac Lab is not installed in this environment. Run `iteh assets urdf-to-usd` on the Linux VM.")
        raise SystemExit(1) from exc

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    from utils.urdf_to_usd import convert_urdf_to_usd
    convert_urdf_to_usd(args, simulation_app)
    return 0
