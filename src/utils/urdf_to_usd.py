# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Library utility to convert a URDF into USD format using Isaac Sim / Isaac Lab."""

from __future__ import annotations

import contextlib
import os
from pathlib import Path
from typing import Any

from common.logging import get_logger

log = get_logger("assets.urdf_to_usd")


def convert_urdf_to_usd(args_cli: Any, simulation_app: Any) -> None:
    """Convert a URDF file to USD after `AppLauncher` has been initialized."""
    import carb
    import isaacsim.core.utils.stage as stage_utils
    import omni.kit.app

    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
    from isaaclab.utils.assets import check_file_path
    from isaaclab.utils.dict import print_dict

    try:
        urdf_path = Path(args_cli.input)
        if not check_file_path(urdf_path):
            raise ValueError(f"Invalid URDF file path: {urdf_path}")

        source_dir = Path(urdf_path).parent.parent
        usd_path = source_dir / "usd" / urdf_path.name.replace("urdf", "usd")

        urdf_converter_cfg = UrdfConverterCfg(
            asset_path=urdf_path,
            usd_dir=os.path.dirname(usd_path),
            usd_file_name=os.path.basename(usd_path),
            fix_base=args_cli.fix_base,
            merge_fixed_joints=args_cli.merge_joints,
            force_usd_conversion=True,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=args_cli.joint_stiffness,
                    damping=args_cli.joint_damping,
                ),
                target_type=args_cli.joint_target_type,
            ),
            self_collision=True,
            replace_cylinders_with_capsules=True,
        )

        log.info("Converting URDF file: %s -> %s", urdf_path, usd_path)
        print_dict(urdf_converter_cfg.to_dict(), nesting=0)

        urdf_converter = UrdfConverter(urdf_converter_cfg)
        log.info("USD file successfully generated: %s", urdf_converter.usd_path)

        carb_settings_iface = carb.settings.get_settings()
        local_gui = carb_settings_iface.get("/app/window/enabled")
        livestream_gui = carb_settings_iface.get("/app/livestream/enabled")

        if local_gui or livestream_gui:
            stage_utils.open_stage(urdf_converter.usd_path)
            app = omni.kit.app.get_app_interface()
            with contextlib.suppress(KeyboardInterrupt):
                while app.is_running():
                    app.update()
    finally:
        simulation_app.close()
