"""Generate URDF file from Onshape CAD project."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil

from common.logging import get_logger
from common.paths import REPO_ROOT

log = get_logger("assets.onshape_urdf")


def export_onshape_to_urdf(config_path: str = "assets/urdf/config.json") -> Path:
    """Export an Onshape CAD assembly to URDF."""
    config_file_path = Path(config_path)
    if not config_file_path.exists():
        fallback_path = REPO_ROOT / config_path
        if fallback_path.exists():
            config_file_path = fallback_path
        else:
            raise FileNotFoundError(f"Config file {config_file_path} does not exist!")

    with open(config_file_path, "r") as f:
        robot_name = json.load(f)["output_filename"]

    urdf_dir = config_file_path.parent
    robot_dir = urdf_dir.parent
    scad_dir = robot_dir / "scad"

    if scad_dir.exists():
        assets_dir = urdf_dir / "assets"
        assets_dir.mkdir(exist_ok=True)
        for file in scad_dir.iterdir():
            shutil.copy(file, assets_dir / file.name)

    log.info("Ejecutando onshape-to-robot en %s...", urdf_dir)
    os.system(f"onshape-to-robot {urdf_dir}")

    if (urdf_dir / "assets" / "merged").exists():
        shutil.copytree(
            urdf_dir / "assets" / "merged",
            robot_dir / "meshes",
            dirs_exist_ok=True,
        )

    if (urdf_dir / "assets").exists():
        shutil.rmtree(urdf_dir / "assets")

    output_urdf = urdf_dir / f"{robot_name}.urdf"
    with open(output_urdf, "r") as file:
        content = file.read()

    content = content.replace("assets/merged/", "../meshes/")

    with open(output_urdf, "w") as file:
        file.write(content)

    log.info("URDF successfully generated at: %s", output_urdf)
    return output_urdf
