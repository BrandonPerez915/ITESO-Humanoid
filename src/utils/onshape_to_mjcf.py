"""Generate MJCF file from Onshape CAD project."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil

from common.logging import get_logger
from common.paths import REPO_ROOT

log = get_logger("assets.onshape_mjcf")

_SENSOR_BLOCK = """</actuator>

  <sensor>
    <jointpos name="arm_left_shoulder_pitch_pos"  joint="arm_left_shoulder_pitch_joint"/>
    <jointpos name="arm_left_shoulder_roll_pos"   joint="arm_left_shoulder_roll_joint"/>
    <jointpos name="arm_left_shoulder_yaw_pos"    joint="arm_left_shoulder_yaw_joint"/>
    <jointpos name="arm_left_elbow_pitch_pos"     joint="arm_left_elbow_pitch_joint"/>
    <jointpos name="arm_left_elbow_roll_pos"      joint="arm_left_elbow_roll_joint"/>
    <jointpos name="arm_right_shoulder_pitch_pos" joint="arm_right_shoulder_pitch_joint"/>
    <jointpos name="arm_right_shoulder_roll_pos"  joint="arm_right_shoulder_roll_joint"/>
    <jointpos name="arm_right_shoulder_yaw_pos"   joint="arm_right_shoulder_yaw_joint"/>
    <jointpos name="arm_right_elbow_pitch_pos"    joint="arm_right_elbow_pitch_joint"/>
    <jointpos name="arm_right_elbow_roll_pos"     joint="arm_right_elbow_roll_joint"/>
    <jointpos name="leg_left_hip_roll_pos"        joint="leg_left_hip_roll_joint"/>
    <jointpos name="leg_left_hip_yaw_pos"         joint="leg_left_hip_yaw_joint"/>
    <jointpos name="leg_left_hip_pitch_pos"       joint="leg_left_hip_pitch_joint"/>
    <jointpos name="leg_left_knee_pitch_pos"      joint="leg_left_knee_pitch_joint"/>
    <jointpos name="leg_left_ankle_pitch_pos"     joint="leg_left_ankle_pitch_joint"/>
    <jointpos name="leg_left_ankle_roll_pos"      joint="leg_left_ankle_roll_joint"/>
    <jointpos name="leg_right_hip_roll_pos"       joint="leg_right_hip_roll_joint"/>
    <jointpos name="leg_right_hip_yaw_pos"        joint="leg_right_hip_yaw_joint"/>
    <jointpos name="leg_right_hip_pitch_pos"      joint="leg_right_hip_pitch_joint"/>
    <jointpos name="leg_right_knee_pitch_pos"     joint="leg_right_knee_pitch_joint"/>
    <jointpos name="leg_right_ankle_pitch_pos"    joint="leg_right_ankle_pitch_joint"/>
    <jointpos name="leg_right_ankle_roll_pos"     joint="leg_right_ankle_roll_joint"/>

    <jointvel name="arm_left_shoulder_pitch_vel"  joint="arm_left_shoulder_pitch_joint"/>
    <jointvel name="arm_left_shoulder_roll_vel"   joint="arm_left_shoulder_roll_joint"/>
    <jointvel name="arm_left_shoulder_yaw_vel"    joint="arm_left_shoulder_yaw_joint"/>
    <jointvel name="arm_left_elbow_pitch_vel"     joint="arm_left_elbow_pitch_joint"/>
    <jointvel name="arm_left_elbow_roll_vel"      joint="arm_left_elbow_roll_joint"/>
    <jointvel name="arm_right_shoulder_pitch_vel" joint="arm_right_shoulder_pitch_joint"/>
    <jointvel name="arm_right_shoulder_roll_vel"  joint="arm_right_shoulder_roll_joint"/>
    <jointvel name="arm_right_shoulder_yaw_vel"   joint="arm_right_shoulder_yaw_joint"/>
    <jointvel name="arm_right_elbow_pitch_vel"    joint="arm_right_elbow_pitch_joint"/>
    <jointvel name="arm_right_elbow_roll_vel"     joint="arm_right_elbow_roll_joint"/>
    <jointvel name="leg_left_hip_roll_vel"        joint="leg_left_hip_roll_joint"/>
    <jointvel name="leg_left_hip_yaw_vel"         joint="leg_left_hip_yaw_joint"/>
    <jointvel name="leg_left_hip_pitch_vel"       joint="leg_left_hip_pitch_joint"/>
    <jointvel name="leg_left_knee_pitch_vel"      joint="leg_left_knee_pitch_joint"/>
    <jointvel name="leg_left_ankle_pitch_vel"     joint="leg_left_ankle_pitch_joint"/>
    <jointvel name="leg_left_ankle_roll_vel"      joint="leg_left_ankle_roll_joint"/>
    <jointvel name="leg_right_hip_roll_vel"       joint="leg_right_hip_roll_joint"/>
    <jointvel name="leg_right_hip_yaw_vel"        joint="leg_right_hip_yaw_joint"/>
    <jointvel name="leg_right_hip_pitch_vel"      joint="leg_right_hip_pitch_joint"/>
    <jointvel name="leg_right_knee_pitch_vel"     joint="leg_right_knee_pitch_joint"/>
    <jointvel name="leg_right_ankle_pitch_vel"    joint="leg_right_ankle_pitch_joint"/>
    <jointvel name="leg_right_ankle_roll_vel"     joint="leg_right_ankle_roll_joint"/>

    <jointactuatorfrc name="arm_left_shoulder_pitch_torque"   joint="arm_left_shoulder_pitch_joint"/>
    <jointactuatorfrc name="arm_left_shoulder_roll_torque"    joint="arm_left_shoulder_roll_joint"/>
    <jointactuatorfrc name="arm_left_shoulder_yaw_torque"     joint="arm_left_shoulder_yaw_joint"/>
    <jointactuatorfrc name="arm_left_elbow_pitch_torque"      joint="arm_left_elbow_pitch_joint"/>
    <jointactuatorfrc name="arm_left_elbow_roll_torque"       joint="arm_left_elbow_roll_joint"/>
    <jointactuatorfrc name="arm_right_shoulder_pitch_torque"  joint="arm_right_shoulder_pitch_joint"/>
    <jointactuatorfrc name="arm_right_shoulder_roll_torque"   joint="arm_right_shoulder_roll_joint"/>
    <jointactuatorfrc name="arm_right_shoulder_yaw_torque"    joint="arm_right_shoulder_yaw_joint"/>
    <jointactuatorfrc name="arm_right_elbow_pitch_torque"     joint="arm_right_elbow_pitch_joint"/>
    <jointactuatorfrc name="arm_right_elbow_roll_torque"      joint="arm_right_elbow_roll_joint"/>
    <jointactuatorfrc name="leg_left_hip_roll_torque"         joint="leg_left_hip_roll_joint"/>
    <jointactuatorfrc name="leg_left_hip_yaw_torque"          joint="leg_left_hip_yaw_joint"/>
    <jointactuatorfrc name="leg_left_hip_pitch_torque"        joint="leg_left_hip_pitch_joint"/>
    <jointactuatorfrc name="leg_left_knee_pitch_torque"       joint="leg_left_knee_pitch_joint"/>
    <jointactuatorfrc name="leg_left_ankle_pitch_torque"      joint="leg_left_ankle_pitch_joint"/>
    <jointactuatorfrc name="leg_left_ankle_roll_torque"       joint="leg_left_ankle_roll_joint"/>
    <jointactuatorfrc name="leg_right_hip_roll_torque"        joint="leg_right_hip_roll_joint"/>
    <jointactuatorfrc name="leg_right_hip_yaw_torque"         joint="leg_right_hip_yaw_joint"/>
    <jointactuatorfrc name="leg_right_hip_pitch_torque"       joint="leg_right_hip_pitch_joint"/>
    <jointactuatorfrc name="leg_right_knee_pitch_torque"      joint="leg_right_knee_pitch_joint"/>
    <jointactuatorfrc name="leg_right_ankle_pitch_torque"     joint="leg_right_ankle_pitch_joint"/>
    <jointactuatorfrc name="leg_right_ankle_roll_torque"      joint="leg_right_ankle_roll_joint"/>

    <framequat name="imu_quat" objtype="site" objname="imu" />
    <gyro name="imu_gyro" site="imu" />
    <accelerometer name="imu_acc" site="imu" />
    <framepos name="frame_pos" objtype="site" objname="imu" />
    <framelinvel name="frame_vel" objtype="site" objname="imu" />
  </sensor>"""


def export_onshape_to_mjcf(config_path: str = "assets/mjcf/config.json") -> Path:
    """Export an Onshape CAD assembly to MJCF XML."""
    config_file_path = Path(config_path)
    if not config_file_path.exists():
        fallback_path = REPO_ROOT / config_path
        if fallback_path.exists():
            config_file_path = fallback_path
        else:
            raise FileNotFoundError(f"Config file {config_file_path} does not exist!")

    with open(config_file_path, "r") as f:
        robot_name = json.load(f)["output_filename"]

    mjcf_dir = config_file_path.parent
    robot_dir = mjcf_dir.parent
    scad_dir = robot_dir / "scad"

    if scad_dir.exists():
        assets_dir = mjcf_dir / "assets"
        assets_dir.mkdir(exist_ok=True)
        for file in scad_dir.iterdir():
            shutil.copy(file, assets_dir / file.name)

    log.info("Ejecutando onshape-to-robot en %s...", mjcf_dir)
    os.system(f"onshape-to-robot {mjcf_dir}")

    if (mjcf_dir / "assets" / "merged").exists():
        shutil.copytree(
            mjcf_dir / "assets" / "merged",
            robot_dir / "meshes",
            dirs_exist_ok=True,
        )

    if (mjcf_dir / "assets").exists():
        shutil.rmtree(mjcf_dir / "assets")

    output_xml = mjcf_dir / f"{robot_name}.xml"
    with open(output_xml, "r") as file:
        content = file.read()

    content = content.replace("assets/merged/", "../meshes/")
    content = content.replace("</actuator>", _SENSOR_BLOCK)

    with open(output_xml, "w") as file:
        file.write(content)

    log.info("MJCF successfully generated at: %s", output_xml)
    return output_xml
