"""Post-process exported URDF and MJCF files."""

from __future__ import annotations

from pathlib import Path

from common.logging import get_logger
from common.paths import REPO_ROOT

log = get_logger("assets.postprocess")


def postprocess_urdf(name: str = "berkeley-humanoid-lite") -> Path:
    urdf_path = Path(f"data/urdf/{name}.urdf")
    if not urdf_path.exists():
        urdf_path = REPO_ROOT / "assets" / "urdf" / f"{name}.urdf"
    with open(urdf_path, "r") as f:
        content = f.read()

    content = content.replace("package://", "./")

    with open(urdf_path, "w") as f:
        f.write(content)

    log.info("URDF post-processing completed: %s", urdf_path)
    return urdf_path


def postprocess_mjcf(name: str = "berkeley-humanoid-lite") -> Path:
    mjcf_path = Path(f"data/mjcf/{name}.xml")
    if not mjcf_path.exists():
        mjcf_path = REPO_ROOT / "assets" / "mjcf" / f"{name}.xml"
    with open(mjcf_path, "r") as f:
        content = f.read()

    sensor_content = """
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
  </sensor>
"""
    content = content.replace("<equality/>", sensor_content + "<equality/>")

    with open(mjcf_path, "w") as f:
        f.write(content)

    log.info("MJCF post-processing completed: %s", mjcf_path)
    return mjcf_path
