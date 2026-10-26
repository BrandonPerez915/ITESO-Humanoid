import sys
from pathlib import Path
import json
import time
from typing import Any, Dict

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from iteh.common.paths import REPO_ROOT, SRC_DIR, HARDWARE_CONFIG_DIR
from iteh.control import recoil
from iteh.control.robot import Humanoid

robot_configuration: Dict[str, Any] = {}

robot = Humanoid()

robot.check_connection()

for entry in robot.joints:
    bus, joint_id, joint_name = entry

    print(f"Reading configuration for {joint_name}")

    config: Dict[str, Any] = {
        "position_controller": {},
        "current_controller": {},
        "powerstage": {},
        "motor": {},
        "encoder": {},
    }

    config["device_id"] = bus._read_parameter_u32(joint_id, recoil.Parameter.DEVICE_ID)
    fw_val = bus._read_parameter_u32(joint_id, recoil.Parameter.FIRMWARE_VERSION)
    config["firmware_version"] = hex(fw_val) if fw_val is not None else None
    config["watchdog_timeout"] = bus._read_parameter_u32(joint_id, recoil.Parameter.WATCHDOG_TIMEOUT)
    config["fast_frame_frequency"] = bus.read_fast_frame_frequency(joint_id)

    config["position_controller"]["gear_ratio"] = bus._read_parameter_f32(joint_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO)
    config["position_controller"]["position_kp"] = bus.read_position_kp(joint_id)
    config["position_controller"]["position_ki"] = bus.read_position_ki(joint_id)
    config["position_controller"]["velocity_kp"] = bus.read_velocity_kp(joint_id)
    config["position_controller"]["velocity_ki"] = bus.read_velocity_ki(joint_id)
    config["position_controller"]["torque_limit"] = bus.read_torque_limit(joint_id)
    config["position_controller"]["velocity_limit"] = bus.read_velocity_limit(joint_id)
    config["position_controller"]["position_limit_upper"] = bus.read_position_limit_upper(joint_id)
    config["position_controller"]["position_limit_lower"] = bus.read_position_limit_lower(joint_id)
    config["position_controller"]["position_offset"] = bus.read_position_offset(joint_id)
    config["position_controller"]["torque_filter_alpha"] = bus.read_torque_filter_alpha(joint_id)

    config["current_controller"]["current_kp"] = bus.read_current_kp(joint_id)
    config["current_controller"]["current_ki"] = bus.read_current_ki(joint_id)
    config["current_controller"]["current_limit"] = bus.read_current_limit(joint_id)

    config["powerstage"]["bus_voltage_filter_alpha"] = bus.read_bus_voltage_filter_alpha(joint_id)

    config["motor"]["pole_pairs"] = bus.read_motor_pole_pairs(joint_id)
    config["motor"]["torque_constant"] = bus.read_motor_torque_constant(joint_id)
    config["motor"]["phase_order"] = bus.read_motor_phase_order(joint_id)
    config["motor"]["calibration_current"] = bus.read_motor_calibration_current(joint_id)

    config["encoder"]["cpr"] = bus.read_encoder_cpr(joint_id)
    config["encoder"]["position_offset"] = bus.read_encoder_position_offset(joint_id)
    config["encoder"]["velocity_filter_alpha"] = bus.read_encoder_velocity_filter_alpha(joint_id)
    config["encoder"]["flux_offset"] = bus.read_encoder_flux_offset(joint_id)

    robot_configuration[joint_name] = config

out_path = HARDWARE_CONFIG_DIR / "motor_configuration.json"
print(f"Writing configuration to {out_path}")
with open(out_path, "w") as f:
    json.dump(robot_configuration, f, indent=4)

robot.stop()
