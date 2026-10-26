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

from iteh.common.paths import REPO_ROOT, SRC_DIR
from iteh.control import recoil

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

print(f"Reading configuration from actuator #{args.id}")

config: Dict[str, Any] = {
    "position_controller": {},
    "current_controller": {},
    "powerstage": {},
    "motor": {},
    "encoder": {},
}

config["device_id"] = bus._read_parameter_u32(device_id, recoil.Parameter.DEVICE_ID)
fw_val = bus._read_parameter_u32(device_id, recoil.Parameter.FIRMWARE_VERSION)
config["firmware_version"] = hex(fw_val) if fw_val is not None else None
config["watchdog_timeout"] = bus._read_parameter_u32(device_id, recoil.Parameter.WATCHDOG_TIMEOUT)
config["fast_frame_frequency"] = bus.read_fast_frame_frequency(device_id)

config["position_controller"]["gear_ratio"] = bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO)
config["position_controller"]["position_kp"] = bus.read_position_kp(device_id)
config["position_controller"]["position_ki"] = bus.read_position_ki(device_id)
config["position_controller"]["velocity_kp"] = bus.read_velocity_kp(device_id)
config["position_controller"]["velocity_ki"] = bus.read_velocity_ki(device_id)
config["position_controller"]["torque_limit"] = bus.read_torque_limit(device_id)
config["position_controller"]["velocity_limit"] = bus.read_velocity_limit(device_id)
config["position_controller"]["position_limit_upper"] = bus.read_position_limit_upper(device_id)
config["position_controller"]["position_limit_lower"] = bus.read_position_limit_lower(device_id)
config["position_controller"]["position_offset"] = bus.read_position_offset(device_id)
config["position_controller"]["torque_filter_alpha"] = bus.read_torque_filter_alpha(device_id)

config["current_controller"]["current_kp"] = bus.read_current_kp(device_id)
config["current_controller"]["current_ki"] = bus.read_current_ki(device_id)
config["current_controller"]["current_limit"] = bus.read_current_limit(device_id)

config["powerstage"]["bus_voltage_filter_alpha"] = bus.read_bus_voltage_filter_alpha(device_id)

config["motor"]["pole_pairs"] = bus.read_motor_pole_pairs(device_id)
config["motor"]["torque_constant"] = bus.read_motor_torque_constant(device_id)
config["motor"]["phase_order"] = bus.read_motor_phase_order(device_id)
config["motor"]["calibration_current"] = bus.read_motor_calibration_current(device_id)

config["encoder"]["cpr"] = bus.read_encoder_cpr(device_id)
config["encoder"]["position_offset"] = bus.read_encoder_position_offset(device_id)
config["encoder"]["velocity_filter_alpha"] = bus.read_encoder_velocity_filter_alpha(device_id)
config["encoder"]["flux_offset"] = bus.read_encoder_flux_offset(device_id)

print(json.dumps(config, indent=4))

# write to file
with open("motor_configuration.json", "w") as f:
    json.dump(config, f, indent=4)

bus.stop()
