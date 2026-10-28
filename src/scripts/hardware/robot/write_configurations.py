import sys
from pathlib import Path
import json
import time
import os

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[4]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from common.paths import get_config_path
from iteh.control import recoil
from src.iteh.robot.humanoid import Humanoid

_config_file = str(get_config_path("motor.json"))
if not os.path.exists(_config_file):
    for _candidate in [
        _repo_root / "configs" / "hardware" / "motor.json",
        _repo_root / "motor_configuration.json",
        _repo_root / "robot_configuration.json",
        _repo_root / "configs" / "robot_configuration.json",
    ]:
        if os.path.exists(str(_candidate)):
            _config_file = str(_candidate)
            break

if not os.path.exists(_config_file):
    raise FileNotFoundError(f"No se encontró archivo de configuración de motores en {_repo_root}.")

robot_configuration = json.load(open(_config_file))

robot = Humanoid()

delay_t = 0.1

store_to_flash = True


for entry in robot.joints:
    bus, joint_id, joint_name = entry
    print(f"Pinging {joint_name} ... ", end="\t")
    result = bus.ping(joint_id)
    print(f"success: {result}")

    firmware_version = hex(bus._read_parameter_u32(joint_id, recoil.Parameter.FIRMWARE_VERSION)) # type: ignore
    print(f"  firmware version: {firmware_version}")

    time.sleep(0.1)


for entry in robot.joints:
    bus, joint_id, joint_name = entry

    print(f"Writing configuration for {joint_name}")

    config = robot_configuration.get(joint_name)
    if not config:
        raise ValueError(f"No configuration found for {joint_name}")

    val = config["fast_frame_frequency"]
    print(f" setting fast frame frequency to {val}")
    bus.write_fast_frame_frequency(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["gear_ratio"]
    print(f" setting gear ratio to {val}")
    bus._write_parameter_f32(joint_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, val)
    time.sleep(delay_t)

    val = config["position_controller"]["position_kp"]
    print(f" setting KP to {val}")
    bus.write_position_kp(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["position_ki"]
    print(f" setting KI to {val}")
    bus.write_position_ki(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["velocity_kp"]
    print(f" setting KD to {val}")
    bus.write_velocity_kp(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["velocity_ki"]
    print(f" setting velocity KI to {val}")
    bus.write_velocity_ki(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["torque_limit"]
    print(f" setting torque limit to {val}")
    bus.write_torque_limit(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["velocity_limit"]
    print(f" setting velocity limit to {val}")
    bus.write_velocity_limit(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["position_limit_lower"]
    print(f" setting position limit lower to {val}")
    bus.write_position_limit_lower(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["position_limit_upper"]
    print(f" setting position limit upper to {val}")
    bus.write_position_limit_upper(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["position_offset"]
    print(f" setting position offset to {val}")
    bus.write_position_offset(joint_id, val)
    time.sleep(delay_t)

    val = config["position_controller"]["torque_filter_alpha"]
    print(f" setting torque filter alpha to {val}")
    bus.write_torque_filter_alpha(joint_id, val)
    time.sleep(delay_t)

    val = config["current_controller"]["i_limit"]
    print(f" setting current limit to {val}")
    bus.write_current_limit(joint_id, val)
    time.sleep(delay_t)

    val = config["current_controller"]["i_kp"]
    print(f" setting current Kp to {val}")
    bus.write_current_kp(joint_id, val)
    time.sleep(delay_t)

    val = config["current_controller"]["i_ki"]
    print(f" setting current Ki to {val}")
    bus.write_current_ki(joint_id, val)
    time.sleep(delay_t)

    val = config["motor"]["pole_pairs"]
    print(f" setting pole pairs to {val}")
    bus.write_motor_pole_pairs(joint_id, val)
    time.sleep(delay_t)

    val = config["motor"]["torque_constant"]
    print(f" setting torque constant to {val}")
    bus.write_motor_torque_constant(joint_id, val)
    time.sleep(delay_t)

    val = config["motor"]["phase_order"]
    print(f" setting phase order to {val}")
    bus.write_motor_phase_order(joint_id, val)
    time.sleep(delay_t)

    val = config["motor"]["max_calibration_current"]
    print(f" setting max calibration current to {val}")
    bus.write_motor_calibration_current(joint_id, val)
    time.sleep(delay_t)

    val = config["encoder"]["cpr"]
    print(f" setting cpr to {val}")
    bus.write_encoder_cpr(joint_id, val)
    time.sleep(delay_t)

    val = config["encoder"]["position_offset"]
    print(f" setting position offset to {val}")
    bus.write_encoder_position_offset(joint_id, val)
    time.sleep(delay_t)

    val = config["encoder"]["velocity_filter_alpha"]
    print(f" setting velocity filter alpha to {val}")
    bus.write_encoder_velocity_filter_alpha(joint_id, val)
    time.sleep(delay_t)

    val = config["encoder"]["flux_offset"]
    print(f" setting flux offset to {val}")
    bus.write_encoder_flux_offset(joint_id, val)
    time.sleep(delay_t)

    if store_to_flash:
        print(" storing to flash")
        bus.store_settings_to_flash(joint_id)
        time.sleep(0.2)

    time.sleep(0.5)


robot.stop()
