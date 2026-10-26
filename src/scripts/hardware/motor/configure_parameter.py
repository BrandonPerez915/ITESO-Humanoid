import sys
from pathlib import Path

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


def configure_fast_frame_rate(bus: recoil.Bus, device_id: int, fast_frame_rate: int):
    print("Rate (before):\t", bus._read_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY))
    bus._write_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY, fast_frame_rate)
    print("Rate (updated):\t", bus._read_parameter_u32(device_id, recoil.Parameter.FAST_FRAME_FREQUENCY))


def configure_gear_ratio(bus: recoil.Bus, device_id: int, gear_ratio: float):
    print("Gear Ratio (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO, -gear_ratio)
    print("Gear Ratio (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO))


def configure_position_pd(bus: recoil.Bus, device_id: int, kp: float, kd: float):
    print("Kp (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP, kp)
    print("Kp (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KP))

    print("Kd (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KD))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KD, kd)
    print("Kd (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_KD))


def configure_torque_limit(bus: recoil.Bus, device_id: int, torque_limit: float):
    print("Torque Limit (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT, torque_limit)
    print("Torque Limit (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT))


def configure_velocity_limit(bus: recoil.Bus, device_id: int, velocity_limit: float):
    print("Velocity Limit (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT, velocity_limit)
    print("Velocity Limit (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT))


def configure_position_limit(bus: recoil.Bus, device_id: int, position_limit_lower: float, position_limit_upper: float):
    print("Lower Limit (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, position_limit_lower)
    print("Lower Limit (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER))

    print("Upper Limit (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, position_limit_upper)
    print("Upper Limit (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER))


def configure_torque_filter_alpha(bus: recoil.Bus, device_id: int, alpha: float):
    print("Filter Alpha (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA))
    bus._write_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA, alpha)
    print("Filter Alpha (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA))


def configure_current_limit(bus: recoil.Bus, device_id: int, current_limit: float):
    print("Current Limit (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_CURRENT_LIMIT))
    bus._write_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_CURRENT_LIMIT, current_limit)
    print("Current Limit (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.CURRENT_CONTROLLER_CURRENT_LIMIT))


def configure_current_bandwidth(bus: recoil.Bus, device_id: int, bandwidth: float):
    r = bus._read_parameter_f32(device_id, recoil.Parameter.MOTOR_PHASE_RESISTANCE)
    l = bus._read_parameter_f32(device_id, recoil.Parameter.MOTOR_PHASE_INDUCTANCE)
    if r is not None and l is not None:
        bus.set_current_bandwidth(device_id, bandwidth, r, l)
    print("Current Bandwidth updated")


def configure_torque_bandwidth(bus: recoil.Bus, device_id: int, bandwidth: float):
    bus.set_torque_bandwidth(device_id, bandwidth)
    print("Torque Bandwidth updated")


def configure_velocity_filter_bandwidth(bus: recoil.Bus, device_id: int, bandwidth: float):
    bus.set_encoder_velocity_bandwidth(device_id, bandwidth)
    print("Velocity Filter Bandwidth updated")


def configure_motor_pole_pairs(bus: recoil.Bus, device_id: int, pole_pairs: int):
    print("Pole Pairs (before):\t", bus._read_parameter_u32(device_id, recoil.Parameter.MOTOR_POLE_PAIRS))
    bus._write_parameter_u32(device_id, recoil.Parameter.MOTOR_POLE_PAIRS, pole_pairs)
    print("Pole Pairs (updated):\t", bus._read_parameter_u32(device_id, recoil.Parameter.MOTOR_POLE_PAIRS))


def configure_motor_torque_constant(bus: recoil.Bus, device_id: int, torque_constant: float):
    print("Torque Constant (before):\t", bus._read_parameter_f32(device_id, recoil.Parameter.MOTOR_TORQUE_CONSTANT))
    bus._write_parameter_f32(device_id, recoil.Parameter.MOTOR_TORQUE_CONSTANT, torque_constant)
    print("Torque Constant (updated):\t", bus._read_parameter_f32(device_id, recoil.Parameter.MOTOR_TORQUE_CONSTANT))


bus.stop()
