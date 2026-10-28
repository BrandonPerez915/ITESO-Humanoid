import sys
from pathlib import Path
import time
import numpy as np

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[4]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loop_rate_limiters import RateLimiter

from iteh.control import recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

kp = 0.2
kd = 0.005

frequency = 1.0  # motion frequency is 1 Hz
amplitude = 1.0  # motion amplitude is 1 rad

rate = RateLimiter(frequency=200.0)


bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.2)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

try:
    while True:
        target_angle = np.sin(2 * np.pi * frequency * time.time()) * amplitude

        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(f"Measured pos: {measured_position:.3f} \tvel: {measured_velocity:.3f}")

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()
