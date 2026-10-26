import sys
from pathlib import Path
import time
import numpy as np

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
from loop_rate_limiters import RateLimiter

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

bus.feed(device_id)
bus.set_mode(device_id, recoil.Mode.POSITION)

time_start = time.time()

try:
    while True:
        t = time.time() - time_start
        target_position = np.sin(2 * np.pi * frequency * t) * amplitude
        target_velocity = np.cos(2 * np.pi * frequency * t) * amplitude * 2 * np.pi * frequency

        bus.write_position_target(device_id, target_position)
        bus.write_velocity_target(device_id, target_velocity)

        rate.sleep()

except KeyboardInterrupt:
    print("\n[INFO] Movimiento interrumpido.")
finally:
    bus.set_mode(device_id, recoil.Mode.IDLE)
    bus.stop()
