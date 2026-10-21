import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import numpy as np
from loop_rate_limiters import RateLimiter

try:
    from bhl.control.robot.bimanual import Bimanual
except ImportError:
    from src.control.robot.bimanual import Bimanual


np.set_printoptions(precision=3, suppress=True)

rate = RateLimiter(100)

robot = Bimanual()

robot.run(kp=20, kd=2, torque_limit=0.5)

try:
    while True:
        obs = robot.step(np.zeros((10,), dtype=np.float32))

        print(obs)

        rate.sleep()

except KeyboardInterrupt:
    print("Stopping robot due to keyboard interrupt.")
finally:
    robot.stop()
