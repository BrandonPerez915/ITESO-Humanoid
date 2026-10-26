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
from iteh.control.robot.bimanual import Bimanual

import numpy as np
from loop_rate_limiters import RateLimiter

np.set_printoptions(precision=3, suppress=True)

rate = RateLimiter(100)

robot = Bimanual()

robot.start(kp=20, kd=2, torque_limit=0.5)

try:
    while True:
        obs = robot.step(np.zeros((10,), dtype=np.float32))

        print(obs)

        rate.sleep()

except KeyboardInterrupt:
    print("Stopping robot due to keyboard interrupt.")
finally:
    robot.stop()
