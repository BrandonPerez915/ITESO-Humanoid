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
from iteh.control.robot import Humanoid

import numpy as np
from cc.udp import UDP

robot = Humanoid()
udp = UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))

try:
    while True:
        acs = np.zeros(12)
        obs = robot.step(acs)
        udp.send_numpy(obs)

        print(robot.joint_position_measured)

except KeyboardInterrupt:
    print("\n[INFO] Detenido por el usuario.")
finally:
    robot.stop()
    print("Robot detenido.")
