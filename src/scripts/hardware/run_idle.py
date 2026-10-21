import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import numpy as np
from cc.udp import UDP

try:
    from bhl.control.robot import Humanoid
except ImportError:
    try:
        from src.control.robot import Humanoid
    except ImportError:
        from control.robot import Humanoid


robot = Humanoid()
# udp = UDP(send_addr=("172.28.0.5", 8000))

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
