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

from iteh.common.paths import REPO_ROOT, SRC_DIR, get_config_path
from iteh.control.robot import Humanoid
from iteh.control.policy import RlController, Cfg

from cc.udp import UDP
from loop_rate_limiters import RateLimiter

# Load configuration
cfg = Cfg.from_arguments()

print(f"Policy frequency: {1 / cfg.policy_dt} Hz")

udp = UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))

# Initialize and start policy controller
controller = RlController(cfg)
controller.load_policy()

rate = RateLimiter(1 / cfg.policy_dt)

robot = Humanoid()
robot.enter_damping()

obs = robot.reset()

try:
    while True:
        actions = controller.update(obs)
        obs = robot.step(actions)
        udp.send_numpy(obs)

        rate.sleep()

except KeyboardInterrupt:
    print("\n[INFO] Locomoción detenida por el usuario.")
finally:
    robot.stop()
    print("Robot detenido.")
