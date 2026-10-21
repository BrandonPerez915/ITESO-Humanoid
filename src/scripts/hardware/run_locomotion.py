import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from cc.udp import UDP
from loop_rate_limiters import RateLimiter

try:
    from bhl.control.robot import Humanoid
    from bhl.control.policy import RlController, Cfg
except ImportError:
    try:
        from src.control.robot import Humanoid
        from src.control.policy.rl_controller import RlController
        from src.control.policy.config import Cfg
    except ImportError:
        from control.robot import Humanoid
        from control.policy.rl_controller import RlController
        from control.policy.config import Cfg


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
