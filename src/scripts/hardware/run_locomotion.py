import sys
from pathlib import Path
from cc.udp import UDP
from loop_rate_limiters import RateLimiter

from src.iteh.robot.humanoid import Humanoid
from src.iteh.policy import RlController, PolicyConfig

# Load configuration
cfg = PolicyConfig.from_arguments()

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
