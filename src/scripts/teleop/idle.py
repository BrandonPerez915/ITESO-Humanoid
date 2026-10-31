import numpy as np
from loop_rate_limiters import RateLimiter

from iteh.bimanual import Bimanual

np.set_printoptions(precision=3, suppress=True)

rate = RateLimiter(100)
robot = Bimanual()

robot.start(kp=20, kd=2, torque_limit=0.5) # type: ignore

try:
    while True:
        obs = robot.step(np.zeros((12,), dtype=np.float32))
        print(obs)
        rate.sleep()

except KeyboardInterrupt:
    print("Stopping robot due to keyboard interrupt.")
finally:
    robot.stop()
