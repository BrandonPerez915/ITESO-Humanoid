import numpy as np
from cc.udp import UDP

from src.iteh import Humanoid

def main():
    robot = Humanoid()
    udp = UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))

    try:
        while True:
            acs = np.zeros(12)
            obs = robot.step(acs)
            udp.send_numpy(obs)

            print(robot.joint_position_measured)
    except KeyboardInterrupt:
        print("Stopping robot due to keyboard interrupt.")

    robot.stop()

if __name__ == "__main__":
    main()
