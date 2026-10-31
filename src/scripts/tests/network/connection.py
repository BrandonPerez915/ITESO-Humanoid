import sys

from iteh.bimanual import Bimanual
from iteh.humanoid import Humanoid

def main(humanoid: bool = False):
    if humanoid:
        robot = Humanoid()
    else:
        robot = Bimanual()

    try:
        while True:
            robot.check_connection()
    except KeyboardInterrupt:
        print("Stopping robot due to keyboard interrupt.")

    robot.stop()

if __name__ == "__main__":
    humanoid_flag = "--humanoid" in sys.argv
    main(humanoid=humanoid_flag)
