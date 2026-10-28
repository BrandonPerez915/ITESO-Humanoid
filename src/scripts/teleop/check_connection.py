from src.iteh.robot.bimanual import Bimanual

robot = Bimanual()

try:
    while True:
        robot.check_connection()
except KeyboardInterrupt:
    print("Stopping robot due to keyboard interrupt.")

robot.stop()
