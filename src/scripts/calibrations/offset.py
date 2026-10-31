import sys
import time

from src.iteh import Can
from src.iteh import ModeFlag


def main(channel: str, device_id: int):
    bus = Can.Bus(channel=channel, bitrate=1000000)
    bus.set_mode(device_id, ModeFlag.CALIBRATION)

    # The motor should now perform the calibration sequence
    # wait for calibration to finish
    time.sleep(20)
    bus.stop()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python electrical_offset.py <channel> <device_id>")
        sys.exit(1)

    channel = sys.argv[1]
    device_id = int(sys.argv[2])
    main(channel, device_id)
