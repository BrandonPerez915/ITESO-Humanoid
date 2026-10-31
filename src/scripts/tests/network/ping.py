import sys

import src.iteh.communication.core as Can

def main(channel: str, device_id: int) -> None:
    bus = Can.Bus(channel=channel, bitrate=1000000)
    status = bus.ping(device_id)

    if status:
        print("Motor is online")
    else:
        print("Motor is offline")

    bus.stop()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python ping.py <channel> <device_id>")
        sys.exit(1)

    channel = sys.argv[1]
    device_id = int(sys.argv[2])
    main(channel, device_id)
