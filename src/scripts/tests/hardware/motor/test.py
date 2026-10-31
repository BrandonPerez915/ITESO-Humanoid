import time
import numpy as np
import sys
from loop_rate_limiters import RateLimiter

import src.iteh.communication.core as Can
from src.iteh.communication.flags import ModeFlag


def main(channel: str, device_id: int):
    KP = 0.2
    KD = 0.005
    FREQUENCY = 1.0  # motion frequency is 1 Hz
    AMPLITUDE = 1.0  # motion amplitude is 1 rad

    rate = RateLimiter(frequency=200.0)
    bus = Can.Bus(channel=channel, bitrate=1000000)
    # Set the motor to position control mode
    bus.set_mode(device_id, ModeFlag.POSITION)
    bus.feed(device_id)

    bus.write_position_kp(device_id, KP)
    bus.write_position_kd(device_id, KD)
    bus.write_torque_limit(device_id, 0.2)

    try:
        while True:
            target_angle = np.sin(2 * np.pi * FREQUENCY * time.time()) * AMPLITUDE

            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
            if measured_position is not None and measured_velocity is not None:
                print(f"Measured pos: {measured_position:.3f} \tvel: {measured_velocity:.3f}")

            rate.sleep()

    except KeyboardInterrupt:
        pass

    bus.set_mode(device_id, ModeFlag.IDLE)
    bus.stop()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python move_actuator.py <channel> <device_id>")
        sys.exit(1)

    channel = sys.argv[1]
    device_id = int(sys.argv[2])
    main(channel, device_id)
