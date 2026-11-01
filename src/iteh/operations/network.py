"""Network and CAN bus management operations."""

from __future__ import annotations

import subprocess
from typing import Sequence

from common.logging import get_logger
import iteh.communication.core as Can

log = get_logger("ops.network")


def ping_motor(channel: str = "can0", device_id: int = 1) -> bool:
    """Ping a single motor actuator over CAN."""
    bus = Can.Bus(channel=channel, bitrate=1000000)
    try:
        status = bool(bus.ping(device_id))
        if status:
            log.info("Motor #%d on %s is ONLINE.", device_id, channel)
        else:
            log.warning("Motor #%d on %s is OFFLINE.", device_id, channel)
        return status
    finally:
        bus.stop()


def check_connection(humanoid: bool = False) -> None:
    """Continuously ping all joints of the Humanoid (legs) or Bimanual (arms) robot."""
    from iteh.bimanual import Bimanual
    from iteh.humanoid import Humanoid

    robot = Humanoid() if humanoid else Bimanual()
    log.info("Checking CAN connection continuously (%s)...", "Humanoid" if humanoid else "Bimanual")
    try:
        while True:
            robot.check_connection()
    except KeyboardInterrupt:
        log.info("Connection check stopped by user.")
    finally:
        robot.stop()


def setup_can_interfaces(
    up: bool = True,
    channels: Sequence[str] = ("can0", "can1", "can2", "can3"),
    bitrate: int = 1000000,
) -> int:
    """Bring SocketCAN interfaces up or down via `ip link`."""
    exit_code = 0
    for ch in channels:
        if up:
            cmd = ["sudo", "ip", "link", "set", ch, "up", "type", "can", "bitrate", str(bitrate)]
        else:
            cmd = ["sudo", "ip", "link", "set", ch, "down"]
        log.info("Executing: %s", " ".join(cmd))
        res = subprocess.run(cmd, check=False)
        if res.returncode != 0:
            log.error("Failed to configure interface %s (code %d)", ch, res.returncode)
            exit_code = res.returncode
    return exit_code
