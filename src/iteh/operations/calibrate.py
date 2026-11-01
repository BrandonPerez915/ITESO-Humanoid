"""Calibration operations for joint encoder offsets and actuator electrical offsets."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Optional
import numpy as np
import yaml

from common.logging import get_logger, status_line
from common.paths import REPO_ROOT
import iteh.communication.core as Can
from iteh.communication.flags import ModeFlag

log = get_logger("ops.calibrate")


def calibrate_joints(output_path: Optional[str] = None) -> Path:
    """Calibrate encoder position offsets of each leg joint after power cycle."""
    from iteh import Humanoid

    robot = Humanoid()

    joint_axis_directions = np.array([-1, +1, -1, -1, -1, +1, -1, +1, +1, +1, +1, +1])
    ideal_values = np.array(
        [
            np.deg2rad(-10),
            np.deg2rad(+33.75),
            np.deg2rad(+56.25),
            np.deg2rad(0),
            np.deg2rad(-45),
            np.deg2rad(-15),
            np.deg2rad(+10),
            np.deg2rad(-33.75),
            np.deg2rad(+56.25),
            np.deg2rad(0),
            np.deg2rad(-45),
            np.deg2rad(+15),
        ]
    )

    limit_readings = (
        np.array([joint[0].read_position_measured(joint[1]) for joint in robot.joints]) * joint_axis_directions
    )
    log.info("Initial readings: %s", [f"{reading:.2f}" for reading in limit_readings])
    log.info("Move joints to their mechanical hard stops and press Mode 1 (IDLE) to save...")

    cal_path = Path(output_path) if output_path else (REPO_ROOT / "configs" / "hardware" / "calibration.yaml")
    try:
        while robot.command_controller.commands.get("mode_switch") != 1:
            joint_readings = (
                np.array([joint[0].read_position_measured(joint[1]) for joint in robot.joints])
                * joint_axis_directions
            )

            limit_readings[0] = min(limit_readings[0], joint_readings[0])
            limit_readings[1] = max(limit_readings[1], joint_readings[1])
            limit_readings[2] = max(limit_readings[2], joint_readings[2])
            limit_readings[3] = min(limit_readings[3], joint_readings[3])
            limit_readings[4] = min(limit_readings[4], joint_readings[4])
            limit_readings[5] = min(limit_readings[5], joint_readings[5])

            limit_readings[6] = max(limit_readings[6], joint_readings[6])
            limit_readings[7] = min(limit_readings[7], joint_readings[7])
            limit_readings[8] = max(limit_readings[8], joint_readings[8])
            limit_readings[9] = min(limit_readings[9], joint_readings[9])
            limit_readings[10] = min(limit_readings[10], joint_readings[10])
            limit_readings[11] = max(limit_readings[11], joint_readings[11])

            status_line("Current hard-stop limits: " + " ".join(f"{r:+.2f}" for r in limit_readings))
            time.sleep(0.05)

        offsets = limit_readings - ideal_values
        log.info("Final hard-stop readings: %s", [f"{limit:.4f}" for limit in limit_readings])
        log.info("Calculated offsets: %s", [f"{offset:.4f}" for offset in offsets])

        calibration_data = {
            "position_offsets": [float(offset) for offset in offsets],
        }

        cal_path.parent.mkdir(parents=True, exist_ok=True)
        with open(cal_path, "w") as f:
            yaml.dump(calibration_data, f)
        log.info("Calibration saved to: %s", cal_path)

    except KeyboardInterrupt:
        log.info("Calibration interrupted by user.")
    finally:
        robot.stop()

    return cal_path


def calibrate_electrical_offset(channel: str = "can0", device_id: int = 1, wait_sec: float = 20.0) -> None:
    """Trigger the motor electrical offset calibration sequence on a CAN actuator."""
    log.info("Starting electrical calibration on channel=%s, device_id=%d...", channel, device_id)
    bus = Can.Bus(channel=channel, bitrate=1000000)
    try:
        bus.set_mode(device_id, ModeFlag.CALIBRATION)
        log.info("Waiting %.1f seconds for actuator calibration sequence to finish...", wait_sec)
        time.sleep(wait_sec)
        log.info("Electrical calibration finished.")
    finally:
        bus.stop()
