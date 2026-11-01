"""Hardware component diagnostics and single-motor configuration operations."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import time
from typing import Optional
import numpy as np

from common.logging import get_logger, status_line
import iteh.communication.core as Can
from iteh.communication.flags import ModeFlag, ParameterFlag
from iteh.operations.robot import _read_actuator_dict

log = get_logger("ops.hardware")


def test_imu(
    port: str = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.1:1.0",
    baudrate: int = 1000000,
) -> None:
    """Read and display real-time Euler angles from the BNO085 / serial IMU."""
    import scipy.spatial.transform as st
    import serial

    sync_1 = b"\x75"
    sync_2 = b"\x65"

    log.info("Opening IMU serial port: %s @ %d bps...", port, baudrate)
    ser = serial.Serial(port, baudrate, timeout=0.001)

    try:
        while True:
            t = time.perf_counter()
            b1 = ser.read(1)
            if not b1 or b1 != sync_1:
                continue
            b2 = ser.read(1)
            if not b2 or b2 != sync_2:
                continue

            _ = ser.read(2)
            data_buffer = ser.read(28)
            if len(data_buffer) < 28:
                continue

            w, x, y, z, _, _, _ = struct.unpack("f" * 7, data_buffer)
            euler = st.Rotation.from_quat([w, x, y, z], scalar_first=True).as_euler("xyz", degrees=True)
            freq = 1.0 / max(time.perf_counter() - t, 1e-6)
            status_line(f"Roll: {euler[0]:+.2f} deg | Pitch: {euler[1]:+.2f} deg | Yaw: {euler[2]:+.2f} deg | {freq:.1f} Hz")
    except KeyboardInterrupt:
        log.info("IMU test stopped by user.")
    finally:
        ser.close()


def test_joystick() -> None:
    """Run gamepad input controller and display live command values."""
    from iteh.inputs.gamepad import Se2Gamepad

    gamepad = Se2Gamepad()
    gamepad.run()
    log.info("Testing Gamepad controller (Ctrl+C to exit)...")

    try:
        while True:
            status_line(
                f"Mode: {gamepad.commands['mode_switch']} | "
                f"Vx: {gamepad.commands['velocity_x']:+.2f} | "
                f"Vy: {gamepad.commands['velocity_y']:+.2f} | "
                f"Vyaw: {gamepad.commands['velocity_yaw']:+.2f}"
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        log.info("Stopping Gamepad test.")
    finally:
        gamepad.stop()


def test_gripper(
    port: str = "/dev/ttyUSB0",
    baudrate: int = 115200,
    left_target: float = 0.8,
    right_target: float = 0.8,
) -> None:
    """Send target closure commands to serial grippers and read feedback."""
    import serial

    log.info("Opening gripper serial port: %s @ %d bps...", port, baudrate)
    ser = serial.Serial(port, baudrate, timeout=0.5)
    try:
        while True:
            data = struct.pack("<ffb", float(left_target), float(right_target), 0x0C)
            ser.write(data)
            line = ser.readline()
            if line:
                log.info("Gripper feedback: %s", line.decode("utf-8", errors="replace").strip())
    except KeyboardInterrupt:
        log.info("Gripper test stopped.")
    finally:
        ser.close()


def read_motor_config(
    channel: str = "can0",
    device_id: int = 1,
    output_path: str = "motor_configuration.json",
) -> Path:
    """Read configuration from a single actuator over CAN and save to JSON."""
    bus = Can.Bus(channel=channel, bitrate=1000000)
    try:
        log.info("Reading configuration for actuator #%d on %s...", device_id, channel)
        config = _read_actuator_dict(bus, device_id)
        time.sleep(0.1)

        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w") as f:
            json.dump(config, f, indent=4)
        log.info("Actuator configuration saved to: %s", out)
        return out
    finally:
        bus.stop()


def configure_motor_parameter(
    channel: str = "can0",
    device_id: int = 1,
    fast_frame_rate: Optional[int] = None,
    gear_ratio: Optional[float] = None,
    kp: Optional[float] = None,
    kd: Optional[float] = None,
    torque_limit: Optional[float] = None,
    phase_order: Optional[int] = None,
    lower_limit: Optional[float] = None,
    upper_limit: Optional[float] = None,
    bandwidth: Optional[float] = None,
    flash: bool = False,
) -> None:
    """Configure specific parameters on a single CAN motor actuator."""
    bus = Can.Bus(channel=channel, bitrate=1000000)
    try:
        if fast_frame_rate is not None:
            before = bus._read_parameter_u32(device_id, ParameterFlag.FAST_FRAME_FREQUENCY)
            bus._write_parameter_u32(device_id, ParameterFlag.FAST_FRAME_FREQUENCY, int(fast_frame_rate))
            after = bus._read_parameter_u32(device_id, ParameterFlag.FAST_FRAME_FREQUENCY)
            log.info("Fast frame frequency: %s -> %s", before, after)

        if gear_ratio is not None:
            before = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO)
            bus._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO, -float(gear_ratio))
            after = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO)
            log.info("Gear ratio: %s -> %s", before, after)

        if kp is not None:
            before = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KP)
            bus._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KP, float(kp))
            after = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KP)
            log.info("Position Kp: %s -> %s", before, after)

        if kd is not None:
            before = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KP)
            bus._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KP, float(kd))
            after = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KP)
            log.info("Position Kd: %s -> %s", before, after)

        if torque_limit is not None:
            before = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_LIMIT)
            bus._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_LIMIT, float(torque_limit))
            after = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_LIMIT)
            log.info("Torque limit: %s -> %s", before, after)

        if phase_order is not None:
            before = bus._read_parameter_i32(device_id, ParameterFlag.MOTOR_PHASE_ORDER)
            bus._write_parameter_i32(device_id, ParameterFlag.MOTOR_PHASE_ORDER, int(phase_order))
            after = bus._read_parameter_i32(device_id, ParameterFlag.MOTOR_PHASE_ORDER)
            log.info("Phase order: %s -> %s", before, after)

        if lower_limit is not None:
            before = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_LOWER)
            bus._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, float(lower_limit))
            after = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_LOWER)
            log.info("Position lower limit: %s -> %s", before, after)

        if upper_limit is not None:
            before = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_UPPER)
            bus._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, float(upper_limit))
            after = bus._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_UPPER)
            log.info("Position upper limit: %s -> %s", before, after)

        if bandwidth is not None:
            bus.set_current_bandwidth(device_id, float(bandwidth), 0.0001, 0.000001)
            log.info("Current bandwidth updated to %s Hz", bandwidth)

        if flash:
            bus.store_settings_to_flash(device_id)
            log.info("Configuration saved to flash memory of actuator #%d.", device_id)
    finally:
        bus.stop()


def test_motor_motion(
    channel: str = "can0",
    device_id: int = 1,
    frequency: float = 1.0,
    amplitude: float = 1.0,
    kp: float = 0.2,
    kd: float = 0.005,
    torque_limit: float = 0.2,
) -> None:
    """Run a sine-wave position tracking test on a single CAN motor."""
    from loop_rate_limiters import RateLimiter

    rate = RateLimiter(frequency=200.0)
    bus = Can.Bus(channel=channel, bitrate=1000000)
    log.info(
        "Starting sine wave test on channel=%s, id=%d (freq=%.2f Hz, amp=%.2f rad)...",
        channel,
        device_id,
        frequency,
        amplitude,
    )

    bus.set_mode(device_id, ModeFlag.POSITION)
    bus.feed(device_id)
    bus.write_position_kp(device_id, kp)
    bus.write_position_kd(device_id, kd)
    bus.write_torque_limit(device_id, torque_limit)

    try:
        while True:
            target_angle = float(np.sin(2.0 * np.pi * frequency * time.time()) * amplitude)
            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
            if measured_position is not None and measured_velocity is not None:
                status_line(f"Target: {target_angle:+.3f} | Pos: {measured_position:+.3f} | Vel: {measured_velocity:+.3f}")
            rate.sleep()
    except KeyboardInterrupt:
        log.info("Motor test stopped by user.")
    finally:
        bus.set_mode(device_id, ModeFlag.IDLE)
        bus.stop()
