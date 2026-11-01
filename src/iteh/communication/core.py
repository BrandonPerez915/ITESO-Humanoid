# Copyright (c) 2025, -T.K.-.
from __future__ import annotations
import math
import struct
import time
import can

from common.logging import get_logger
from .flags import FunctionFlag, ModeFlag, ParameterFlag

log = get_logger("comm.can")

class DataFrame:
    def __init__(
        self,
        device_id: int = 0,
        func_id: int | None = None,
        size: int = 0,
        data: bytes | bytearray = b""
    ):
        self.device_id = device_id
        self.func_id = func_id
        self.size = size
        self.data = data

        assert self.size == len(self.data)

class CANFrame(DataFrame):
    ID_STANDARD = 0
    ID_EXTENDED = 1

    DEVICE_ID_MSK = 0x7F
    FUNC_ID_POS = 7
    FUNC_ID_MSK = 0x0F << FUNC_ID_POS

    def __init__(
        self,
        device_id: int = 0,
        func_id: int | None = None,
        size: int = 0,
        data: bytes | bytearray = b""
    ):
        super().__init__(device_id, func_id, size, data)
        assert self.size <= 8

class Bus:
    @staticmethod
    def unpack(format_str: str | bytes, data: bytes |bytearray) -> tuple:
        try:
            return struct.unpack(format_str, data)
        except struct.error as e:
            log.warning("struct unpack warning: %s (%s)", e, data)
            return ()


    def __init__(self, channel: str, bitrate: int = 1000000):
        """
        Args:
            channel (str): The port to use for communication, e.g., "can0"
            bitrate (int): The bitrate for the CAN bus, default is 1 Mbps
        """
        self.channel = channel
        self.bitrate = bitrate

        self.__bus = can.interface.Bus(interface="socketcan", channel=self.channel, bitrate=self.bitrate)


    def __del__(self):
        self.stop()


    def stop(self):
        self.__bus.shutdown()

    """
    Receive data.

    timeout == None: blocking forever
    timeout == 0: non-blocking (the actual delay is around 0.1s)
    timeout > 0: blocking for timeout seconds

    @param timeout: timeout in seconds
    """

    def receive(self,
    filter_device_id: int | None = None,
    filter_function: int | None = None,
    timeout=None
    ) -> CANFrame | None:
        while True:
            try:
                msg = self.__bus.recv(timeout=timeout)
            except can.exceptions.CanOperationError as e:
                log.error("<CANReceive> error: %s", e)
                return None
            except TypeError as e:
                log.error("<CANReceive> error: %s", e)
                return None

            if not msg:
                return None

            if msg.is_error_frame:
                log.warning("<%s> Error Frame: %s, %s", self.channel, msg.arbitration_id, msg.dlc)
                continue

            frame = CANFrame(
                device_id=msg.arbitration_id & CANFrame.DEVICE_ID_MSK,
                func_id=msg.arbitration_id >> CANFrame.FUNC_ID_POS,
                size=msg.dlc,
                data=msg.data
            )
            if filter_device_id:
                if frame.device_id != filter_device_id:
                    continue
            if filter_function:
                if frame.func_id != filter_function:
                    continue

            return frame


    def transmit(self, frame: CANFrame):
        assert frame.device_id <= CANFrame.DEVICE_ID_MSK, "device_id: {0} out of range".format(frame.device_id)
        assert frame.func_id is not None

        can_id = (frame.func_id << CANFrame.FUNC_ID_POS) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)

        self.__bus.send(msg)


    def ping(self, device_id: int, timeout=0.1) -> bool:
        self.transmit(CANFrame(device_id, FunctionFlag.RECEIVE_PDO_1, size=1, data=b"\xCA"))
        rx_frame = self.receive(filter_device_id=device_id, filter_function=FunctionFlag.TRANSMIT_PDO_1, timeout=timeout)
        if not rx_frame:
            return False
        rx_data = self.unpack("<BBBBBBBB", rx_frame.data)[0]
        return rx_data == 0xCA


    def feed(self, device_id: int) -> None:
        self.transmit(CANFrame(device_id, FunctionFlag.HEARTBEAT))


    def set_mode(self, device_id: int, mode: int) -> None:
        self.transmit(CANFrame(
            device_id,
            FunctionFlag.NMT,
            size=2,
            data=struct.pack("<BB", mode, device_id)
        ))


    def load_settings_from_flash(self, device_id: int) -> None:
        self.transmit(CANFrame(
            device_id,
            FunctionFlag.FLASH,
            size=1,
            data=struct.pack("<B", 2)
        ))


    def store_settings_to_flash(self, device_id: int) -> None:
        self.transmit(CANFrame(
            device_id,
            FunctionFlag.FLASH,
            size=1,
            data=struct.pack("<B", 1)
        ))


    def store_setting_to_flash(self, device_id: int) -> None:
        """Alias for store_settings_to_flash."""
        self.store_settings_to_flash(device_id)


    def _read_parameter(self, device_id: int, param_id: int, timeout=None) -> CANFrame | None:
        self.transmit(CANFrame(
            device_id,
            FunctionFlag.RECEIVE_SDO,
            size=3,
            data=struct.pack("<BH", 0x02 << 5, param_id)
        ))
        rx_frame = self.receive(filter_device_id=device_id, filter_function=FunctionFlag.TRANSMIT_SDO, timeout=timeout)
        return rx_frame


    def _write_parameter(self, device_id: int, param_id: int, tx_data: bytes) -> None:
        self.transmit(CANFrame(
            device_id,
            FunctionFlag.RECEIVE_SDO,
            size=8,
            data=struct.pack("<BHB", 0x01 << 5, param_id, 0) + tx_data
        ))


    def _read_parameter_bytes(self, device_id: int, param_id: int, timeout=None) -> bytes | bytearray | None:
        rx_frame = self._read_parameter(device_id, param_id, timeout)
        if not rx_frame:
            return None
        rx_data = rx_frame.data[0:4]
        return rx_data


    def _read_parameter_f32(self, device_id: int, param_id: int, timeout=None) -> float | None:
        rx_frame = self._read_parameter(device_id, param_id, timeout)
        if not rx_frame:
            return None
        rx_data = self.unpack("<f", rx_frame.data[0:4])[0]
        return rx_data


    def _read_parameter_i32(self, device_id: int, param_id: int, timeout=None) -> int | None:
        rx_frame = self._read_parameter(device_id, param_id, timeout)
        if not rx_frame:
            return None
        rx_data = self.unpack("<l", rx_frame.data[0:4])[0]
        return rx_data


    def _read_parameter_u32(self, device_id: int, param_id: int, timeout=None) -> int | None:
        rx_frame = self._read_parameter(device_id, param_id, timeout)
        if not rx_frame:
            return None
        rx_data = self.unpack("<L", rx_frame.data[0:4])[0]
        return rx_data


    def _write_parameter_bytes(self, device_id: int, param_id: int, value: bytes):
        self._write_parameter(device_id, param_id, value)


    def _write_parameter_f32(self, device_id: int, param_id: int, value: float):
        tx_data = struct.pack("<f", value)
        self._write_parameter(device_id, param_id, tx_data)


    def _write_parameter_i32(self, device_id: int, param_id: int, value: int):
        assert isinstance(value, int), "value must be an integer"
        tx_data = struct.pack("<l", value)
        self._write_parameter(device_id, param_id, tx_data)


    def _write_parameter_u32(self, device_id: int, param_id: int, value: int):
        assert isinstance(value, int), "value must be an integer"
        assert value >= 0, "value must be unsigned integer"
        tx_data = struct.pack("<L", value)
        self._write_parameter(device_id, param_id, tx_data)

    # Parameter fields

    def read_fast_frame_frequency(self, device_id: int) -> int | None:
        return self._read_parameter_u32(device_id, ParameterFlag.FAST_FRAME_FREQUENCY)


    def write_fast_frame_frequency(self, device_id: int, value: int) -> None:
        self._write_parameter_u32(device_id, ParameterFlag.FAST_FRAME_FREQUENCY, value)


    def read_gear_ratio(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO)


    def write_gear_ratio(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO, value)


    def read_position_kp(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KP)


    def write_position_kp(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KP, value)


    def read_position_kd(self, device_id: int) -> float | None:
        return self.read_velocity_kp(device_id)


    def write_position_kd(self, device_id: int, value: float):
        self.write_velocity_kp(device_id, value)


    def read_position_ki(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KI)


    def write_position_ki(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_KI, value)


    def read_velocity_kp(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KP)


    def write_velocity_kp(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KP, value)


    def read_velocity_ki(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KI)


    def write_velocity_ki(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_KI, value)


    def read_torque_limit(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_LIMIT)


    def write_torque_limit(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_LIMIT, value)


    def read_velocity_limit(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_LIMIT)


    def write_velocity_limit(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_LIMIT, value)


    def read_position_limit_lower(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_LOWER)


    def write_position_limit_lower(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_LOWER, value)


    def read_position_limit_upper(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_UPPER)


    def write_position_limit_upper(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_LIMIT_UPPER, value)


    def read_position_offset(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_OFFSET)


    def write_position_offset(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_OFFSET, value)


    def read_torque_target(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_TARGET)


    def write_torque_target(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_TARGET, value)


    def read_torque_measured(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_MEASURED)


    def read_velocity_target(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_TARGET)


    def write_velocity_target(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_TARGET, value)


    def read_velocity_measured(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_VELOCITY_MEASURED)


    def read_position_target(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_TARGET)


    def write_position_target(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_TARGET, value)


    def read_position_measured(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_POSITION_MEASURED)


    def read_torque_filter_alpha(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA)


    def write_torque_filter_alpha(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA, value)


    def read_current_limit(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.CURRENT_CONTROLLER_I_LIMIT)


    def write_current_limit(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.CURRENT_CONTROLLER_I_LIMIT, value)


    def read_current_kp(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.CURRENT_CONTROLLER_I_KP)


    def write_current_kp(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.CURRENT_CONTROLLER_I_KP, value)


    def read_current_ki(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.CURRENT_CONTROLLER_I_KI)


    def write_current_ki(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.CURRENT_CONTROLLER_I_KI, value)


    def read_bus_voltage_filter_alpha(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA)


    def write_bus_voltage_filter_alpha(self, device_id: int, value: float):
        self._write_parameter_f32(device_id, ParameterFlag.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA, value)


    def read_motor_pole_pairs(self, device_id: int) -> int | None:
        return self._read_parameter_u32(device_id, ParameterFlag.MOTOR_POLE_PAIRS)


    def write_motor_pole_pairs(self, device_id: int, value: int):
        return self._write_parameter_u32(device_id, ParameterFlag.MOTOR_POLE_PAIRS, value)


    def read_motor_torque_constant(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.MOTOR_TORQUE_CONSTANT)


    def write_motor_torque_constant(self, device_id: int, value: float):
        return self._write_parameter_f32(device_id, ParameterFlag.MOTOR_TORQUE_CONSTANT, value)


    def read_motor_phase_order(self, device_id: int) -> int | None:
        return self._read_parameter_i32(device_id, ParameterFlag.MOTOR_PHASE_ORDER)


    def write_motor_phase_order(self, device_id: int, value: int):
        return self._write_parameter_i32(device_id, ParameterFlag.MOTOR_PHASE_ORDER, value)


    def read_motor_calibration_current(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.MOTOR_MAX_CALIBRATION_CURRENT)


    def write_motor_calibration_current(self, device_id: int, value: float):
        return self._write_parameter_f32(device_id, ParameterFlag.MOTOR_MAX_CALIBRATION_CURRENT, value)


    def read_encoder_cpr(self, device_id: int) -> int | None:
        return self._read_parameter_u32(device_id, ParameterFlag.ENCODER_CPR)


    def write_encoder_cpr(self, device_id: int, value: int):
        return self._write_parameter_u32(device_id, ParameterFlag.ENCODER_CPR, value)


    def read_encoder_position_offset(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.ENCODER_POSITION_OFFSET)


    def write_encoder_position_offset(self, device_id: int, value: float):
        return self._write_parameter_f32(device_id, ParameterFlag.ENCODER_POSITION_OFFSET, value)


    def read_encoder_velocity_filter_alpha(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.ENCODER_VELOCITY_FILTER_ALPHA)


    def write_encoder_velocity_filter_alpha(self, device_id: int, value: float):
        return self._write_parameter_f32(device_id, ParameterFlag.ENCODER_VELOCITY_FILTER_ALPHA, value)


    def read_encoder_flux_offset(self, device_id: int) -> float | None:
        return self._read_parameter_f32(device_id, ParameterFlag.ENCODER_FLUX_OFFSET)


    def write_encoder_flux_offset(self, device_id: int, value: float):
        return self._write_parameter_f32(device_id, ParameterFlag.ENCODER_FLUX_OFFSET, value)


    # Quick helper functions
    def set_current_bandwidth(self, device_id: int, bandwidth_hz: float, phase_resistance: float, phase_inductance: float):
        kp = bandwidth_hz * 2.0 * math.pi * phase_inductance
        ki = phase_resistance / phase_inductance
        log.info("Calculated current kp: %.4f, ki: %.4f", kp, ki)
        self.write_current_kp(device_id, kp)
        self.write_current_ki(device_id, ki)


    def set_torque_bandwidth(self, device_id: int, bandwidth_hz: float, position_loop_rate: float = 2000):
        alpha = min(max(1. - math.exp(-2. * math.pi * (bandwidth_hz / position_loop_rate)), 0.), 1.)
        log.info("Calculated torque filter alpha: %.4f", alpha)
        self.write_torque_filter_alpha(device_id, alpha)


    def set_bus_voltage_bandwidth(self, device_id: int, bandwidth_hz: float, bus_voltage_update_rate: float = 10000):
        alpha = min(max(1. - math.exp(-2. * math.pi * (bandwidth_hz / bus_voltage_update_rate)), 0.), 1.)
        log.info("Calculated bus voltage filter alpha: %.4f", alpha)
        self.write_bus_voltage_filter_alpha(device_id, alpha)


    def set_encoder_velocity_bandwidth(self, device_id: int, bandwidth_hz: float, encoder_update_rate: float = 10000):
        alpha = min(max(1. - math.exp(-2. * math.pi * (bandwidth_hz / encoder_update_rate)), 0.), 1.)
        log.info("Calculated encoder velocity filter alpha: %.4f", alpha)
        self.write_encoder_velocity_filter_alpha(device_id, alpha)


    def write_read_pdo_2(self, device_id: int, position_target: float, velocity_target: float) -> tuple:
        self.transmit_pdo_2(device_id, position_target, velocity_target)
        return self.receive_pdo_2(device_id)


    def transmit_pdo_2(self, device_id: int, position_target: float, velocity_target: float):
        self.transmit(CANFrame(
            device_id,
            FunctionFlag.RECEIVE_PDO_2,
            size=8,
            data=struct.pack("<ff", position_target, velocity_target)
        ))


    def receive_pdo_2(self, device_id: int) -> tuple:
        rx_frame = self.receive(filter_device_id=device_id, timeout=0.001)

        if rx_frame:
            measured_position, measured_velocity = struct.unpack("<ff", rx_frame.data[0:8])
            return measured_position, measured_velocity
        else:
            log.error("<%s> No response from device %d, timeout", self.channel, device_id)
            return None, None
