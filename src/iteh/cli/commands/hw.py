"""`iteh hw` subcommands: `imu`, `joystick`, `gripper`, and `motor` (`get-config`, `set-param`, `test`)."""

from __future__ import annotations

import argparse


def register(subparsers: argparse._SubParsersAction, common_parent: argparse.ArgumentParser) -> None: # type: ignore
    p_hw = subparsers.add_parser(
        "hw",
        parents=[common_parent],
        help="Individual hardware testing (gamepad, grippers, CAN motors).",
        description="Diagnostic and configuration commands for hardware components.",
    )
    hw_sub = p_hw.add_subparsers(dest="subcommand", metavar="<subcommand>", required=True)

    # iteh hw joystick
    p_joy = hw_sub.add_parser(
        "joystick",
        parents=[common_parent],
        help="Test command reading from Gamepad controller.",
    )
    p_joy.set_defaults(func=_cmd_hw_joystick, command_id="hw.joystick", allow_unknown_args=False)

    # iteh hw gripper
    p_grip = hw_sub.add_parser(
        "gripper",
        parents=[common_parent],
        help="Send open/close commands to grippers via serial port.",
    )
    p_grip.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port for gripper.")
    p_grip.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate.")
    p_grip.add_argument("--left", type=float, default=0.8, help="Left gripper target (0.25 open, 0.85 closed).")
    p_grip.add_argument("--right", type=float, default=0.8, help="Right gripper target (0.25 open, 0.85 closed).")
    p_grip.set_defaults(func=_cmd_hw_gripper, command_id="hw.gripper", allow_unknown_args=False)

    # iteh hw motor <action>
    p_motor = hw_sub.add_parser(
        "motor",
        parents=[common_parent],
        help="Individual operations on a CAN actuator (get-config, set-param, test).",
    )
    motor_sub = p_motor.add_subparsers(dest="motor_action", metavar="<action>", required=True)

    # iteh hw motor get-config
    p_mget = motor_sub.add_parser(
        "get-config",
        parents=[common_parent],
        help="Read parameters from an individual CAN actuator and save to JSON.",
    )
    p_mget.add_argument("channel", type=str, help="CAN channel (e.g. can0).")
    p_mget.add_argument("device_id", type=int, help="CAN actuator ID.")
    p_mget.add_argument("--output", type=str, default="motor_configuration.json", help="Output JSON file.")
    p_mget.set_defaults(func=_cmd_motor_get_config, command_id="hw.motor.get_config", allow_unknown_args=False)

    # iteh hw motor set-param
    p_mset = motor_sub.add_parser(
        "set-param",
        parents=[common_parent],
        help="Modify specific parameters on an individual CAN actuator.",
    )
    p_mset.add_argument("channel", type=str, help="CAN channel (e.g. can0).")
    p_mset.add_argument("device_id", type=int, help="CAN actuator ID.")
    p_mset.add_argument("--fast-frame-rate", type=int, default=None, help="Fast frame frequency.")
    p_mset.add_argument("--gear-ratio", type=float, default=None, help="Gear reduction ratio.")
    p_mset.add_argument("--kp", type=float, default=None, help="Position proportional gain Kp.")
    p_mset.add_argument("--kd", type=float, default=None, help="Position derivative gain Kd.")
    p_mset.add_argument("--torque-limit", type=float, default=None, help="Torque limit.")
    p_mset.add_argument("--phase-order", type=int, default=None, help="Motor phase order (1 or -1).")
    p_mset.add_argument("--lower-limit", type=float, default=None, help="Position lower limit (rad).")
    p_mset.add_argument("--upper-limit", type=float, default=None, help="Position upper limit (rad).")
    p_mset.add_argument("--bandwidth", type=float, default=None, help="Current controller bandwidth.")
    p_mset.add_argument("--flash", action="store_true", default=False, help="Save changes to flash memory.")
    p_mset.set_defaults(func=_cmd_motor_set_param, command_id="hw.motor.set_param", allow_unknown_args=False)

    # iteh hw motor test
    p_mtest = motor_sub.add_parser(
        "test",
        parents=[common_parent],
        help="Drive a CAN actuator following a sinusoidal trajectory.",
    )
    p_mtest.add_argument("channel", type=str, help="CAN channel (e.g. can0).")
    p_mtest.add_argument("device_id", type=int, help="CAN actuator ID.")
    p_mtest.add_argument("--frequency", type=float, default=1.0, help="Sinusoidal frequency in Hz.")
    p_mtest.add_argument("--amplitude", type=float, default=1.0, help="Amplitude in radians.")
    p_mtest.add_argument("--kp", type=float, default=0.2, help="Position gain Kp.")
    p_mtest.add_argument("--kd", type=float, default=0.005, help="Derivative gain Kd.")
    p_mtest.add_argument("--torque-limit", type=float, default=0.2, help="Torque limit in Nm.")
    p_mtest.set_defaults(func=_cmd_motor_test, command_id="hw.motor.test", allow_unknown_args=False)


def _cmd_hw_joystick(args: argparse.Namespace, extra_args: list[str]) -> int:
    del args, extra_args
    from iteh.operations.hardware import test_joystick

    test_joystick()
    return 0


def _cmd_hw_gripper(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.hardware import test_gripper

    test_gripper(port=args.port, baudrate=args.baudrate, left_target=args.left, right_target=args.right)
    return 0


def _cmd_motor_get_config(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.hardware import read_motor_config

    read_motor_config(channel=args.channel, device_id=args.device_id, output_path=args.output)
    return 0


def _cmd_motor_set_param(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.hardware import configure_motor_parameter

    configure_motor_parameter(
        channel=args.channel,
        device_id=args.device_id,
        fast_frame_rate=args.fast_frame_rate,
        gear_ratio=args.gear_ratio,
        kp=args.kp,
        kd=args.kd,
        torque_limit=args.torque_limit,
        phase_order=args.phase_order,
        lower_limit=args.lower_limit,
        upper_limit=args.upper_limit,
        bandwidth=args.bandwidth,
        flash=args.flash,
    )
    return 0


def _cmd_motor_test(args: argparse.Namespace, extra_args: list[str]) -> int:
    del extra_args
    from iteh.operations.hardware import test_motor_motion

    test_motor_motion(
        channel=args.channel,
        device_id=args.device_id,
        frequency=args.frequency,
        amplitude=args.amplitude,
        kp=args.kp,
        kd=args.kd,
        torque_limit=args.torque_limit,
    )
    return 0
