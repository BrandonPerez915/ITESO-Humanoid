"""
test_joystick.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Test script for gamepad input controller.
"""

from src.iteh.inputs.gamepad import Se2Gamepad

gamepad = Se2Gamepad()
gamepad.run()

while True:
    print(
        gamepad.commands["mode_switch"],
        gamepad.commands["velocity_x"],
        gamepad.commands["velocity_y"],
        gamepad.commands["velocity_yaw"]
    )
