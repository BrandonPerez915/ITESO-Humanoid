"""
test_joystick.py

Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

Test script for gamepad input controller.
"""

import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    from bhl.control.policy.gamepad import Se2Gamepad
except ImportError:
    try:
        from src.control.policy.gamepad import Se2Gamepad
    except ImportError:
        from control.policy.gamepad import Se2Gamepad

gamepad = Se2Gamepad()
gamepad.run()

while True:
    print(
        gamepad.commands["mode_switch"],
        gamepad.commands["velocity_x"],
        gamepad.commands["velocity_y"],
        gamepad.commands["velocity_yaw"]
    )
