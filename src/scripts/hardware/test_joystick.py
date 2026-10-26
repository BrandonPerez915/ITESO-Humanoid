"""
test_joystick.py

Test script for gamepad input controller.
"""

import sys
from pathlib import Path

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from iteh.common.paths import REPO_ROOT, SRC_DIR
from iteh.control.policy.gamepad import Se2Gamepad

gamepad = Se2Gamepad()
gamepad.run()

try:
    while True:
        print(
            gamepad.commands["mode_switch"],
            gamepad.commands["velocity_x"],
            gamepad.commands["velocity_y"],
            gamepad.commands["velocity_yaw"]
        )
except KeyboardInterrupt:
    print("\n[INFO] Test finalizado por el usuario.")
