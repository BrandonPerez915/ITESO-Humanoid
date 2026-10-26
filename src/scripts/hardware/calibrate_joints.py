"""
calibrate_joints.py

Run this script after each power cycle to calibrate the encoder offset of each joint.
"""

import sys
from pathlib import Path
import time

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from iteh.common.paths import REPO_ROOT, SRC_DIR, get_config_path
from iteh.control.robot import Humanoid

import numpy as np
import yaml

robot = Humanoid()

joint_axis_directions = np.array([
    -1, +1, -1,
    -1,
    -1, +1,
    -1, +1, +1,
    +1,
    +1, +1
])

ideal_values = np.array([
    np.deg2rad(-(10)),
    np.deg2rad(+(33.75)),
    np.deg2rad(+(56.25)),
    np.deg2rad(+(0)),
    np.deg2rad(-(45)),
    np.deg2rad(-(15)),

    np.deg2rad(+(10)),
    np.deg2rad(-(33.75)),
    np.deg2rad(+(56.25)),
    np.deg2rad(+(0)),
    np.deg2rad(-(45)),
    np.deg2rad(+(15)),
])

print("initial readings:")
limit_readings = np.array([joint[0].read_position_measured(joint[1]) for joint in robot.joints]) * joint_axis_directions
print([f"{reading:.2f}" for reading in limit_readings])

robot.enter_damping()

try:
    while True:
        limit_readings = np.array([joint[0].read_position_measured(joint[1]) for joint in robot.joints]) * joint_axis_directions
        print(time.time(), [f"{reading:.2f}" for reading in limit_readings])

        time.sleep(0.05)

    print("final readings at the limits:")
    print([f"{limit:.4f}" for limit in limit_readings])

    print("offsets:")
    print([f"{offset:.4f}" for offset in (limit_readings - ideal_values)])

    calibration_data = {
        "position_offsets": [float(offset) for offset in (limit_readings - ideal_values)],
    }

    cal_path = REPO_ROOT / "calibration.yaml"
    with open(cal_path, "w") as f:
        yaml.dump(calibration_data, f)
    print(f"Calibración guardada en: {cal_path}")

except KeyboardInterrupt:
    print("\n[INFO] Calibración interrumpida por el usuario.")
finally:
    robot.stop()
