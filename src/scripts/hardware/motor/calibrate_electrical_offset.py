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

from iteh.common.paths import REPO_ROOT, SRC_DIR
from iteh.control import recoil

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

bus.set_mode(device_id, recoil.Mode.CALIBRATION)

# the motor should now perform the calibration sequence

# wait for calibration to finish
time.sleep(20)

bus.stop()
