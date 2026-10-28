import sys
from pathlib import Path
import time

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[4]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from iteh.control import recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

bus.set_mode(device_id, recoil.Mode.CALIBRATION)

# the motor should now perform the calibration sequence

# wait for calibration to finish
time.sleep(20)

bus.stop()
