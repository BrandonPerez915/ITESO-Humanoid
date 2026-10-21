import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    from bhl.control.robot.bimanual import Bimanual
except ImportError:
    from src.control.robot.bimanual import Bimanual


robot = Bimanual()

try:
    while True:
        robot.check_connection()
except KeyboardInterrupt:
    print("Stopping robot due to keyboard interrupt.")

robot.stop()
