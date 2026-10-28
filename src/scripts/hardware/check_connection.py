import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[3]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

from src.iteh.robot.humanoid import Humanoid


robot = Humanoid()

robot.check_connection()

robot.stop()
