import sys
from pathlib import Path

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[3]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    from bhl.control import recoil
except ImportError:
    try:
        import src.control.recoil as recoil
    except ImportError:
        from control import recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

status = bus.ping(device_id)

if status:
    print("Motor is online")
else:
    print("Motor is offline")

bus.stop()
