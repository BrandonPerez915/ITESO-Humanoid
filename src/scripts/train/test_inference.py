import sys
from pathlib import Path
import os
import argparse
from typing import Union, cast

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import numpy as np
from omegaconf import DictConfig, ListConfig, OmegaConf

from iteh.common.paths import REPO_ROOT, SRC_DIR, get_config_path
from iteh.control.policy import RlController, Cfg


def parse_arguments() -> Cfg:
    """
    Parse command line arguments and load configuration file.

    Returns:
        Cfg: Loaded configuration object
    """
    parser = argparse.ArgumentParser(description="Policy Runner for ITESO Humanoid / Berkeley Humanoid Lite")
    parser.add_argument(
        "--config",
        type=str,
        default="humanoid.yaml",
        help="Path to or name of the configuration file",
    )
    args = parser.parse_args()

    config_path = str(get_config_path(args.config))
    print("Loading config file from ", config_path)

    with open(config_path, "r") as f:
        cfg = OmegaConf.load(f)

    return cast(Cfg, cfg)


# Load configuration
cfg = parse_arguments()

# Initialize and start policy controller
controller = RlController(cfg)
controller.load_policy()

obs = np.zeros((63,), dtype=np.float32)  # Example observation, adjust as needed

actions = controller.update(obs)
print("[INFO] Test inference succeeded. Actions shape:", actions.shape if actions is not None else None)
