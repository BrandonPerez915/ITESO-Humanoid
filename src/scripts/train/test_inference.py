
import sys
from pathlib import Path
import os

# Ensure project root and src/ are in sys.path
_repo_root = Path(__file__).resolve().parents[2]
_src_dir = _repo_root / "src"
for _path in [str(_repo_root), str(_src_dir)]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import argparse
from typing import Union

import numpy as np
from omegaconf import DictConfig, ListConfig, OmegaConf

try:
    from bhl.control.policy import RlController
    from bhl.common.paths import get_config_path
except ImportError:
    from src.control.policy.rl_controller import RlController
    from src.iteh.common.paths import get_config_path


def parse_arguments() -> Union[DictConfig, ListConfig]:
    """
    Parse command line arguments and load configuration file.

    Returns:
        Union[DictConfig, ListConfig]: Loaded configuration object
    """
    parser = argparse.ArgumentParser(description="Policy Runner for Berkeley Humanoid Lite")
    parser.add_argument("--config", type=str, default="policy_humanoid.yaml",
                       help="Path to the configuration file")
    args = parser.parse_args()

    config_path = str(get_config_path(args.config))
    print("Loading config file from ", config_path)

    with open(config_path, "r") as f:
        cfg = OmegaConf.load(f)

    return cfg


# Load configuration
cfg = parse_arguments()


# Initialize and start policy controller
controller = RlController(cfg)
controller.load_policy()

obs = np.zeros((63,), dtype=np.float32)  # Example observation, adjust as needed

actions = controller.update(obs)
