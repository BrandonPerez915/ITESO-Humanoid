import argparse
from typing import Union
import numpy as np
from omegaconf import DictConfig, ListConfig, OmegaConf

from src.iteh.policy.runner import RlController
from src.common.paths import get_config_path

def parse_arguments() -> Union[DictConfig, ListConfig]:
    """
    Parse command line arguments and load configuration file.

    Returns:
        Union[DictConfig, ListConfig]: Loaded configuration object
    """
    parser = argparse.ArgumentParser(description="Policy Runner for Berkeley Humanoid Lite")
    parser.add_argument("--config", type=str, default="humanoid.yaml",
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

obs = np.zeros((7 + cfg.num_actions * 2 + 4,), dtype=np.float32)  # Example observation, adjust as needed

actions = controller.update(obs)
