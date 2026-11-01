from __future__ import annotations

import os
from omegaconf import DictConfig, ListConfig, OmegaConf

from common.logging import get_logger
from common.paths import CONFIGS_DIR, get_config_path

log = get_logger("policy.config")


class PolicyConfig(DictConfig):
    """Configuration class holding simulation and robot parameters."""
    # Policy configurations
    policy_checkpoint_path: str
    # Networking configurations
    ip_robot_addr: str
    ip_policy_obs_port: int
    ip_host_addr: str
    ip_policy_acs_port: int
    # Physics configurations
    control_dt: float
    policy_dt: float
    physics_dt: float
    cutoff_freq: float
    # Articulation configurations
    num_joints: int
    joints: list[str]
    joint_kp: list[float] | float
    joint_kd: list[float] | float
    effort_limits: list[float]
    default_base_position: list[float]
    default_joint_positions: list[float]
    # Observation configurations
    num_observations: int
    history_length: int
    # Command configurations
    command_velocity: list[float]
    # Action configurations
    num_actions: int
    action_indices: list[int]
    action_scale: float
    action_limit_lower: float
    action_limit_upper: float

    @staticmethod
    def load(config_name: str = "biped_25hz.yaml", control_mode: str = "keyboard") -> DictConfig | ListConfig:
        """Load a configuration file by name or path."""
        config_path = str(get_config_path(config_name))

        if not os.path.exists(config_path):
            available = [
                str(p.relative_to(CONFIGS_DIR))
                for p in sorted(CONFIGS_DIR.rglob("*.yaml"))
            ]
            raise FileNotFoundError(
                f"Configuration file '{config_name}' not found at '{config_path}'.\n"
                f"Available files in configs/: {', '.join(available)}"
            )

        log.info("Loading configuration file: %s (control=%s)", config_path, control_mode)

        with open(config_path, "r") as f:
            cfg = OmegaConf.load(f)

        cfg.control_mode = control_mode
        return cfg
