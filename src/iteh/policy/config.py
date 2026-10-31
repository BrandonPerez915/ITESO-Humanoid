from __future__ import annotations
import argparse
import os
from omegaconf import DictConfig, ListConfig, OmegaConf

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
    def from_arguments() -> DictConfig | ListConfig:
        """
        Parse command line arguments and load configuration file.

        Returns:
            DictConfig | ListConfig: Loaded configuration object
        """
        parser = argparse.ArgumentParser(description="Policy Runner for Berkeley Humanoid Lite")
        parser.add_argument(
            "--config",
            type=str,
            default="biped_25hz.yaml",
            help="Path to the configuration file",
        )
        parser.add_argument(
            "--control",
            type=str,
            default="keyboard",
            choices=["keyboard", "ros2", "gamepad"],
            help="Input controller mode: 'keyboard', 'ros2', or 'gamepad'",
        )
        args, _ = parser.parse_known_args()

        return PolicyConfig.load(args.config, control_mode=args.control)

    @staticmethod
    def load(config_name: str = "biped_25hz.yaml", control_mode: str = "keyboard") -> DictConfig | ListConfig:
        """Load a configuration file by name or path."""
        from common.paths import get_config_path, CONFIGS_DIR
        config_path = str(get_config_path(config_name))

        if not os.path.exists(config_path):
            available = [
                str(p.relative_to(CONFIGS_DIR))
                for p in sorted(CONFIGS_DIR.rglob("*.yaml"))
            ]
            raise FileNotFoundError(
                f"Archivo de configuración '{config_name}' no encontrado en '{config_path}'.\n"
                f"Archivos disponibles en configs/: {', '.join(available)}"
            )

        print("Loading config file from ", config_path)
        print("Selected control mode  : ", control_mode)

        with open(config_path, "r") as f:
            cfg = OmegaConf.load(f)

        cfg.control_mode = control_mode
        return cfg
