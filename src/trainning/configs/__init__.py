"""Environment and RL runner configurations for Berkeley Humanoid Lite."""

import gymnasium as gym

import trainning.configs.biped_rsl as biped
import trainning.configs.humanoid_rsl as humanoid
import trainning.configs.robot as robot

import trainning.configs.enviroments.robust as robust_env
import trainning.configs.enviroments.base as base_env
import trainning.configs.enviroments.velocity as velocity_env

__all__ = [
    "biped",
    "humanoid",
    "robot",
    "robust_env",
    "base_env",
    "velocity_env",
]

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.enviroments.base:BerkeleyHumanoidLiteEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.humanoid_rsl:HumanoidPPORunnerConfig",
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.enviroments.base:BerkeleyHumanoidLiteBipedEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.biped_rsl:BipedPPORunnerConfig",
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Robust-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.enviroments.robust:BerkeleyHumanoidLiteRobustEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.humanoid_rsl:BerkeleyHumanoidLiteRobustPPORunnerCfg",
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Legs-Robust-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.enviroments.robust:BerkeleyHumanoidLiteLegsRobustEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.humanoid_rsl:BerkeleyHumanoidLiteLegsRobustPPORunnerCfg",
    },
)
