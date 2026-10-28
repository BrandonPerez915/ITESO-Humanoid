"""Environment and RL runner configurations for Berkeley Humanoid Lite."""

try:
    import gymnasium as gym
except ImportError:
    try:
        import gym
    except ImportError:
        gym = None

if gym is not None:
    gym.register(
        id="Velocity-Berkeley-Humanoid-Lite-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.enviroment:BerkeleyHumanoidLiteEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.humanoid_rsl:HumanoidPPORunnerConfig",
        },
    )

    gym.register(
        id="Velocity-Berkeley-Humanoid-Lite-Biped-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.enviroment:BerkeleyHumanoidLiteBipedEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.biped_rsl:BipedPPORunnerConfig",
        },
    )

    gym.register(
        id="Velocity-Berkeley-Humanoid-Lite-Robust-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.robust_enviroment:BerkeleyHumanoidLiteRobustEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.humanoid_rsl:BerkeleyHumanoidLiteRobustPPORunnerCfg",
        },
    )

    gym.register(
        id="Velocity-Berkeley-Humanoid-Lite-Legs-Robust-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.robust_enviroment:BerkeleyHumanoidLiteLegsRobustEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.humanoid_rsl:BerkeleyHumanoidLiteLegsRobustPPORunnerCfg",
        },
    )
