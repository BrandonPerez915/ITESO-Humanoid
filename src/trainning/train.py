"""Library module to train an RL agent with RSL-RL in Isaac Lab."""

from __future__ import annotations

from datetime import datetime
import os
import pickle
import sys
from typing import TYPE_CHECKING, Any

from common.logging import get_logger
from common.paths import LOGS_DIR

if TYPE_CHECKING:
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg

log = get_logger("train.run")

REGISTERED_TASKS = (
    (
        "Velocity-Berkeley-Humanoid-Lite-v0",
        "Full Humanoid (22 DOFs) - Standard velocity tracking",
    ),
    (
        "Velocity-Berkeley-Humanoid-Lite-Biped-v0",
        "Biped (12 Leg DOFs) - Standard velocity tracking",
    ),
    (
        "Velocity-Berkeley-Humanoid-Lite-Robust-v0",
        "Full Humanoid (22 DOFs) - Robust training (payloads, arm wrenches, CoM, curriculum)",
    ),
    (
        "Velocity-Berkeley-Humanoid-Lite-Legs-Robust-v0",
        "Legs-controlled Policy with 22-DOF URDF - Robust training with random arm perturbations",
    ),
)


def update_rsl_rl_cfg(agent_cfg: RslRlOnPolicyRunnerCfg, args_cli: Any) -> RslRlOnPolicyRunnerCfg:
    """Update configuration for RSL-RL agent based on CLI inputs."""
    if getattr(args_cli, "seed", None) is not None:
        agent_cfg.seed = args_cli.seed
    if getattr(args_cli, "resume", None) is not None:
        agent_cfg.resume = args_cli.resume
    if getattr(args_cli, "load_run", None) is not None:
        agent_cfg.load_run = args_cli.load_run
    if getattr(args_cli, "checkpoint", None) is not None:
        agent_cfg.load_checkpoint = args_cli.checkpoint
    if getattr(args_cli, "run_name", None) is not None:
        agent_cfg.run_name = args_cli.run_name
    if getattr(args_cli, "logger", None) is not None:
        agent_cfg.logger = args_cli.logger
    if agent_cfg.logger in {"wandb", "neptune"} and getattr(args_cli, "log_project_name", None):
        agent_cfg.wandb_project = args_cli.log_project_name
        agent_cfg.neptune_project = args_cli.log_project_name
    return agent_cfg


def parse_rsl_rl_cfg(task_name: str, args_cli: Any) -> RslRlOnPolicyRunnerCfg:
    """Load and update the RSL-RL configuration from the Gymnasium registry."""
    from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

    rslrl_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(task_name, "rsl_rl_cfg_entry_point")
    return update_rsl_rl_cfg(rslrl_cfg, args_cli)


def dump_pickle(filename: str, data: object) -> None:
    """Saves data into a PKL file safely."""
    if not filename.endswith("pkl"):
        filename += ".pkl"
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, "wb") as f:
        pickle.dump(data, f)


def run_train(args_cli: Any, hydra_args: list[str], simulation_app: Any) -> None:
    """Execute RSL-RL training after `AppLauncher` has been initialized."""
    import gymnasium as gym
    import torch
    from rsl_rl.runners import OnPolicyRunner

    from isaaclab.envs import (
        DirectMARLEnv,
        DirectMARLEnvCfg,
        DirectRLEnvCfg,
        ManagerBasedRLEnvCfg,
        multi_agent_to_single_agent,
    )
    from isaaclab.utils.dict import print_dict
    from isaaclab.utils.io import dump_yaml
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
    from isaaclab_tasks.utils import get_checkpoint_path
    from isaaclab_tasks.utils.hydra import hydra_task_config

    import trainning  # noqa: F401

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.deterministic = False
    torch.backends.cudnn.benchmark = False

    # Clear sys.argv for Hydra task decorator
    sys.argv = [sys.argv[0]] + list(hydra_args)

    @hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
    def _train_task(
        env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
        agent_cfg: RslRlOnPolicyRunnerCfg,
    ) -> None:
        agent_cfg = update_rsl_rl_cfg(agent_cfg, args_cli)
        env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
        agent_cfg.max_iterations = (
            args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
        )

        env_cfg.seed = agent_cfg.seed
        env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

        log_root_path = str((LOGS_DIR / "rsl_rl" / agent_cfg.experiment_name).resolve())
        log.info("Guardando experimento RSL-RL en directorio: %s", log_root_path)

        isaaclab_log_dir = os.path.join(log_root_path, "isaaclab")
        os.makedirs(isaaclab_log_dir, exist_ok=True)
        if hasattr(env_cfg, "log_dir"):
            env_cfg.log_dir = isaaclab_log_dir
        if hasattr(env_cfg.sim, "log_dir"):
            env_cfg.sim.log_dir = isaaclab_log_dir

        run_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if agent_cfg.run_name:
            run_folder += f"_{agent_cfg.run_name}"
        log_dir = os.path.join(log_root_path, run_folder)

        resume_path = None
        if agent_cfg.resume:
            resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

        env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

        if isinstance(env.unwrapped, DirectMARLEnv):
            env = multi_agent_to_single_agent(env)

        if args_cli.video:
            video_kwargs = {
                "video_folder": os.path.join(log_dir, "videos", "train"),
                "step_trigger": lambda step: step % args_cli.video_interval == 0,
                "video_length": args_cli.video_length,
                "disable_logger": True,
            }
            log.info("Recording videos during training to: %s", video_kwargs["video_folder"])
            print_dict(video_kwargs, nesting=4)
            env = gym.wrappers.RecordVideo(env, **video_kwargs)

        try:
            env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))
        except TypeError:
            env = RslRlVecEnvWrapper(env)

        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
        runner.add_git_repo_to_log(__file__)

        if agent_cfg.resume and resume_path is not None:
            log.info("Loading checkpoint from: %s", resume_path)
            runner.load(resume_path)

        dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
        dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
        dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
        dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

        log.info("Starting training for %d iterations...", agent_cfg.max_iterations)
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        env.close()
        log.info("Training finished. Checkpoints in: %s", log_dir)

    try:
        _train_task()  # type: ignore[call-arg]
    finally:
        simulation_app.close()
