"""Library module to evaluate/play a checkpoint of an RL agent from RSL-RL and export ONNX/JIT."""

from __future__ import annotations

import os
from typing import Any

from common.logging import get_logger
from common.paths import CONFIGS_DIR, LOGS_DIR
from trainning.train import parse_rsl_rl_cfg

log = get_logger("train.play")

def run_play_rl(args_cli: Any, simulation_app: Any) -> None:
    """Play and export an RSL-RL checkpoint in Isaac Lab after `AppLauncher` is initialized."""
    import gymnasium as gym
    from omegaconf import OmegaConf
    import torch
    from rsl_rl.runners import OnPolicyRunner

    from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
    from isaaclab.utils.dict import print_dict
    import isaaclab.utils.string as string_utils
    from isaaclab_rl.rsl_rl import (
        RslRlOnPolicyRunnerCfg,
        RslRlVecEnvWrapper,
        export_policy_as_jit,
        export_policy_as_onnx,
    )
    from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg

    import trainning  # noqa: F401

    try:
        env_cfg = parse_env_cfg(
            args_cli.task,
            device=args_cli.device,
            num_envs=args_cli.num_envs,
            use_fabric=not args_cli.disable_fabric,
        )
        agent_cfg: RslRlOnPolicyRunnerCfg = parse_rsl_rl_cfg(args_cli.task, args_cli)

        log_root_path = str((LOGS_DIR / "rsl_rl" / agent_cfg.experiment_name).resolve())
        log.info("Loading experiment from directory: %s", log_root_path)
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
        log_dir = os.path.dirname(resume_path)

        env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

        if isinstance(env.unwrapped, DirectMARLEnv):
            env = multi_agent_to_single_agent(env)

        if args_cli.video:
            video_kwargs = {
                "video_folder": os.path.join(log_dir, "videos", "play"),
                "step_trigger": lambda step: step == 0,
                "video_length": args_cli.video_length,
                "disable_logger": True,
            }
            log.info("Recording evaluation video to: %s", video_kwargs["video_folder"])
            print_dict(video_kwargs, nesting=4)
            env = gym.wrappers.RecordVideo(env, **video_kwargs)

        try:
            env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))
        except TypeError:
            env = RslRlVecEnvWrapper(env)

        log.info("Loading model from checkpoint: %s", resume_path)
        ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        ppo_runner.load(resume_path)

        policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

        try:
            policy_nn = ppo_runner.alg.policy
        except AttributeError:
            policy_nn = ppo_runner.alg.actor_critic

        if hasattr(policy_nn, "actor_obs_normalizer"):
            obs_normalizer = policy_nn.actor_obs_normalizer
        elif hasattr(ppo_runner, "obs_normalizer"):
            obs_normalizer = ppo_runner.obs_normalizer
        else:
            obs_normalizer = None

        export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
        export_policy_as_jit(policy_nn, obs_normalizer, path=export_model_dir, filename="policy.pt")
        export_policy_as_onnx(policy_nn, normalizer=obs_normalizer, path=export_model_dir, filename="policy.onnx")
        log.info("Policy exported (JIT and ONNX) to: %s", export_model_dir)

        # Export deployment YAML configuration
        num_joints = len(env_cfg.scene.robot.init_state.joint_pos)
        joint_names = list(env_cfg.scene.robot.init_state.joint_pos.keys())
        init_joint_pos = list(env_cfg.scene.robot.init_state.joint_pos.values())

        joint_kp = torch.zeros(num_joints, device=env.unwrapped.device)
        joint_kd = torch.zeros(num_joints, device=env.unwrapped.device)
        effort_limits = torch.zeros(num_joints, device=env.unwrapped.device)

        for group in env_cfg.scene.robot.actuators.values():
            match_expr_dict = {expr: None for expr in group.joint_names_expr}
            indices, _, _ = string_utils.resolve_matching_names_values(
                match_expr_dict, joint_names, preserve_order=True
            )
            joint_kp[indices] = group.stiffness
            joint_kd[indices] = group.damping
            effort_limits[indices] = group.effort_limit

        match_expr_list = {expr: None for expr in env_cfg.actions.joint_pos.joint_names}
        action_indices, _, _ = string_utils.resolve_matching_names_values(
            match_expr_list, joint_names, preserve_order=True
        )

        isaac_history_len = env_cfg.observations.policy.actions.history_length or 0
        num_history_frames = max(isaac_history_len, 1)
        total_policy_obs = int(env.observation_space["policy"].shape[-1])
        single_step_obs = total_policy_obs // num_history_frames
        deploy_history_len = max(isaac_history_len - 1, 0)

        deploy_config = {
            "policy_checkpoint_path": f"{export_model_dir}/policy.onnx",
            "ip_robot_addr": "127.0.0.1",
            "ip_policy_obs_port": 10000,
            "ip_host_addr": "127.0.0.1",
            "ip_policy_acs_port": 10001,
            "control_dt": 0.004,
            "policy_dt": env_cfg.sim.dt * env_cfg.decimation,
            "physics_dt": 0.0005,
            "cutoff_freq": 1000,
            "num_joints": num_joints,
            "joints": joint_names,
            "joint_kp": joint_kp.tolist(),
            "joint_kd": joint_kd.tolist(),
            "effort_limits": effort_limits.tolist(),
            "default_base_position": list(env_cfg.scene.robot.init_state.pos),
            "default_joint_positions": init_joint_pos,
            "num_observations": single_step_obs,
            "history_length": deploy_history_len,
            "command_velocity": env_cfg.observations.policy.velocity_commands.func(
                env.unwrapped, env_cfg.observations.policy.velocity_commands.params["command_name"]
            )[0].tolist(),
            "num_actions": env.action_space.shape[-1],
            "action_scale": env_cfg.actions.joint_pos.scale,
            "action_indices": action_indices,
            "action_limit_lower": -10000,
            "action_limit_upper": 10000,
        }
        policies_dir = CONFIGS_DIR / "policies"
        policies_dir.mkdir(parents=True, exist_ok=True)
        OmegaConf.save(deploy_config, str(CONFIGS_DIR / "policy_latest.yaml"))
        OmegaConf.save(deploy_config, str(policies_dir / "policy_latest.yaml"))
        log.info("Deployment configuration saved to: %s", policies_dir / "policy_latest.yaml")

        obs_ret = env.get_observations()
        obs = obs_ret[0] if isinstance(obs_ret, tuple) else obs_ret
        timestep = 0
        while simulation_app.is_running():
            with torch.inference_mode():
                actions = policy(obs)
                step_ret = env.step(actions)
                obs = step_ret[0]
            if args_cli.video:
                timestep += 1
                if timestep == args_cli.video_length:
                    break

        env.close()
    finally:
        simulation_app.close()
