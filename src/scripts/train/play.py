"""Script to play a checkpoint of an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

# local imports
import scripts.train.cli_args as cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Play an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during play.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import os

import gymnasium as gym
from omegaconf import OmegaConf
import torch

from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.dict import print_dict
import isaaclab.utils.string as string_utils
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg

# Import extensions to set up environment tasks
import train  # noqa: F401

def main():
    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_dir = os.path.dirname(resume_path)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    try:
        env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))
    except TypeError:
        env = RslRlVecEnvWrapper(env)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # extract policy neural network and normalizer (compatible across rsl_rl versions)
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

    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(policy_nn, obs_normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(policy_nn, normalizer=obs_normalizer, path=export_model_dir, filename="policy.onnx")

    # === export the yaml config for deployment ===
    num_joints = len(env_cfg.scene.robot.init_state.joint_pos)

    # we take the joint order defined from the init joint state entry
    joint_names = [name for name in env_cfg.scene.robot.init_state.joint_pos.keys()]
    init_joint_pos = [v for v in env_cfg.scene.robot.init_state.joint_pos.values()]

    joint_kp = torch.zeros(num_joints, device=env.unwrapped.device)
    joint_kd = torch.zeros(num_joints, device=env.unwrapped.device)
    effort_limits = torch.zeros(num_joints, device=env.unwrapped.device)

    # extract the configurations from the actuator groups
    for group in env_cfg.scene.robot.actuators.values():
        match_expr_list = [expr for expr in group.joint_names_expr]
        match_expr_dict = {expr: None for expr in match_expr_list}

        indicies, _, _ = string_utils.resolve_matching_names_values(match_expr_dict, joint_names, preserve_order=True)
        joint_kp[indicies] = group.stiffness
        joint_kd[indicies] = group.damping
        effort_limits[indicies] = group.effort_limit

    # extract the indices of the actuated joints
    match_expr_list = {expr: None for expr in env_cfg.actions.joint_pos.joint_names}
    action_indices, _, _ = string_utils.resolve_matching_names_values(match_expr_list, joint_names, preserve_order=True)

    isaac_history_len = env_cfg.observations.policy.actions.history_length or 0
    num_history_frames = max(isaac_history_len, 1)
    total_policy_obs = int(env.observation_space["policy"].shape[-1])
    single_step_obs = total_policy_obs // num_history_frames
    deploy_history_len = max(isaac_history_len - 1, 0)

    deploy_config = {
        # === Policy configurations ===
        "policy_checkpoint_path": f"{export_model_dir}/policy.onnx",

        # === Networking configurations ===
        "ip_robot_addr": "127.0.0.1",
        "ip_policy_obs_port": 10000,
        "ip_host_addr": "127.0.0.1",
        "ip_policy_acs_port": 10001,

        # === Physics configurations ===
        "control_dt": 0.004,   # 250 Hz
        "policy_dt": env_cfg.sim.dt * env_cfg.decimation,      # 25 Hz
        "physics_dt": 0.0005,    # 2000 Hz
        "cutoff_freq": 1000,

        # === Articulation configurations ===
        "num_joints": num_joints,
        "joints": joint_names,
        "joint_kp": joint_kp.tolist(),
        "joint_kd": joint_kd.tolist(),
        "effort_limits": effort_limits.tolist(),
        "default_base_position": list(env_cfg.scene.robot.init_state.pos),
        "default_joint_positions": init_joint_pos,

        # === Observation configurations ===
        "num_observations": single_step_obs,
        "history_length": deploy_history_len,

        # === Command configurations ===
        # sample a command
        "command_velocity": env_cfg.observations.policy.velocity_commands.func(
            env.unwrapped, env_cfg.observations.policy.velocity_commands.params["command_name"]
        )[0].tolist(),

        # === Action configurations ===
        "num_actions": env.action_space.shape[-1],
        "action_scale": env_cfg.actions.joint_pos.scale,
        "action_indices": action_indices,
        "action_limit_lower": -10000,
        "action_limit_upper": 10000,
    }
    os.makedirs("configs/policies", exist_ok=True)
    OmegaConf.save(deploy_config, "configs/policy_latest.yaml")
    OmegaConf.save(deploy_config, "configs/policies/policy_latest.yaml")

    # reset environment
    obs_ret = env.get_observations()
    obs = obs_ret[0] if isinstance(obs_ret, tuple) else obs_ret
    timestep = 0
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            step_ret = env.step(actions)
            obs = step_ret[0]
        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
