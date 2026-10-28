"""Common functions that can be used to create curriculum for the learning environment.

The functions can be passed to the :class:`isaaclab.managers.CurriculumTermCfg` object to enable
the curriculum introduced by the function.
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.terrains import TerrainImporter

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def terrain_levels_vel(
    env: ManagerBasedRLEnv, env_ids: Sequence[int], asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Curriculum based on the distance the robot walked when commanded to move at a desired velocity.

    This term is used to increase the difficulty of the terrain when the robot walks far enough and decrease the
    difficulty when the robot walks less than half of the distance required by the commanded velocity.

    .. note::
        It is only possible to use this term with the terrain type ``generator``. For further information
        on different terrain types, check the :class:`isaaclab.terrains.TerrainImporter` class.

    Returns:
        The mean terrain level for the given environment ids.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    terrain: TerrainImporter = env.scene.terrain
    command = env.command_manager.get_command("base_velocity")
    # compute the distance the robot walked
    distance = torch.norm(asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1)
    # robots that walked far enough progress to harder terrains
    move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
    # robots that walked less than half of their required distance go to simpler terrains
    move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
    move_down *= ~move_up
    # update terrain levels
    terrain.update_env_origins(env_ids, move_up, move_down)
    # return the mean terrain level
    return torch.mean(terrain.terrain_levels.float())

def disturbance_levels_vel(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_lin_vel_xy_exp",
    initial_scale: float = 0.1,
    min_scale: float = 0.1,
    max_scale: float = 1.0,
    up_threshold: float = 0.75,
    down_threshold: float = 0.40,
    step_up: float = 0.05,
    step_down: float = 0.025,
    update_interval_steps: int = 480,
) -> torch.Tensor:
    """Automatic curriculum for the magnitude of disturbances (wrenches, pushes, arm motions).

    Every ``update_interval_steps`` environment steps, the mean episodic value of
    ``reward_term_name`` of the environments being reset is normalized by its weight and the
    maximum episode length (``~1.0`` means perfect tracking for the full episode). If it is above
    ``up_threshold`` the disturbance scale increases by ``step_up``; if it is below
    ``down_threshold`` it decreases by ``step_down``. The scale is stored in
    ``env.robustness_curriculum_scale`` and read by the robustness events.

    Returns:
        The current disturbance scale (logged as ``Curriculum/<term_name>``).
    """
    if not hasattr(env, "robustness_curriculum_scale"):
        env.robustness_curriculum_scale = float(initial_scale)
        env._robustness_curriculum_last_update = 0

    scale = env.robustness_curriculum_scale
    if len(env_ids) == 0 or env.common_step_counter - env._robustness_curriculum_last_update < update_interval_steps:
        return torch.tensor(scale, device=env.device)

    weight = env.reward_manager.get_term_cfg(reward_term_name).weight
    episode_sums = env.reward_manager._episode_sums[reward_term_name][env_ids]
    performance = torch.mean(episode_sums).item() / (env.max_episode_length_s * weight)

    if performance > up_threshold:
        scale += step_up
    elif performance < down_threshold:
        scale -= step_down
    scale = min(max(scale, min_scale), max_scale)

    env.robustness_curriculum_scale = scale
    env._robustness_curriculum_last_update = env.common_step_counter
    return torch.tensor(scale, device=env.device)
