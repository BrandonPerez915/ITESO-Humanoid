"""Privileged observation terms for the asymmetric critic of the robust locomotion tasks.

These terms expose disturbances that the real robot cannot measure (wrist forces, payload masses,
contact forces). They must only be used in the ``critic`` observation group.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

from .events import get_robust_buffer, get_robustness_scale

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def applied_external_wrench(env: ManagerBasedRLEnv, buffer_name: str, num_bodies: int, scale: float = 0.1) -> torch.Tensor:
    """External forces/torques applied by ``apply_curriculum_external_wrench`` (``num_bodies * 6`` values)."""
    return get_robust_buffer(env, buffer_name, num_bodies * 6) * scale


def hand_payload_mass(env: ManagerBasedRLEnv, buffer_name: str = "hand_payload", num_bodies: int = 2) -> torch.Tensor:
    """Extra mass (kg) attached to each hand by ``randomize_hand_payload``."""
    return get_robust_buffer(env, buffer_name, num_bodies)


def robustness_curriculum_level(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Current disturbance curriculum level, broadcast to every environment."""
    return torch.full((env.scene.num_envs, 1), get_robustness_scale(env), device=env.device)


def feet_contact_forces(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, scale: float = 0.01) -> torch.Tensor:
    """Net contact forces (world frame) on the selected bodies, flattened and scaled."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]
    return forces.reshape(env.scene.num_envs, -1) * scale
