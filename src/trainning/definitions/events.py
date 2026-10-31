from __future__ import annotations
import math
import torch
from typing import TYPE_CHECKING, Literal

from isaaclab.assets import Articulation
from isaaclab.envs.mdp.events import _randomize_prop_by_op, push_by_setting_velocity
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

# Disturbance scale used before the curriculum term has run for the first time.
DEFAULT_ROBUSTNESS_SCALE = 0.1

def get_robustness_scale(env: ManagerBasedEnv, default: float = DEFAULT_ROBUSTNESS_SCALE) -> float:
    """Return the current robustness curriculum level in ``[0, 1]`` (set by ``disturbance_levels_vel``)."""
    return float(getattr(env, "robustness_curriculum_scale", default))

def get_robust_buffer(env: ManagerBasedEnv, name: str, dim: int) -> torch.Tensor:
    """Return (and lazily create) a per-environment float buffer of shape ``(num_envs, dim)``.

    These buffers store the disturbances applied by the events below so that privileged
    observations for the critic can expose them without touching private simulator state.
    """
    buffers = getattr(env, "_robust_buffers", None)
    if buffers is None:
        buffers = {}
        env._robust_buffers = buffers
    if name not in buffers or buffers[name].shape[-1] != dim:
        buffers[name] = torch.zeros((env.scene.num_envs, dim), dtype=torch.float32, device=env.device)
    return buffers[name]

def _resolve_ids(ids, total: int, device) -> torch.Tensor:
    """Convert resolved SceneEntityCfg ids (list or slice) into a long tensor."""
    if isinstance(ids, slice):
        return torch.arange(total, dtype=torch.long, device=device)[ids]
    return torch.as_tensor(ids, dtype=torch.long, device=device)

def _sample_axis_ranges(axis_range, num: int, num_bodies: int, device) -> torch.Tensor:
    """Sample a ``(num, num_bodies, 3)`` tensor from per-axis ranges.

    ``axis_range`` can be a single ``(low, high)`` tuple applied to every axis or a dict
    with optional ``"x"``, ``"y"``, ``"z"`` keys.
    """
    if isinstance(axis_range, dict):
        ranges = [axis_range.get(key, (0.0, 0.0)) for key in ("x", "y", "z")]
    else:
        ranges = [axis_range] * 3
    ranges_t = torch.tensor(ranges, dtype=torch.float32, device=device)
    low, high = ranges_t[:, 0], ranges_t[:, 1]
    return torch.rand((num, num_bodies, 3), device=device) * (high - low) + low

def randomize_joint_default_pos(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """
    Randomize the joint default positions which may be different from URDF due to calibration errors.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    # resolve joint indices
    if asset_cfg.joint_ids == slice(None):
        joint_ids = slice(None)  # for optimization purposes
    else:
        joint_ids = torch.tensor(asset_cfg.joint_ids, dtype=torch.int, device=asset.device)

    if pos_distribution_params is not None:
        pos = asset.data.default_joint_pos.to(asset.device).clone()
        pos = _randomize_prop_by_op(
            pos, pos_distribution_params, env_ids, joint_ids, operation=operation, distribution=distribution
        )[env_ids][:, joint_ids]

        if env_ids != slice(None) and joint_ids != slice(None):
            env_ids = env_ids[:, None]
        asset.data.default_joint_pos[env_ids, joint_ids] = pos


def randomize_actuator_torque_constant(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    torque_constant_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """
    Randomize the friction parameters used in joint friction model.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    # resolve joint indices
    if asset_cfg.joint_ids == slice(None):
        joint_ids = slice(None)  # for optimization purposes
    else:
        joint_ids = torch.tensor(asset_cfg.joint_ids, dtype=torch.int, device=asset.device)

    # sample joint properties from the given ranges and set into the physics simulation
    # -- friction
    if torque_constant_params is not None:
        for actuator in asset.actuators.values():
            actuator_joint_ids = [joint_id in joint_ids for joint_id in actuator.joint_indices]
            if sum(actuator_joint_ids) > 0:
                stiffness = actuator.stiffness.to(asset.device).clone()
                damping = actuator.damping.to(asset.device).clone()
                scale = _randomize_prop_by_op(
                    torch.ones_like(stiffness, device=asset.device),
                    torque_constant_params,
                    env_ids,
                    actuator_joint_ids,
                    operation=operation,
                    distribution=distribution,
                )
                stiffness[env_ids[:, None], actuator_joint_ids] *= scale[env_ids[:, None], actuator_joint_ids]
                damping[env_ids[:, None], actuator_joint_ids] *= scale[env_ids[:, None], actuator_joint_ids]

                asset.write_joint_stiffness_to_sim(stiffness, joint_ids=actuator.joint_indices, env_ids=env_ids)
                asset.write_joint_damping_to_sim(damping, joint_ids=actuator.joint_indices, env_ids=env_ids)

def randomize_body_com_offset(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    com_range: dict[str, tuple[float, float]],
):
    """Shift the center of mass of the selected bodies by a random offset (body frame, meters).

    Simulates off-center batteries/backpacks on the torso or grasped objects held eccentrically.
    Intended for ``startup`` mode. Offsets are applied relative to the default CoM, so repeated
    calls do not accumulate.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    coms = asset.root_physx_view.get_coms().clone()
    dev = coms.device

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=dev)
    else:
        env_ids = env_ids.to(dev)
    body_ids = _resolve_ids(asset_cfg.body_ids, asset.num_bodies, dev)

    # cache default CoMs on first call to avoid accumulating offsets
    default_coms = getattr(env, "_robust_default_coms", None)
    if default_coms is None:
        default_coms = coms.clone()
        env._robust_default_coms = default_coms

    offsets = _sample_axis_ranges(com_range, len(env_ids), len(body_ids), dev)
    coms[env_ids[:, None], body_ids, :3] = default_coms[env_ids[:, None], body_ids, :3] + offsets
    asset.root_physx_view.set_coms(coms, env_ids)

def randomize_hand_payload(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    mass_range: tuple[float, float] = (0.0, 1.5),
    mode_probs: tuple[float, float, float] = (0.25, 0.45, 0.30),
    payload_radius: float = 0.05,
    buffer_name: str = "hand_payload",
):
    """Attach a random payload (extra mass + inertia) to the hand links.

    For each environment one of three scenarios is sampled with ``mode_probs``:
      0. no payload (free hands),
      1. asymmetric payload: an object in a single (random) hand,
      2. payload in every selected hand (e.g. carrying a box with both hands), with
         independently sampled masses per hand.

    The object is modeled as a solid sphere of radius ``payload_radius`` placed at the link CoM,
    so its inertia ``0.4 * m * r^2`` is added to the link inertia.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    masses = asset.root_physx_view.get_masses()
    dev = masses.device

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=dev)
    else:
        env_ids = env_ids.to(dev)
    body_ids = _resolve_ids(asset_cfg.body_ids, asset.num_bodies, dev)
    num_envs, num_bodies = len(env_ids), len(body_ids)

    # sample payload masses and scenario per environment
    low, high = mass_range
    payload = torch.rand((num_envs, num_bodies), device=dev) * (high - low) + low
    modes = torch.multinomial(torch.tensor(mode_probs, dtype=torch.float32, device=dev), num_envs, replacement=True)

    mask = torch.zeros((num_envs, num_bodies), device=dev)
    mask[modes == 2] = 1.0
    single = torch.nonzero(modes == 1).squeeze(-1)
    if len(single) > 0:
        chosen = torch.randint(0, num_bodies, (len(single),), device=dev)
        mask[single, chosen] = 1.0
    payload = payload * mask

    # update masses
    default_mass = asset.data.default_mass.to(dev)[env_ids[:, None], body_ids]
    masses[env_ids[:, None], body_ids] = default_mass + payload
    asset.root_physx_view.set_masses(masses, env_ids)

    # update inertias (diagonal entries of the flattened 3x3 tensor)
    inertias = asset.root_physx_view.get_inertias()
    default_inertia = asset.data.default_inertia.to(dev)[env_ids[:, None], body_ids].clone()
    sphere_inertia = 0.4 * payload * payload_radius**2
    for diag_idx in (0, 4, 8):
        default_inertia[..., diag_idx] += sphere_inertia
    inertias[env_ids[:, None], body_ids] = default_inertia
    asset.root_physx_view.set_inertias(inertias, env_ids)

    # store for privileged observations
    buffer = get_robust_buffer(env, buffer_name, num_bodies)
    buffer[env_ids.to(env.device)] = payload.to(env.device)

def apply_curriculum_external_wrench(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    force_range: tuple[float, float] | dict[str, tuple[float, float]],
    torque_range: tuple[float, float] | dict[str, tuple[float, float]],
    active_prob: float = 1.0,
    buffer_name: str | None = None,
):
    """Apply random 3D forces/torques (body frame) scaled by the robustness curriculum.

    The wrench persists until the term is triggered again, so in ``interval`` mode it behaves as a
    sustained push/pull (e.g. somebody pulling the hand, an object being handed over) that changes
    every few seconds. With ``active_prob < 1`` some environments receive no wrench at each
    resample, mixing sustained and intermittent disturbances.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    num_bodies = len(_resolve_ids(asset_cfg.body_ids, asset.num_bodies, "cpu"))
    num_envs = len(env_ids)
    if num_envs == 0:
        return

    scale = get_robustness_scale(env)
    forces = _sample_axis_ranges(force_range, num_envs, num_bodies, asset.device) * scale
    torques = _sample_axis_ranges(torque_range, num_envs, num_bodies, asset.device) * scale

    if active_prob < 1.0:
        active = (torch.rand((num_envs, 1, 1), device=asset.device) < active_prob).float()
        forces = forces * active
        torques = torques * active

    # Some Isaac Lab versions disable *all* external wrenches of the asset when a call passes
    # only zeros. Keep a negligible non-zero value to avoid wiping wrenches from other terms.
    if not (forces.any() or torques.any()):
        forces[..., 2] = 1.0e-6

    asset.set_external_force_and_torque(forces, torques, env_ids=env_ids, body_ids=asset_cfg.body_ids)

    if buffer_name is not None:
        buffer = get_robust_buffer(env, buffer_name, num_bodies * 6)
        buffer[env_ids] = torch.cat([forces, torques], dim=-1).reshape(num_envs, -1)

def push_by_setting_velocity_curriculum(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    velocity_range: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Impulsive push (velocity kick) whose magnitude grows with the robustness curriculum."""
    scale = get_robustness_scale(env)
    scaled_range = {key: (low * scale, high * scale) for key, (low, high) in velocity_range.items()}
    push_by_setting_velocity(env, env_ids, scaled_range, asset_cfg)

def drive_random_arm_movements(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    offset_range: tuple[float, float] = (-0.6, 0.6),
    amplitude_range: tuple[float, float] = (0.0, 0.6),
    frequency_range: tuple[float, float] = (0.2, 1.5),
    resample_interval_s: tuple[float, float] = (1.5, 4.0),
    static_prob: float = 0.15,
):
    """Move the arm joints (not controlled by the policy) with random postures + oscillations.

    Target per joint: ``default + scale * (offset + amplitude * sin(2*pi*f*t + phase))`` clamped to
    the soft joint limits, where ``scale`` is the robustness curriculum level. Parameters are
    resampled every ``resample_interval_s`` seconds and at every episode reset. With probability
    ``static_prob`` the oscillation is disabled (static random posture, e.g. holding an object).

    Use in ``interval`` mode with ``interval_range_s=(0.0, 0.0)`` so it runs every policy step.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    device = asset.device

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=device)
    if len(env_ids) == 0:
        return
    joint_ids = _resolve_ids(asset_cfg.joint_ids, asset.num_joints, device)
    num_joints = len(joint_ids)

    offset = get_robust_buffer(env, "arm_motion_offset", num_joints)
    amplitude = get_robust_buffer(env, "arm_motion_amplitude", num_joints)
    frequency = get_robust_buffer(env, "arm_motion_frequency", num_joints)
    phase = get_robust_buffer(env, "arm_motion_phase", num_joints)
    timer = get_robust_buffer(env, "arm_motion_timer", 1)

    dt = env.step_dt
    timer[env_ids] -= dt
    needs_resample = (timer[env_ids, 0] <= 0.0) | (env.episode_length_buf[env_ids] <= 1)
    ids = env_ids[needs_resample]
    if len(ids) > 0:
        n = len(ids)
        offset[ids] = torch.rand((n, num_joints), device=device) * (offset_range[1] - offset_range[0]) + offset_range[0]
        amp = torch.rand((n, num_joints), device=device) * (amplitude_range[1] - amplitude_range[0]) + amplitude_range[0]
        static = (torch.rand((n, 1), device=device) < static_prob).float()
        amplitude[ids] = amp * (1.0 - static)
        frequency[ids] = torch.rand((n, num_joints), device=device) * (frequency_range[1] - frequency_range[0]) + frequency_range[0]
        phase[ids] = torch.rand((n, num_joints), device=device) * 2.0 * math.pi
        timer[ids, 0] = torch.rand(n, device=device) * (resample_interval_s[1] - resample_interval_s[0]) + resample_interval_s[0]

    scale = get_robustness_scale(env)
    t = (env.episode_length_buf[env_ids].float() * dt).unsqueeze(-1)
    motion = offset[env_ids] + amplitude[env_ids] * torch.sin(2.0 * math.pi * frequency[env_ids] * t + phase[env_ids])

    default_pos = asset.data.default_joint_pos[env_ids][:, joint_ids]
    limits = asset.data.soft_joint_pos_limits[env_ids][:, joint_ids]
    target = torch.clamp(default_pos + scale * motion, limits[..., 0], limits[..., 1])

    asset.set_joint_position_target(target, joint_ids=joint_ids.tolist(), env_ids=env_ids)
