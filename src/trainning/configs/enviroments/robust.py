"""Robust locomotion environments for the Berkeley Humanoid Lite / ITESO Humanoid.

Two tasks are defined:

* ``BerkeleyHumanoidLiteRobustEnvCfg`` (22 DoF): the policy controls arms and legs while it
  receives hand payloads, external wrenches on hands/elbows/torso, pushes and actuator/physics
  randomization.
* ``BerkeleyHumanoidLiteLegsRobustEnvCfg`` (12 DoF on the 22 DoF robot): the policy controls only
  the legs, while the arms are moved by a random motion generator (simulating teleoperation or
  manipulation) on top of all the disturbances above.

Both use an asymmetric actor-critic: the actor only sees proprioception with a short history
(deployable on hardware), the critic additionally gets privileged information (true base linear
velocity, applied wrenches, payload masses, foot contact forces, curriculum level).

All disturbance magnitudes are scaled by an automatic curriculum (``mdp.disturbance_levels_vel``).
"""

from __future__ import annotations

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
try:
    from ... import mdp
    from ..robot import ARM_JOINTS, HUMANOID_JOINTS, LEG_JOINTS
    from .base import BerkeleyHumanoidLiteEnvCfg, EventsCfg, RewardsCfg
except (ImportError, ValueError):
    from trainning import mdp
    from trainning.configs.robot import ARM_JOINTS, HUMANOID_JOINTS, LEG_JOINTS
    from trainning.configs.enviroments.base import BerkeleyHumanoidLiteEnvCfg, EventsCfg, RewardsCfg

# Total number of frames buffered per observation (4 past + 1 current = 0.2 s at 25 Hz).
ACTOR_HISTORY_LENGTH = 5
HAND_BODIES = "arm_.*_elbow_roll"
ELBOW_BODIES = "arm_.*_elbow_pitch"
FEET_BODIES = ".*_ankle_roll"

def _policy_terms(joint_names: list[str]) -> dict:
    """Proprioceptive actor terms. The order must match ``RlController`` frame layout:
    command, base angular velocity, projected gravity, joint pos, joint vel, last action."""

    joint_config = SceneEntityCfg("robot", joint_names=joint_names, preserve_order=True)
    return dict(
        velocity_commands=ObsTerm(
            func=mdp.generated_commands, params={"command_name": "base_velocity"},
            history_length=ACTOR_HISTORY_LENGTH,
        ),
        base_ang_vel=ObsTerm(
            func=mdp.base_ang_vel, noise=Unoise(n_min=-0.3, n_max=0.3),
            history_length=ACTOR_HISTORY_LENGTH,
        ),
        projected_gravity=ObsTerm(
            func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05),
            history_length=ACTOR_HISTORY_LENGTH,
        ),
        joint_pos=ObsTerm(
            func=mdp.joint_pos_rel, params={"asset_cfg": joint_config}, noise=Unoise(n_min=-0.05, n_max=0.05),
            history_length=ACTOR_HISTORY_LENGTH,
        ),
        joint_vel=ObsTerm(
            func=mdp.joint_vel_rel, params={"asset_cfg": joint_config}, noise=Unoise(n_min=-2.0, n_max=2.0),
            history_length=ACTOR_HISTORY_LENGTH,
        ),
        actions=ObsTerm(func=mdp.last_action, history_length=ACTOR_HISTORY_LENGTH),
    )


@configclass
class RobustPolicyObsCfg(ObsGroup):
    """Actor observations (22 DoF): proprioception with history, deployable on hardware."""

    velocity_commands = _policy_terms(HUMANOID_JOINTS)["velocity_commands"]
    base_ang_vel = _policy_terms(HUMANOID_JOINTS)["base_ang_vel"]
    projected_gravity = _policy_terms(HUMANOID_JOINTS)["projected_gravity"]
    joint_pos = _policy_terms(HUMANOID_JOINTS)["joint_pos"]
    joint_vel = _policy_terms(HUMANOID_JOINTS)["joint_vel"]
    actions = _policy_terms(HUMANOID_JOINTS)["actions"]

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True


@configclass
class RobustCriticObsCfg(ObsGroup):
    """Critic observations: noise-free proprioception with history + privileged information."""

    velocity_commands = ObsTerm(
        func=mdp.generated_commands, params={"command_name": "base_velocity"}, history_length=ACTOR_HISTORY_LENGTH
    )
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel, history_length=ACTOR_HISTORY_LENGTH)
    projected_gravity = ObsTerm(func=mdp.projected_gravity, history_length=ACTOR_HISTORY_LENGTH)
    joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HUMANOID_JOINTS, preserve_order=True)},
        history_length=ACTOR_HISTORY_LENGTH,
    )
    joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HUMANOID_JOINTS, preserve_order=True)},
        history_length=ACTOR_HISTORY_LENGTH,
    )
    actions = ObsTerm(func=mdp.last_action, history_length=ACTOR_HISTORY_LENGTH)

    # --- privileged information (simulation only) ---
    base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
    hand_wrench = ObsTerm(func=mdp.applied_external_wrench, params={"buffer_name": "hand_wrench", "num_bodies": 2})
    elbow_wrench = ObsTerm(func=mdp.applied_external_wrench, params={"buffer_name": "elbow_wrench", "num_bodies": 2})
    base_wrench = ObsTerm(func=mdp.applied_external_wrench, params={"buffer_name": "base_wrench", "num_bodies": 1})
    hand_payload = ObsTerm(func=mdp.hand_payload_mass, params={"buffer_name": "hand_payload", "num_bodies": 2})
    feet_contact = ObsTerm(
        func=mdp.feet_contact_forces,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET_BODIES)},
    )
    disturbance_level = ObsTerm(func=mdp.robustness_curriculum_level)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True


@configclass
class RobustObservationsCfg:
    policy: RobustPolicyObsCfg = RobustPolicyObsCfg()
    critic: RobustCriticObsCfg = RobustCriticObsCfg()


@configclass
class LegsRobustPolicyObsCfg(ObsGroup):
    """Actor observations (12 DoF legs). Arms are disturbances, inferred through the history."""

    velocity_commands = _policy_terms(LEG_JOINTS)["velocity_commands"]
    base_ang_vel = _policy_terms(LEG_JOINTS)["base_ang_vel"]
    projected_gravity = _policy_terms(LEG_JOINTS)["projected_gravity"]
    joint_pos = _policy_terms(LEG_JOINTS)["joint_pos"]
    joint_vel = _policy_terms(LEG_JOINTS)["joint_vel"]
    actions = _policy_terms(LEG_JOINTS)["actions"]

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True


@configclass
class LegsRobustCriticObsCfg(RobustCriticObsCfg):
    """Critic for the legs task: leg proprioception + arm state as privileged information."""

    joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS, preserve_order=True)},
        history_length=ACTOR_HISTORY_LENGTH,
    )
    joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS, preserve_order=True)},
        history_length=ACTOR_HISTORY_LENGTH,
    )
    arm_joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINTS, preserve_order=True)},
    )
    arm_joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINTS, preserve_order=True)},
    )


@configclass
class LegsRobustObservationsCfg:
    policy: LegsRobustPolicyObsCfg = LegsRobustPolicyObsCfg()
    critic: LegsRobustCriticObsCfg = LegsRobustCriticObsCfg()


##
# Events (payloads, CoM, wrenches, pushes, domain randomization)
##

@configclass
class RobustEventsCfg(EventsCfg):
    """Disturbances and domain randomization for the robust tasks."""

    # === Startup behaviors ===
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.3),
            "dynamic_friction_range": (0.3, 1.3),
            "restitution_range": (0.0, 0.1),
            "num_buckets": 64,
        },
        mode="startup",
    )
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 2.5),
            "operation": "add",
        },
        mode="startup",
    )
    base_com = EventTerm(
        func=mdp.randomize_body_com_offset,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "com_range": {"x": (-0.04, 0.04), "y": (-0.03, 0.03), "z": (-0.04, 0.04)},
        },
        mode="startup",
    )
    hand_payload = EventTerm(
        func=mdp.randomize_hand_payload,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=HAND_BODIES),
            "mass_range": (0.0, 1.5),
            "mode_probs": (0.25, 0.45, 0.30),  # none / single hand / both hands
            "payload_radius": 0.05,
            "buffer_name": "hand_payload",
        },
        mode="startup",
    )
    hand_com = EventTerm(
        func=mdp.randomize_body_com_offset,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=HAND_BODIES),
            "com_range": {"x": (-0.03, 0.03), "y": (-0.03, 0.03), "z": (-0.05, 0.02)},
        },
        mode="startup",
    )
    joint_parameters = EventTerm(
        func=mdp.randomize_joint_parameters,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "friction_distribution_params": (0.0, 0.05),
            "armature_distribution_params": (0.001, 0.004),
            "operation": "abs",
        },
        mode="startup",
    )

    # === Reset behaviors ===
    base_external_force_torque = EventTerm(
        func=mdp.apply_curriculum_external_wrench,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (-15.0, 15.0),
            "torque_range": (-2.0, 2.0),
            "active_prob": 0.6,
            "buffer_name": "base_wrench",
        },
        mode="reset",
    )
    hand_external_wrench_reset = EventTerm(
        func=mdp.apply_curriculum_external_wrench,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=HAND_BODIES),
            "force_range": {"x": (-15.0, 15.0), "y": (-15.0, 15.0), "z": (-25.0, 10.0)},
            "torque_range": (-3.0, 3.0),
            "active_prob": 0.5,
            "buffer_name": "hand_wrench",
        },
        mode="reset",
    )

    # === Interval behaviors ===
    hand_external_wrench = EventTerm(
        func=mdp.apply_curriculum_external_wrench,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=HAND_BODIES),
            "force_range": {"x": (-15.0, 15.0), "y": (-15.0, 15.0), "z": (-25.0, 10.0)},
            "torque_range": (-3.0, 3.0),
            "active_prob": 0.7,
            "buffer_name": "hand_wrench",
        },
        mode="interval",
        interval_range_s=(2.0, 5.0),
    )
    elbow_external_wrench = EventTerm(
        func=mdp.apply_curriculum_external_wrench,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=ELBOW_BODIES),
            "force_range": (-10.0, 10.0),
            "torque_range": (-1.0, 1.0),
            "active_prob": 0.5,
            "buffer_name": "elbow_wrench",
        },
        mode="interval",
        interval_range_s=(3.0, 6.0),
    )
    base_external_wrench = EventTerm(
        func=mdp.apply_curriculum_external_wrench,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (-15.0, 15.0),
            "torque_range": (-2.0, 2.0),
            "active_prob": 0.6,
            "buffer_name": "base_wrench",
        },
        mode="interval",
        interval_range_s=(3.0, 6.0),
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity_curriculum,
        params={"velocity_range": {"x": (-0.8, 0.8), "y": (-0.8, 0.8), "yaw": (-0.6, 0.6)}},
        mode="interval",
        interval_range_s=(6.0, 10.0),
    )


@configclass
class LegsRobustEventsCfg(RobustEventsCfg):
    """Adds random arm motions (arms are not controlled by the policy in this task)."""

    random_arm_movements = EventTerm(
        func=mdp.drive_random_arm_movements,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINTS, preserve_order=True),
            "offset_range": (-0.6, 0.6),
            "amplitude_range": (0.0, 0.6),
            "frequency_range": (0.2, 1.5),
            "resample_interval_s": (1.5, 4.0),
            "static_prob": 0.15,
        },
        mode="interval",
        interval_range_s=(0.01, 0.02),  # < step_dt (0.04 s) so it triggers every policy step while keeping low < high
    )


##
# Rewards
##

@configclass
class RobustRewardsCfg(RewardsCfg):
    """Rewards for the 22 DoF robust task."""

    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.5)

    # let the arms help with balance instead of pinning them to the default pose
    joint_deviation_shoulder = RewTerm(
        func=mdp.joint_deviation_l1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint"])},
        weight=-0.3,
    )
    joint_deviation_elbow = RewTerm(
        func=mdp.joint_deviation_l1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_elbow_pitch_joint", ".*_elbow_roll_joint"])},
        weight=-0.3,
    )

    # robustness terms
    base_height_below_min = RewTerm(func=mdp.base_height_below_min, params={"min_height": -0.12}, weight=-20.0)
    feet_air_time_symmetry = RewTerm(
        func=mdp.feet_air_time_symmetry,
        params={"command_name": "base_velocity", "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET_BODIES)},
        weight=-0.5,
    )
    base_ang_acc_xy_l2 = RewTerm(func=mdp.base_ang_acc_xy_l2, weight=-1.0e-4)


@configclass
class LegsRobustRewardsCfg(RobustRewardsCfg):
    """Rewards for the 12 DoF legs task: arms are driven externally, so no arm penalties."""

    dof_torques_l2 = RewTerm(
        func=mdp.joint_torques_l2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
        weight=-2.0e-5,
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
        weight=-1.0e-7,
    )
    joint_deviation_shoulder = None
    joint_deviation_elbow = None


##
# Curriculum
##

@configclass
class RobustCurriculumCfg:
    """Automatic curriculum for the disturbance magnitude."""

    disturbance_scale = CurrTerm(
        func=mdp.disturbance_levels_vel,
        params={
            "reward_term_name": "track_lin_vel_xy_exp",
            "initial_scale": 0.1,
            "min_scale": 0.1,
            "max_scale": 1.0,
            "up_threshold": 0.75,
            "down_threshold": 0.40,
            "step_up": 0.05,
            "step_down": 0.025,
            "update_interval_steps": 480,
        },
    )


##
# Environments
##

@configclass
class BerkeleyHumanoidLiteRobustEnvCfg(BerkeleyHumanoidLiteEnvCfg):
    """22 DoF whole-body robust locomotion (arms + legs controlled by the policy)."""

    observations: RobustObservationsCfg = RobustObservationsCfg()
    rewards: RobustRewardsCfg = RobustRewardsCfg()
    events: RobustEventsCfg = RobustEventsCfg()
    # NOTE: Isaac Lab reads the field ``curriculum`` (singular).
    curriculum: RobustCurriculumCfg = RobustCurriculumCfg()

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 20.0


@configclass
class BerkeleyHumanoidLiteLegsRobustEnvCfg(BerkeleyHumanoidLiteRobustEnvCfg):
    """12 DoF leg policy on the full 22 DoF robot, with randomly moving arms as disturbances."""

    observations: LegsRobustObservationsCfg = LegsRobustObservationsCfg()
    rewards: LegsRobustRewardsCfg = LegsRobustRewardsCfg()
    events: LegsRobustEventsCfg = LegsRobustEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.actions.joint_pos.joint_names = LEG_JOINTS # type: ignore
