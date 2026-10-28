# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from common.paths import get_usd_path

LEG_JOINTS = [
    "leg_left_hip_roll_joint",
    "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint",
    "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint",
    "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint",
    "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint",
    "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint",
    "leg_right_ankle_roll_joint",
]

ARM_JOINTS = [
    "arm_left_shoulder_pitch_joint",
    "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint",
    "arm_left_elbow_pitch_joint",
    "arm_left_elbow_roll_joint",
    "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_pitch_joint",
    "arm_right_elbow_roll_joint",
]

HUMANOID_JOINTS = ARM_JOINTS + LEG_JOINTS

BIPED_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(get_usd_path("berkeley_humanoid_lite_biped.usd")),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "leg_left_hip_roll_joint": 0.0,
            "leg_left_hip_yaw_joint": 0.0,
            "leg_left_hip_pitch_joint": 0.0,
            "leg_left_knee_pitch_joint": 0.0,
            "leg_left_ankle_pitch_joint": 0.0,
            "leg_left_ankle_roll_joint": 0.0,
            "leg_right_hip_roll_joint": 0.0,
            "leg_right_hip_yaw_joint": 0.0,
            "leg_right_hip_pitch_joint": 0.0,
            "leg_right_knee_pitch_joint": 0.0,
            "leg_right_ankle_pitch_joint": 0.0,
            "leg_right_ankle_roll_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "leg_left_hip_roll_joint",
                "leg_left_hip_yaw_joint",
                "leg_left_hip_pitch_joint",
                "leg_left_knee_pitch_joint",
                "leg_left_ankle_pitch_joint",
                "leg_left_ankle_roll_joint",
                "leg_right_hip_roll_joint",
                "leg_right_hip_yaw_joint",
                "leg_right_hip_pitch_joint",
                "leg_right_knee_pitch_joint",
                "leg_right_ankle_pitch_joint",
                "leg_right_ankle_roll_joint",
            ],
            effort_limit=30.0,
            velocity_limit=100.0,
            stiffness=40.0,
            damping=2.0,
            armature=0.002,
        ),
    },
)

HUMANOID_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(get_usd_path("berkeley_humanoid_lite.usd")),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "arm_left_shoulder_pitch_joint": 0.0,
            "arm_left_shoulder_roll_joint": 0.0,
            "arm_left_shoulder_yaw_joint": 0.0,
            "arm_left_elbow_pitch_joint": 0.0,
            "arm_left_elbow_roll_joint": 0.0,
            "arm_right_shoulder_pitch_joint": 0.0,
            "arm_right_shoulder_roll_joint": 0.0,
            "arm_right_shoulder_yaw_joint": 0.0,
            "arm_right_elbow_pitch_joint": 0.0,
            "arm_right_elbow_roll_joint": 0.0,
            "leg_left_hip_roll_joint": 0.0,
            "leg_left_hip_yaw_joint": 0.0,
            "leg_left_hip_pitch_joint": 0.0,
            "leg_left_knee_pitch_joint": 0.0,
            "leg_left_ankle_pitch_joint": 0.0,
            "leg_left_ankle_roll_joint": 0.0,
            "leg_right_hip_roll_joint": 0.0,
            "leg_right_hip_yaw_joint": 0.0,
            "leg_right_hip_pitch_joint": 0.0,
            "leg_right_knee_pitch_joint": 0.0,
            "leg_right_ankle_pitch_joint": 0.0,
            "leg_right_ankle_roll_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                "arm_left_shoulder_pitch_joint",
                "arm_left_shoulder_roll_joint",
                "arm_left_shoulder_yaw_joint",
                "arm_left_elbow_pitch_joint",
                "arm_left_elbow_roll_joint",
                "arm_right_shoulder_pitch_joint",
                "arm_right_shoulder_roll_joint",
                "arm_right_shoulder_yaw_joint",
                "arm_right_elbow_pitch_joint",
                "arm_right_elbow_roll_joint",
            ],
            effort_limit=30.0,
            velocity_limit=100.0,
            stiffness=40.0,
            damping=2.0,
            armature=0.002,
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "leg_left_hip_roll_joint",
                "leg_left_hip_yaw_joint",
                "leg_left_hip_pitch_joint",
                "leg_left_knee_pitch_joint",
                "leg_left_ankle_pitch_joint",
                "leg_left_ankle_roll_joint",
                "leg_right_hip_roll_joint",
                "leg_right_hip_yaw_joint",
                "leg_right_hip_pitch_joint",
                "leg_right_knee_pitch_joint",
                "leg_right_ankle_pitch_joint",
                "leg_right_ankle_roll_joint",
            ],
            effort_limit=30.0,
            velocity_limit=100.0,
            stiffness=40.0,
            damping=2.0,
            armature=0.002,
        ),
    },
)

# Compatibility aliases
HUMANOID_LITE_BIPED_CFG = BIPED_CONFIG
HUMANOID_LITE_CFG = HUMANOID_CONFIG
HUMANOID_LITE_JOINTS = HUMANOID_JOINTS
HUMANOID_LITE_LEG_JOINTS = LEG_JOINTS

