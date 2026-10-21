"""
Backwards-compatibility shim for environments.mujoco.
The canonical location is now `bhl.envs.mujoco`.
"""

from iteh.envs.mujoco import (
    MujocoEnv,
    MujocoVisualizer,
    MujocoSimulator,
    Cfg,
)
from iteh.common.math import quat_rotate_inverse
