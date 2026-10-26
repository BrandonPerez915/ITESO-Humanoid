"""This sub-module contains the functions that are specific to the locomotion environments."""

from __future__ import annotations
from typing import Any

try:
    from isaaclab.envs.mdp import *  # noqa: F401, F403
except (ImportError, ModuleNotFoundError):
    pass

from .curriculums import *  # noqa: F401, F403
from .rewards import *  # noqa: F401, F403
from .events import *  # noqa: F401, F403
from .terminations import *  # noqa: F401, F403


def __getattr__(name: str) -> Any:
    """Fallback attribute access when IsaacLab is not installed."""
    return Any
