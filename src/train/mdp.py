"""This sub-module contains the functions that are specific to the locomotion environments."""

from isaaclab.envs.mdp import *

from src.train.curriculums import *
from src.train.events import *
from src.train.observations import *
from src.train.rewards import *
from src.train.terminations import *

def __getattr__(name: str):
    """Dynamic fallback for IsaacLab built-in terms when running without Isaac Sim."""
    def _stub(*args, **kwargs):
        return None
    return _stub
