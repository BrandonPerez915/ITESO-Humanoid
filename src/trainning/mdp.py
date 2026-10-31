"""This sub-module contains the functions that are specific to the locomotion environments."""

from isaaclab.envs.mdp import *

from trainning.definitions.curriculums import *
from trainning.definitions.events import *
from trainning.definitions.observations import *
from trainning.definitions.rewards import *
from trainning.definitions.terminations import *

def __getattr__(name: str):
    """Dynamic fallback for IsaacLab built-in terms when running without Isaac Sim."""
    def _stub(*args, **kwargs):
        return None
    return _stub
