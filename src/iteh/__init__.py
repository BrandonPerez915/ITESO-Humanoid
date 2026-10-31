"""
ITESO Humanoid (iteh) core package.
"""

from src.iteh.humanoid import Humanoid
from src.iteh.bimanual import Bimanual
import src.iteh.communication.core as Can
from src.iteh.communication.flags import FunctionFlag, ModeFlag, ErrorFlag, ParameterFlag

__all__ = [
    "Humanoid",
    "Bimanual",
    "Can",
    "FunctionFlag",
    "ModeFlag",
    "ErrorFlag",
    "ParameterFlag",
]

__version__ = "1.1.0"
