"""
ITESO Humanoid (iteh) core package.
"""

from iteh.humanoid import Humanoid
from iteh.bimanual import Bimanual
from iteh.communication import core as Can
from iteh.communication import flags

__version__ = "1.1.0"
__all__ = ["Humanoid", "Bimanual", "Can", "flags"]
