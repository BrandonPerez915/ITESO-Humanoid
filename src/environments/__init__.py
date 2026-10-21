from .mujoco import *

try:
    from iteh.control.policy.config import Cfg
except ImportError:
    try:
        from src.control.policy.config import Cfg
    except ImportError:
        try:
            from control.policy.config import Cfg
        except ImportError:
            pass
