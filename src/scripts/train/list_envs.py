"""
Script to print all the available environments in the extension.

The script iterates over all registered environments and stores the details in a table.
It prints the name of the environment, the entry point and the config file.
"""

"""Launch Isaac Sim Simulator first."""

from isaaclab.app import AppLauncher

# launch omniverse app
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app


import sys
from pathlib import Path

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

import gymnasium as gym
from prettytable import PrettyTable

# Import extensions to set up environment tasks
import iteh.tasks  # noqa: F401


def main():
    """Print all environments registered in `isaac.lab_demo` extension."""
    # print all the available environments
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available Environments in Isaac Lab Template Extension"
    # set alignment of table columns
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"

    # count of environments
    index = 0
    # acquire all Isaac environments names
    for task_spec in gym.registry.values():
        if "Template-" in task_spec.id:
            # add details to table
            table.add_row([index + 1, task_spec.id, task_spec.entry_point, task_spec.kwargs["env_cfg_entry_point"]])
            # increment count
            index += 1

    print(table)


if __name__ == "__main__":
    try:
        # run the main function
        main()
    except Exception as e:
        raise e
    finally:
        # close the app
        simulation_app.close()
