"""
macOS trampoline helper for MuJoCo passive viewer.

On macOS (Darwin), MuJoCo's interactive GUI viewer (Cocoa / AppKit) requires
execution on the main thread via the `mjpython` trampoline binary.
This module automatically detects macOS, checks if running under `mjpython`,
and transparently re-executes the command under `mjpython` so that commands like:
    iteh sim play --config biped_25hz.yaml
work directly without manual `mjpython` invocation.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import sys
from typing import Optional

from common.logging import get_logger, shutdown_logging
from common.paths import REPO_ROOT, SRC_DIR

log = get_logger("sim.mjpython")


def find_mjpython() -> Optional[str]:
    """Find the path to the `mjpython` executable on macOS."""
    candidates = [
        Path(sys.executable).parent / "mjpython",
        Path(sys.prefix) / "bin" / "mjpython",
        REPO_ROOT / ".venv" / "bin" / "mjpython",
    ]

    which_path = shutil.which("mjpython")
    if which_path:
        candidates.insert(0, Path(which_path))

    # Also check inside mujoco package bundle if available
    try:
        import mujoco

        mujoco_dir = Path(mujoco.__file__).parent
        app_bin = mujoco_dir / "MuJoCo_(mjpython).app" / "Contents" / "MacOS" / "mjpython"
        if app_bin.exists():
            candidates.append(app_bin)
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate.resolve())

    return None


def is_running_under_mjpython() -> bool:
    """Return True if already running under mjpython on macOS, or if not on macOS."""
    if sys.platform != "darwin":
        return True

    if os.environ.get("_ITEH_MJPYTHON_ACTIVE") == "1":
        return True

    try:
        import mujoco.viewer

        mjpy = getattr(mujoco.viewer, "_MJPYTHON", None)
        base = getattr(mujoco.viewer, "_MjPythonBase", object)
        if isinstance(mjpy, base):
            return True
    except Exception:
        pass

    exe_name = Path(sys.executable).name.lower()
    return "mjpython" in exe_name


def _get_relaunch_argv(mjpython_path: str, sys_argv: list[str]) -> list[str]:
    """Reconstruct argv to pass to mjpython."""
    # If invoked as `python -m iteh.cli ...`
    if "-m" in sys_argv and "iteh.cli" in sys_argv:
        idx = sys_argv.index("-m")
        return [mjpython_path, *sys_argv[idx:]]

    # If invoked via the console script entry point (e.g. `iteh sim play ...`)
    if sys_argv and (sys_argv[0].endswith("iteh") or sys_argv[0].endswith("__main__.py")):
        return [mjpython_path, "-m", "iteh.cli", *sys_argv[1:]]

    # Fallback: if arguments contain "sim" or "play", run through iteh.cli module
    if any(arg in ("sim", "play", "visualize") for arg in sys_argv):
        return [mjpython_path, "-m", "iteh.cli", *sys_argv[1:]]

    # Otherwise standalone script
    return [mjpython_path, *sys_argv]


def ensure_mjpython() -> None:
    """
    Ensure the process is running under `mjpython` on macOS.

    If running on macOS and not under `mjpython`, automatically re-executes
    the current command using `mjpython` in-place.
    """
    if sys.platform != "darwin":
        return

    if is_running_under_mjpython():
        return

    if os.environ.get("_ITEH_MJPYTHON_RELAUNCHED") == "1":
        # Avoid recursion if relaunch was already attempted
        return

    mjpython_path = find_mjpython()
    if not mjpython_path:
        log.warning(
            "macOS detected but 'mjpython' executable was not found. "
            "MuJoCo passive viewer requires 'mjpython' on macOS and may fail with a RuntimeError. "
            "Ensure 'mujoco' is installed in your Python environment or run: "
            "mjpython -m iteh.cli sim ..."
        )
        return

    log.info("macOS detected: relaunching via '%s' for MuJoCo GUI viewer support...", mjpython_path)

    new_argv = _get_relaunch_argv(mjpython_path, sys.argv)

    new_env = dict(os.environ)
    new_env["_ITEH_MJPYTHON_RELAUNCHED"] = "1"
    new_env["_ITEH_MJPYTHON_ACTIVE"] = "1"
    existing_pythonpath = new_env.get("PYTHONPATH", "")
    new_env["PYTHONPATH"] = f"{SRC_DIR}:{REPO_ROOT}:{existing_pythonpath}".strip(":")

    shutdown_logging()

    sys.stdout.flush()
    sys.stderr.flush()

    os.execve(mjpython_path, new_argv, new_env)
