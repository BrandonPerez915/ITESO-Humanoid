"""
Centralized non-blocking logging system for ITESO Humanoid (`iteh` CLI).

Behavior:
  - Console logging is always active (`INFO` by default, configurable via `--log-level`, `-v`, `-q`).
  - File logging to `<REPO_ROOT>/logs/<group>/<command>/<timestamp>/` is ONLY enabled
    when the `--log` flag is passed on the CLI.
  - Uses standard library `QueueHandler` + `QueueListener` so disk I/O never blocks
    high-frequency control loops (1 kHz PD, 25-50 Hz RL policy, ROS 2 timers).
"""

from __future__ import annotations

import atexit
from datetime import datetime
import json
import logging
from logging.handlers import QueueHandler, QueueListener
import os
from pathlib import Path
import platform
import queue
import socket
import subprocess
import sys
import time
from typing import Any, Optional

from common.paths import LOGS_DIR, REPO_ROOT

_ROOT_LOGGER_NAME = "iteh"
_queue_listener: Optional[QueueListener] = None
_current_run_dir: Optional[Path] = None
_metadata_path: Optional[Path] = None
_start_time: float = 0.0
_exit_code: int = 0
_metadata_cache: dict[str, Any] = {}


class _ColorConsoleFormatter(logging.Formatter):
    """Compact ANSI-colored console formatter."""

    COLORS = {
        logging.DEBUG: "\033[36m",     # Cyan
        logging.INFO: "\033[32m",      # Green
        logging.WARNING: "\033[33m",   # Yellow
        logging.ERROR: "\033[31m",     # Red
        logging.CRITICAL: "\033[1;31m",
    }
    RESET = "\033[0m"

    def __init__(self, use_color: bool = True) -> None:
        super().__init__(fmt="%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%H:%M:%S")
        self.use_color = use_color and sys.stderr.isatty()

    def format(self, record: logging.LogRecord) -> str:
        msg = super().format(record)
        if not self.use_color:
            return msg
        color = self.COLORS.get(record.levelno, "")
        return f"{color}{msg}{self.RESET}" if color else msg


class _JsonLinesFormatter(logging.Formatter):
    """Structured JSONL formatter for machine-readable log analysis."""

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "timestamp": datetime.fromtimestamp(record.created).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "thread": record.threadName,
            "module": record.module,
            "line": record.lineno,
            "message": record.getMessage(),
        }
        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False)


def _get_git_revision() -> Optional[str]:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(REPO_ROOT),
            stderr=subprocess.DEVNULL,
            timeout=1.5,
        )
        return out.decode("utf-8").strip()
    except Exception:
        return None


def setup_logging(
    command: str = "cli",
    enabled: bool = False,
    level: str = "INFO",
    log_dir: Optional[Path | str] = None,
    argv: Optional[list[str]] = None,
) -> Optional[Path]:
    """
    Configure the centralized `iteh` logger.

    Args:
        command: Dot or slash-separated command path (e.g. `"train.play"`, `"sim.play"`).
        enabled: If True (`--log` passed), creates a timestamped run directory under `logs/`
                 and writes `iteh.log`, `events.jsonl`, and `metadata.json`.
        level: Logging level name (`DEBUG`, `INFO`, `WARNING`, `ERROR`).
        log_dir: Optional override for the base `logs/` directory.
        argv: Command-line arguments list for metadata recording.

    Returns:
        Path to the created run log directory if `enabled=True`, otherwise `None`.
    """
    global _queue_listener, _current_run_dir, _metadata_path, _start_time, _metadata_cache

    shutdown_logging()

    numeric_level = getattr(logging, level.upper(), logging.INFO)
    root_logger = logging.getLogger(_ROOT_LOGGER_NAME)
    root_logger.setLevel(min(numeric_level, logging.DEBUG if enabled else numeric_level))
    root_logger.handlers.clear()
    root_logger.propagate = False

    # 1. Console Handler (always active)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(_ColorConsoleFormatter(use_color=True))
    root_logger.addHandler(console_handler)

    if not enabled:
        _current_run_dir = None
        _metadata_path = None
        return None

    # 2. File Logging (ONLY when --log is enabled)
    base_logs = Path(log_dir) if log_dir is not None else LOGS_DIR
    cmd_parts = [p for p in command.replace(".", "/").split("/") if p]
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_dir = base_logs.joinpath(*cmd_parts, timestamp)
    run_dir.mkdir(parents=True, exist_ok=True)
    _current_run_dir = run_dir
    _start_time = time.time()

    # Configure ROS 2 log redirection if a ROS 2 node is started in this process
    ros_log_dir = run_dir / "ros2"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)

    # Create human-readable and JSONL file handlers behind a non-blocking QueueListener
    text_file_path = run_dir / "iteh.log"
    jsonl_file_path = run_dir / "events.jsonl"

    text_handler = logging.FileHandler(text_file_path, encoding="utf-8")
    text_handler.setLevel(logging.DEBUG)
    text_handler.setFormatter(
        logging.Formatter(
            fmt="%(asctime)s | %(levelname)-8s | %(threadName)-12s | %(name)s:%(lineno)d | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
    )

    jsonl_handler = logging.FileHandler(jsonl_file_path, encoding="utf-8")
    jsonl_handler.setLevel(logging.DEBUG)
    jsonl_handler.setFormatter(_JsonLinesFormatter())

    log_queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
    queue_handler = QueueHandler(log_queue)
    queue_handler.setLevel(logging.DEBUG)
    root_logger.addHandler(queue_handler)

    _queue_listener = QueueListener(log_queue, text_handler, jsonl_handler, respect_handler_level=True)
    _queue_listener.start()

    # Capture Python warnings & uncaught exceptions into log file
    logging.captureWarnings(True)
    warnings_logger = logging.getLogger("py.warnings")
    warnings_logger.handlers.clear()
    warnings_logger.addHandler(queue_handler)

    _orig_excepthook = sys.excepthook

    def _exception_hook(exc_type, exc_value, exc_tb):
        if issubclass(exc_type, KeyboardInterrupt):
            root_logger.info("Execution interrupted by user (KeyboardInterrupt).")
        else:
            root_logger.critical("Uncaught exception:", exc_info=(exc_type, exc_value, exc_tb))
        _orig_excepthook(exc_type, exc_value, exc_tb)

    sys.excepthook = _exception_hook

    # Write initial run metadata
    _metadata_path = run_dir / "metadata.json"
    _metadata_cache = {
        "command": command,
        "argv": argv if argv is not None else sys.argv,
        "start_time": datetime.fromtimestamp(_start_time).isoformat(),
        "end_time": None,
        "duration_sec": None,
        "exit_code": None,
        "host": socket.gethostname(),
        "platform": platform.platform(),
        "python_version": platform.python_version(),
        "cwd": os.getcwd(),
        "git_commit": _get_git_revision(),
        "log_file": str(text_file_path),
        "events_file": str(jsonl_file_path),
    }
    _write_metadata()

    atexit.register(shutdown_logging)
    root_logger.info("Logging enabled -> %s", run_dir)
    return run_dir


def _write_metadata() -> None:
    if _metadata_path is None:
        return
    try:
        with open(_metadata_path, "w", encoding="utf-8") as f:
            json.dump(_metadata_cache, f, indent=2, ensure_ascii=False)
    except Exception:
        pass


def set_exit_code(code: int) -> None:
    """Record the exit code for metadata.json before shutdown."""
    global _exit_code
    _exit_code = int(code)


def shutdown_logging() -> None:
    """Flush queued log records, stop QueueListener, and finalize metadata.json."""
    global _queue_listener, _metadata_path
    if _metadata_path is not None and _start_time > 0:
        end_t = time.time()
        _metadata_cache["end_time"] = datetime.fromtimestamp(end_t).isoformat()
        _metadata_cache["duration_sec"] = round(end_t - _start_time, 3)
        _metadata_cache["exit_code"] = _exit_code
        _write_metadata()
        _metadata_path = None

    if _queue_listener is not None:
        try:
            _queue_listener.stop()
        except Exception:
            pass
        _queue_listener = None


def get_logger(name: str = "core") -> logging.Logger:
    """Return a child logger under the `iteh` hierarchy."""
    if name == _ROOT_LOGGER_NAME or name.startswith(f"{_ROOT_LOGGER_NAME}."):
        return logging.getLogger(name)
    return logging.getLogger(f"{_ROOT_LOGGER_NAME}.{name}")


def get_run_dir() -> Optional[Path]:
    """Return the active run directory under `logs/` if `--log` was enabled, else `None`."""
    return _current_run_dir


def status_line(message: str) -> None:
    """Print an in-place status line to stdout without polluting the file log."""
    sys.stdout.write(f"\r{message}    ")
    sys.stdout.flush()
