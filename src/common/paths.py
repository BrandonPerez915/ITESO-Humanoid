"""
paths.py - Centralized repository path resolution for ITESO Humanoid.
"""

import os
import sys
from pathlib import Path

# Repository root (where assets, configs, models, logs, src live)
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"

# Ensure both SRC_DIR and REPO_ROOT are importable regardless of how Python was invoked
for path in (str(SRC_DIR), str(REPO_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

ASSETS_DIR = REPO_ROOT / "assets"
MODELS_DIR = REPO_ROOT / "models"
CONFIGS_DIR = REPO_ROOT / "configs"
LOGS_DIR = Path(os.environ.get("ITEH_LOG_DIR", str(REPO_ROOT / "logs")))


def get_logs_dir(*subpaths: str) -> Path:
    """Return path inside the centralized logs directory (<REPO_ROOT>/logs)."""
    return LOGS_DIR.joinpath(*subpaths)


def get_asset_path(*subpaths: str) -> Path:
    """Return absolute path within assets directory."""
    return ASSETS_DIR.joinpath(*subpaths)


def get_mjcf_path(biped: bool = False, scene: bool = True) -> Path:
    """Return path to MJCF XML file."""
    if biped:
        filename = "bhl_biped_scene.xml" if scene else "berkeley_humanoid_lite_biped.xml"
    else:
        filename = "bhl_scene.xml" if scene else "berkeley_humanoid_lite.xml"
    return ASSETS_DIR / "mjcf" / filename


def get_urdf_path(biped: bool = False) -> Path:
    """Return path to URDF file."""
    filename = "berkeley_humanoid_lite_biped.urdf" if biped else "berkeley_humanoid_lite.urdf"
    return ASSETS_DIR / "urdf" / filename


def get_usd_path(filename: str) -> Path:
    """Return path to USD file."""
    custom_path = os.environ.get("ISAACLAB_ASSET_DIR")
    if custom_path:
        candidate = Path(custom_path) / "usd" / filename
        if candidate.exists():
            return candidate
    return ASSETS_DIR / "usd" / filename


def get_model_path(model_name: str) -> Path:
    """
    Find and return path to a trained policy checkpoint (ONNX or PT).
    Supports direct path, filename in models/, or legacy checkpoints path.
    """
    path_obj = Path(model_name)
    if path_obj.is_absolute() and path_obj.exists():
        return path_obj

    # Check relative to repo root
    direct_repo = REPO_ROOT / model_name
    if direct_repo.exists():
        return direct_repo

    # Check in models/
    candidate_model = MODELS_DIR / path_obj.name
    if candidate_model.exists():
        return candidate_model

    # Legacy model name aliases (e.g., policy_biped_25hz_a.onnx -> biped_25hz.onnx)
    legacy_model_map = {
        "policy_biped_25hz_a.onnx": "biped_25hz.onnx",
        "policy_biped_50hz.onnx": "biped_50hz.onnx",
        "policy_humanoid.onnx": "humanoid.onnx",
        "policy_humanoid_legs.onnx": "humanoid_legs.onnx",
        "policy_video.onnx": "video.onnx",
    }
    if path_obj.name in legacy_model_map:
        mapped_model = MODELS_DIR / legacy_model_map[path_obj.name]
        if mapped_model.exists():
            return mapped_model

    # Check in configs/policies (legacy)
    candidate_legacy = CONFIGS_DIR / "policies" / path_obj.name
    if candidate_legacy.exists():
        return candidate_legacy

    return MODELS_DIR / path_obj.name


def get_config_path(config_name: str) -> Path:
    """Find and return path to configuration YAML or JSON."""
    path_obj = Path(config_name)
    if path_obj.is_absolute() and path_obj.exists():
        return path_obj

    direct_repo = REPO_ROOT / config_name
    if direct_repo.exists():
        return direct_repo

    candidate_configs = CONFIGS_DIR / config_name
    if candidate_configs.exists():
        return candidate_configs

    for subdir in ("", "policies", "hardware"):
        base_dir = CONFIGS_DIR / subdir if subdir else CONFIGS_DIR
        candidate = base_dir / path_obj.name
        if candidate.exists():
            return candidate

    # Legacy config name aliases (e.g., policy_biped_25hz_a.yaml -> policies/biped_25hz.yaml)
    legacy_config_map = {
        "policy_biped_25hz_a.yaml": "policies/biped_25hz.yaml",
        "policy_biped_50hz.yaml": "policies/biped_50hz.yaml",
        "policy_humanoid.yaml": "policies/humanoid.yaml",
        "policy_humanoid_legs.yaml": "policies/humanoid_legs.yaml",
        "policy_video.yaml": "policies/video.yaml",
        "motor_configuration.json": "hardware/motor.json",
        "motor.json": "hardware/motor.json",
        "calibration.yaml": "hardware/calibration.yaml",
    }
    if path_obj.name in legacy_config_map:
        mapped_cfg = CONFIGS_DIR / legacy_config_map[path_obj.name]
        if mapped_cfg.exists():
            return mapped_cfg

    return CONFIGS_DIR / "policies" / path_obj.name
