"""
paths.py - Centralized repository path resolution for Berkeley Humanoid Lite.
"""

import os
from pathlib import Path
from typing import Union

# Repository root (where assets, configs, models, src live)
REPO_ROOT = Path(__file__).resolve().parents[3]

ASSETS_DIR = REPO_ROOT / "assets"
MODELS_DIR = REPO_ROOT / "models"
CONFIGS_DIR = REPO_ROOT / "configs"
SCRIPTS_DIR = REPO_ROOT / "scripts"


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
    custom_dir = os.environ.get("ISAACLAB_ASSET_DIR")
    if custom_dir:
        candidate = Path(custom_dir) / "usd" / filename
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

    candidate_configs = CONFIGS_DIR / path_obj.name
    if candidate_configs.exists():
        return candidate_configs

    return CONFIGS_DIR / path_obj.name
