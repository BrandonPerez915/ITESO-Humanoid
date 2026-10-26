"""
paths.py - Centralized repository path resolution for ITESO Humanoid (iteh).

Standardizes all filesystem paths (repository root, assets, configs, models,
scripts, URDF, MJCF, USD, meshes) and provides helpers to resolve resources
across the project.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Union


def _find_repo_root() -> Path:
    """Locate the repository root directory by walking up from this file."""
    current = Path(__file__).resolve().parent
    for parent in [current] + list(current.parents):
        if (parent / "pyproject.toml").exists() and (parent / "src" / "iteh").exists():
            return parent
    return Path(__file__).resolve().parents[3]


# Repository root (where assets, configs, models, src live)
REPO_ROOT: Path = _find_repo_root()

# Primary directories
SRC_DIR: Path = REPO_ROOT / "src"
ITEH_DIR: Path = SRC_DIR / "iteh"
ASSETS_DIR: Path = REPO_ROOT / "assets"
CONFIGS_DIR: Path = REPO_ROOT / "configs"
MODELS_DIR: Path = REPO_ROOT / "models"
SCRIPTS_DIR: Path = SRC_DIR / "scripts"

# Asset subdirectories
URDF_DIR: Path = ASSETS_DIR / "urdf"
MJCF_DIR: Path = ASSETS_DIR / "mjcf"
USD_DIR: Path = ASSETS_DIR / "usd"
MESHES_DIR: Path = ASSETS_DIR / "meshes"
SCAD_DIR: Path = ASSETS_DIR / "scad"

# Config subdirectories
POLICIES_CONFIG_DIR: Path = CONFIGS_DIR / "policies"
HARDWARE_CONFIG_DIR: Path = CONFIGS_DIR / "hardware"


def setup_sys_path() -> None:
    """Ensure project root and src/ directory are present at the head of sys.path."""
    for p in [str(REPO_ROOT), str(SRC_DIR)]:
        if p not in sys.path:
            sys.path.insert(0, p)

def get_repo_root() -> Path:
    """Return the absolute Path to the repository root."""
    return REPO_ROOT

def get_src_dir() -> Path:
    """Return the absolute Path to the src/ directory."""
    return SRC_DIR

def get_assets_dir() -> Path:
    """Return the absolute Path to the assets/ directory."""
    return ASSETS_DIR

def get_configs_dir() -> Path:
    """Return the absolute Path to the configs/ directory."""
    return CONFIGS_DIR

def get_models_dir() -> Path:
    """Return the absolute Path to the models/ directory."""
    return MODELS_DIR

def get_scripts_dir() -> Path:
    """Return the absolute Path to the scripts/ directory."""
    return SCRIPTS_DIR

def get_asset_path(*subpaths: Union[str, Path]) -> Path:
    """Return an absolute Path within the assets directory."""
    return ASSETS_DIR.joinpath(*(str(p) for p in subpaths))

def get_urdf_path(biped: bool = False, filename: Union[str, Path, None] = None) -> Path:
    """
    Return path to a URDF file.

    Args:
        biped: If True and filename is None, returns the biped URDF.
        filename: Optional explicit file name or path.
    """
    if filename is not None:
        p = Path(filename)
        if p.is_absolute() and p.exists():
            return p
        if (URDF_DIR / p.name).exists():
            return URDF_DIR / p.name
        if (ASSETS_DIR / p).exists():
            return ASSETS_DIR / p
        if (REPO_ROOT / p).exists():
            return REPO_ROOT / p
        return URDF_DIR / p.name

    name = "berkeley_humanoid_lite_biped.urdf" if biped else "berkeley_humanoid_lite.urdf"
    return URDF_DIR / name

def get_mjcf_path(biped: bool = False, scene: bool = True, filename: Union[str, Path, None] = None) -> Path:
    """
    Return path to an MJCF XML file.

    Args:
        biped: If True, resolves biped XML.
        scene: If True, resolves scene XML with ground plane and lighting.
        filename: Optional explicit file name or path.
    """
    if filename is not None:
        p = Path(filename)
        if p.is_absolute() and p.exists():
            return p
        if (MJCF_DIR / p.name).exists():
            return MJCF_DIR / p.name
        if (ASSETS_DIR / p).exists():
            return ASSETS_DIR / p
        if (REPO_ROOT / p).exists():
            return REPO_ROOT / p
        return MJCF_DIR / p.name

    if biped:
        name = "bhl_biped_scene.xml" if scene else "berkeley_humanoid_lite_biped.xml"
    else:
        name = "bhl_scene.xml" if scene else "berkeley_humanoid_lite.xml"
    return MJCF_DIR / name

def get_usd_path(filename: Union[str, Path]) -> Path:
    """Return path to a USD file, checking ISAACLAB_ASSET_DIR override first."""
    p = Path(filename)
    if p.is_absolute() and p.exists():
        return p

    custom_dir = os.environ.get("ISAACLAB_ASSET_DIR")
    if custom_dir:
        candidate = Path(custom_dir) / "usd" / p.name
        if candidate.exists():
            return candidate

    candidate_direct = USD_DIR / p.name
    if candidate_direct.exists():
        return candidate_direct

    return USD_DIR / p.name

def get_model_path(model_name: Union[str, Path]) -> Path:
    """
    Find and return path to a trained policy checkpoint (.onnx or .pt).
    Supports direct paths, filename with/without extension, relative paths,
    and handles 'policy_' prefix and version suffixes.
    """
    p = Path(model_name)
    if p.is_absolute() and p.exists():
        return p

    # Direct relative to repository root
    direct_repo = REPO_ROOT / p
    if direct_repo.exists():
        return direct_repo

    # Check directly inside models/
    candidate_model = MODELS_DIR / p.name
    if candidate_model.exists():
        return candidate_model

    # Check stripping 'policy_' prefix
    name_str = p.name
    if name_str.startswith("policy_"):
        stripped = name_str[len("policy_"):]
        cand_stripped = MODELS_DIR / stripped
        if cand_stripped.exists():
            return cand_stripped
        # Handle suffix like '_a.onnx' -> '.onnx'
        if "_a." in stripped:
            cand_no_a = MODELS_DIR / stripped.replace("_a.", ".")
            if cand_no_a.exists():
                return cand_no_a

    # Check with common extensions if missing
    for ext in [".onnx", ".pt"]:
        cand = MODELS_DIR / f"{name_str}{ext}"
        if cand.exists():
            return cand
        if name_str.startswith("policy_"):
            cand_s = MODELS_DIR / f"{name_str[len('policy_'):]}{ext}"
            if cand_s.exists():
                return cand_s

    # Check in configs/policies (legacy checkpoints location)
    candidate_legacy = POLICIES_CONFIG_DIR / p.name
    if candidate_legacy.exists():
        return candidate_legacy

    return MODELS_DIR / p.name


def get_config_path(config_name: Union[str, Path]) -> Path:
    """
    Find and return path to configuration YAML or JSON.
    Searches configs/, configs/policies/, configs/hardware/, and repo root.
    Handles legacy names such as policy_humanoid.yaml -> humanoid.yaml.
    """
    p = Path(config_name)
    if p.is_absolute() and p.exists():
        return p

    # Direct relative to repo root
    direct_repo = REPO_ROOT / p
    if direct_repo.exists():
        return direct_repo

    # Check configs/
    direct_config = CONFIGS_DIR / p
    if direct_config.exists():
        return direct_config

    cand_configs = CONFIGS_DIR / p.name
    if cand_configs.exists():
        return cand_configs

    # Check configs/policies/
    cand_policy = POLICIES_CONFIG_DIR / p.name
    if cand_policy.exists():
        return cand_policy

    # Check configs/hardware/
    cand_hw = HARDWARE_CONFIG_DIR / p.name
    if cand_hw.exists():
        return cand_hw

    # Handle legacy prefixes: "policy_humanoid.yaml" -> "humanoid.yaml"
    name_str = p.name
    if name_str.startswith("policy_"):
        stripped = name_str[len("policy_"):]
        cand_stripped = POLICIES_CONFIG_DIR / stripped
        if cand_stripped.exists():
            return cand_stripped

    # Handle missing .yaml or .json extension
    for ext in [".yaml", ".json", ".yml"]:
        cand_ext = POLICIES_CONFIG_DIR / f"{name_str}{ext}"
        if cand_ext.exists():
            return cand_ext
        cand_hw_ext = HARDWARE_CONFIG_DIR / f"{name_str}{ext}"
        if cand_hw_ext.exists():
            return cand_hw_ext

    # Fallback to policies dir or configs dir
    return POLICIES_CONFIG_DIR / p.name
