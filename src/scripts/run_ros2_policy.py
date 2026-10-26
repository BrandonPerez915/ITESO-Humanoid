#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite / ITESO Humanoid Project Developers.

"""
Launcher script for the ROS 2 Reinforcement Learning Policy Node.

Usage:
  python3 src/scripts/run_ros2_policy.py --config biped_25hz.yaml
  python3 src/scripts/run_ros2_policy.py --ros-args -p config:=humanoid_legs.yaml -p watchdog_timeout:=1.0
"""

import sys
import argparse
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

from iteh.common.paths import REPO_ROOT, SRC_DIR, get_config_path

try:
    import rclpy
    from iteh.control.policy.ros2_policy_node import Ros2PolicyNode
except ImportError as e:
    print(f"\n[ERROR] No se pudo cargar el entorno ROS 2 o los módulos de ITESO-Humanoid: {e}")
    print("Asegúrate de:")
    print("  1. Haber activado el entorno con ROS 2 (ej. 'source /opt/ros/<distro>/setup.bash')")
    print("  2. Haber instalado las dependencias de Python del proyecto.")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Launcher del Nodo ROS 2 de Política de Control (ITESO Humanoid)")
    parser.add_argument("--config", type=str, default="biped_25hz.yaml", help="Archivo de configuración (en configs/policies/)")
    parser.add_argument("--rate", type=float, default=0.0, help="Frecuencia forzada en Hz (0.0 para auto-detectar del YAML)")
    args, ros_args = parser.parse_known_args()

    # Iniciar contexto ROS 2 pasando los argumentos remanentes de ROS 2 si existen
    rclpy.init(args=ros_args)

    # Configurar parámetros iniciales
    parameter_overrides = []
    if args.config:
        from rclpy.parameter import Parameter
        parameter_overrides.append(Parameter("config", Parameter.Type.STRING, args.config))
    if args.rate > 0.0:
        from rclpy.parameter import Parameter
        parameter_overrides.append(Parameter("policy_rate", Parameter.Type.DOUBLE, float(args.rate)))

    node = Ros2PolicyNode(parameter_overrides=parameter_overrides)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Ejecución cancelada por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[INFO] Nodo finalizado correctamente.")


if __name__ == "__main__":
    main()
