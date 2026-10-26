#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite / ITESO Humanoid Project Developers.

"""
MuJoCo Simulation Node with ROS 2 Interfaces.

Publishes:
  - /joint_states (sensor_msgs/msg/JointState): Current positions and velocities from MuJoCo.
  - /imu/data (sensor_msgs/msg/Imu): Base orientation quaternion and angular velocity.

Subscribes to:
  - /joint_commands (sensor_msgs/msg/JointState): Target joint positions applied via PD control.

Usage:
  python3 src/scripts/sim/mujoco_ros2_sim.py --config biped_25hz.yaml
"""

from __future__ import annotations

import sys
import time
import argparse
from typing import Any
from pathlib import Path
import numpy as np
import torch
import mujoco
import mujoco.viewer

# Standardize path bootstrap
_repo_root = Path(__file__).resolve()
while _repo_root.parent != _repo_root:
    if (_repo_root / "pyproject.toml").exists() or (_repo_root / "src" / "iteh").exists():
        break
    _repo_root = _repo_root.parent

for _path in [str(_repo_root), str(_repo_root / "src")]:
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState, Imu
    RCLPY_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    RCLPY_AVAILABLE = False
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs): pass
        def create_publisher(self, *args, **kwargs): pass
        def create_subscription(self, *args, **kwargs): pass
        def create_timer(self, *args, **kwargs): pass
        def get_logger(self): return self
        def info(self, *args): pass
        def destroy_node(self): pass
    class JointState: pass  # type: ignore
    class Imu: pass  # type: ignore
    class QoSProfile:  # type: ignore
        def __init__(self, *args: Any, **kwargs: Any) -> None: pass
    class ReliabilityPolicy: BEST_EFFORT = 1  # type: ignore
    class HistoryPolicy: KEEP_LAST = 1  # type: ignore

from iteh.control.policy.config import Cfg
from iteh.common.paths import get_mjcf_path


class MujocoRos2Sim(Node):
    def __init__(self, cfg_name: str = "biped_25hz.yaml"):
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy no está disponible en este entorno.")

        super().__init__("mujoco_sim_bridge")

        self.cfg = Cfg.load(cfg_name)
        self.num_joints = int(self.cfg.num_joints)
        self.joint_names = list(self.cfg.joints)

        # Cargar modelo MJCF
        scene_xml = get_mjcf_path(biped=(self.num_joints != 22), scene=True)
        self.mj_model = mujoco.MjModel.from_xml_path(str(scene_xml))
        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = self.cfg.physics_dt
        self.mj_viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)

        # Parámetros de control PD
        self.sensordata_dof_size = 3 * self.mj_model.nu
        self.joint_kp = torch.tensor(self.cfg.joint_kp, dtype=torch.float32)
        self.joint_kd = torch.tensor(self.cfg.joint_kd, dtype=torch.float32)
        self.effort_limits = torch.tensor(self.cfg.effort_limits, dtype=torch.float32)

        # Posiciones deseadas actuales (inicialmente la postura por defecto)
        self.target_positions = torch.tensor(self.cfg.default_joint_positions, dtype=torch.float32)
        self.name_to_idx = {name: i for i, name in enumerate(self.joint_names)}

        # Inicializar estado en MuJoCo
        self.reset()

        # Publicadores
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_joint_states = self.create_publisher(JointState, "/joint_states", sensor_qos)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", sensor_qos)

        # Suscriptor a comandos articulares
        self.sub_commands = self.create_subscription(
            JointState,
            "/joint_commands",
            self._on_joint_commands,
            10
        )

        # Timer de física y publicación (a la tasa de control)
        self.physics_substeps = max(1, int(np.round(self.cfg.policy_dt / self.cfg.physics_dt)))
        self.timer = self.create_timer(self.cfg.policy_dt, self._sim_step)

        self.get_logger().info(f"Simulador MuJoCo ROS 2 iniciado para modelo: {scene_xml.name}")

    def reset(self):
        self.mj_data.qpos[0:3] = self.cfg.default_base_position
        self.mj_data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])  # Cuaternión identidad
        self.mj_data.qpos[7:] = self.cfg.default_joint_positions
        self.mj_data.qvel[:] = 0.0
        mujoco.mj_step(self.mj_model, self.mj_data)
        self.mj_viewer.sync()

    def _on_joint_commands(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            idx = self.name_to_idx.get(name)
            if idx is not None:
                self.target_positions[idx] = float(pos)

    def _sim_step(self):
        if not self.mj_viewer.is_running():
            rclpy.shutdown()
            return

        # Aplicar pasos físicos en MuJoCo con control PD
        for _ in range(self.physics_substeps):
            cur_pos = torch.tensor(self.mj_data.sensordata[0:self.num_joints], dtype=torch.float32)
            cur_vel = torch.tensor(self.mj_data.sensordata[self.num_joints:2 * self.num_joints], dtype=torch.float32)
            torques = self.joint_kp * (self.target_positions - cur_pos) + self.joint_kd * (-cur_vel)
            torques = torch.clip(torques, -self.effort_limits, self.effort_limits)
            self.mj_data.ctrl[:] = torques.numpy()
            mujoco.mj_step(self.mj_model, self.mj_data)

        self.mj_viewer.sync()

        # Publicar /joint_states
        stamp = self.get_clock().now().to_msg()
        js_msg = JointState()
        js_msg.header.stamp = stamp
        js_msg.header.frame_id = "base_link"
        js_msg.name = self.joint_names
        js_msg.position = self.mj_data.sensordata[0:self.num_joints].tolist()
        js_msg.velocity = self.mj_data.sensordata[self.num_joints:2 * self.num_joints].tolist()
        self.pub_joint_states.publish(js_msg)

        # Publicar /imu/data
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "imu_link"

        # Quaternion en sensordata es [w, x, y, z]
        base_quat = self.mj_data.sensordata[self.sensordata_dof_size+0:self.sensordata_dof_size+4]
        imu_msg.orientation.w = float(base_quat[0])
        imu_msg.orientation.x = float(base_quat[1])
        imu_msg.orientation.y = float(base_quat[2])
        imu_msg.orientation.z = float(base_quat[3])

        base_ang_vel = self.mj_data.sensordata[self.sensordata_dof_size+4:self.sensordata_dof_size+7]
        imu_msg.angular_velocity.x = float(base_ang_vel[0])
        imu_msg.angular_velocity.y = float(base_ang_vel[1])
        imu_msg.angular_velocity.z = float(base_ang_vel[2])

        self.pub_imu.publish(imu_msg)


def main():
    parser = argparse.ArgumentParser(description="Puente de Simulación MuJoCo <-> ROS 2")
    parser.add_argument("--config", type=str, default="biped_25hz.yaml", help="Archivo de configuración")
    args = parser.parse_args()

    if not RCLPY_AVAILABLE:
        print("[ERROR] rclpy no está instalado.")
        sys.exit(1)

    rclpy.init()
    node = MujocoRos2Sim(args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
