"""Robot-level operations (locomotion, idle, inference, kinematics, motor config read/write)."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Optional
import numpy as np

from common.logging import get_logger
from common.paths import REPO_ROOT, get_config_path, get_urdf_path
import iteh.communication.core as Can
from iteh.communication.flags import ParameterFlag

log = get_logger("ops.robot")


def _read_actuator_dict(bus: Can.Bus, joint_id: int) -> dict:
    config = {
        "position_controller": {},
        "current_controller": {},
        "powerstage": {},
        "motor": {},
        "encoder": {},
    }
    config["device_id"] = bus._read_parameter_u32(joint_id, ParameterFlag.DEVICE_ID)
    fw = bus._read_parameter_u32(joint_id, ParameterFlag.FIRMWARE_VERSION)
    config["firmware_version"] = hex(fw) if fw is not None else "0x0"
    config["watchdog_timeout"] = bus._read_parameter_u32(joint_id, ParameterFlag.WATCHDOG_TIMEOUT)
    config["fast_frame_frequency"] = bus.read_fast_frame_frequency(joint_id)

    config["position_controller"]["gear_ratio"] = bus._read_parameter_f32(
        joint_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO
    )
    config["position_controller"]["position_kp"] = bus.read_position_kp(joint_id)
    config["position_controller"]["position_ki"] = bus.read_position_ki(joint_id)
    config["position_controller"]["velocity_kp"] = bus.read_velocity_kp(joint_id)
    config["position_controller"]["velocity_ki"] = bus.read_velocity_ki(joint_id)
    config["position_controller"]["torque_limit"] = bus.read_torque_limit(joint_id)
    config["position_controller"]["velocity_limit"] = bus.read_velocity_limit(joint_id)
    config["position_controller"]["position_limit_upper"] = bus.read_position_limit_upper(joint_id)
    config["position_controller"]["position_limit_lower"] = bus.read_position_limit_lower(joint_id)
    config["position_controller"]["position_offset"] = bus.read_position_offset(joint_id)
    config["position_controller"]["torque_filter_alpha"] = bus.read_torque_filter_alpha(joint_id)

    config["current_controller"]["i_limit"] = bus.read_current_limit(joint_id)
    config["current_controller"]["i_kp"] = bus.read_current_kp(joint_id)
    config["current_controller"]["i_ki"] = bus.read_current_ki(joint_id)

    config["powerstage"]["undervoltage_threshold"] = bus._read_parameter_f32(
        joint_id, ParameterFlag.POWERSTAGE_UNDERVOLTAGE_THRESHOLD
    )
    config["powerstage"]["overvoltage_threshold"] = bus._read_parameter_f32(
        joint_id, ParameterFlag.POWERSTAGE_OVERVOLTAGE_THRESHOLD
    )
    config["powerstage"]["bus_voltage_filter_alpha"] = bus.read_bus_voltage_filter_alpha(joint_id)

    config["motor"]["pole_pairs"] = bus.read_motor_pole_pairs(joint_id)
    config["motor"]["torque_constant"] = bus.read_motor_torque_constant(joint_id)
    config["motor"]["phase_order"] = bus.read_motor_phase_order(joint_id)
    config["motor"]["max_calibration_current"] = bus.read_motor_calibration_current(joint_id)

    config["encoder"]["cpr"] = bus.read_encoder_cpr(joint_id)
    config["encoder"]["position_offset"] = bus.read_encoder_position_offset(joint_id)
    config["encoder"]["velocity_filter_alpha"] = bus.read_encoder_velocity_filter_alpha(joint_id)
    config["encoder"]["flux_offset"] = bus.read_encoder_flux_offset(joint_id)
    return config


def run_locomotion(
    config_name: str = "biped_25hz.yaml",
    remote_ip: str = "172.28.0.5",
    udp_port: int = 11000,
) -> None:
    """Run real-robot RL locomotion loop."""
    from cc.udp import UDP
    from loop_rate_limiters import RateLimiter
    from iteh import Humanoid
    from iteh.policy import PolicyConfig, RlController

    config = PolicyConfig.load(config_name=config_name)
    log.info("Starting locomotion on physical robot (rate: %.1f Hz)", 1.0 / config.policy_dt)
    udp = UDP(("0.0.0.0", udp_port), (remote_ip, udp_port))

    controller = RlController(config)
    controller.load_policy()
    rate = RateLimiter(1.0 / config.policy_dt)

    robot = Humanoid()
    robot.enter_damping()
    obs = robot.reset()

    try:
        while True:
            actions = controller.update(obs)
            obs = robot.step(actions)
            udp.send_numpy(obs)
            rate.sleep()
    except KeyboardInterrupt:
        log.info("Locomotion stopped by user.")
    finally:
        robot.stop()
        log.info("Robot stopped.")


def run_robot_idle(remote_ip: str = "172.28.0.5", udp_port: int = 11000) -> None:
    """Run real-robot idle state streaming observations over UDP."""
    from cc.udp import UDP
    from iteh import Humanoid

    robot = Humanoid()
    udp = UDP(("0.0.0.0", udp_port), (remote_ip, udp_port))
    log.info("Robot in IDLE mode (streaming observations to %s:%d)...", remote_ip, udp_port)

    try:
        while True:
            acs = np.zeros(12, dtype=np.float32)
            obs = robot.step(acs)
            udp.send_numpy(obs)
            log.info("Measured positions: %s", np.round(robot.joint_position_measured, 3))
    except KeyboardInterrupt:
        log.info("Stopping robot due to user interruption.")
    finally:
        robot.stop()


def run_inference_test(config_name: str = "humanoid.yaml") -> np.ndarray:
    """Run a single-step policy inference smoke test."""
    from iteh.policy import PolicyConfig, RlController

    cfg = PolicyConfig.load(config_name=config_name)
    controller = RlController(cfg)
    controller.load_policy()

    obs = np.zeros((7 + cfg.num_actions * 2 + 4,), dtype=np.float32)
    actions = controller.update(obs)
    log.info("Inference successful (%d actions): %s", len(actions), np.round(actions, 4))
    return actions


def read_robot_config(output_path: Optional[str] = None) -> Path:
    """Read all actuator parameters from the robot and save to JSON."""
    from iteh import Humanoid

    robot_configuration = {}
    robot = Humanoid()
    robot.check_connection()

    for bus, joint_id, joint_name in robot.joints:
        log.info("Reading configuration for %s (ID=%d)...", joint_name, joint_id)
        robot_configuration[joint_name] = _read_actuator_dict(bus, joint_id)
        time.sleep(0.1)

    out_path = Path(output_path) if output_path else (REPO_ROOT / "configs" / "hardware" / "motor.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(robot_configuration, f, indent=4)

    log.info("Configuration saved to: %s", out_path)
    robot.stop()
    return out_path


def write_robot_config(config_name: str = "motor.json", store_to_flash: bool = True, delay_t: float = 0.1) -> None:
    """Write actuator parameters from JSON configuration to all robot joints."""
    from iteh import Humanoid

    config_file = str(get_config_path(config_name))
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Motor configuration file not found: {config_file}")

    with open(config_file, "r") as f:
        robot_configuration = json.load(f)

    robot = Humanoid()

    for bus, joint_id, joint_name in robot.joints:
        result = bus.ping(joint_id)
        fw = bus._read_parameter_u32(joint_id, ParameterFlag.FIRMWARE_VERSION)
        log.info("Ping %s: %s (FW: %s)", joint_name, result, hex(fw) if fw is not None else "N/A")
        time.sleep(0.1)

    for bus, joint_id, joint_name in robot.joints:
        log.info("Writing configuration for %s...", joint_name)
        config = robot_configuration.get(joint_name)
        if not config:
            raise ValueError(f"No configuration found for {joint_name}")

        bus.write_fast_frame_frequency(joint_id, config["fast_frame_frequency"])
        time.sleep(delay_t)

        bus._write_parameter_f32(
            joint_id, ParameterFlag.POSITION_CONTROLLER_GEAR_RATIO, config["position_controller"]["gear_ratio"]
        )
        time.sleep(delay_t)

        bus.write_position_kp(joint_id, config["position_controller"]["position_kp"])
        time.sleep(delay_t)
        bus.write_position_ki(joint_id, config["position_controller"]["position_ki"])
        time.sleep(delay_t)
        bus.write_velocity_kp(joint_id, config["position_controller"]["velocity_kp"])
        time.sleep(delay_t)
        bus.write_velocity_ki(joint_id, config["position_controller"]["velocity_ki"])
        time.sleep(delay_t)
        bus.write_torque_limit(joint_id, config["position_controller"]["torque_limit"])
        time.sleep(delay_t)
        bus.write_velocity_limit(joint_id, config["position_controller"]["velocity_limit"])
        time.sleep(delay_t)
        bus.write_position_limit_lower(joint_id, config["position_controller"]["position_limit_lower"])
        time.sleep(delay_t)
        bus.write_position_limit_upper(joint_id, config["position_controller"]["position_limit_upper"])
        time.sleep(delay_t)
        bus.write_position_offset(joint_id, config["position_controller"]["position_offset"])
        time.sleep(delay_t)
        bus.write_torque_filter_alpha(joint_id, config["position_controller"]["torque_filter_alpha"])
        time.sleep(delay_t)

        bus.write_current_limit(joint_id, config["current_controller"]["i_limit"])
        time.sleep(delay_t)
        bus.write_current_kp(joint_id, config["current_controller"]["i_kp"])
        time.sleep(delay_t)
        bus.write_current_ki(joint_id, config["current_controller"]["i_ki"])
        time.sleep(delay_t)

        bus.write_motor_pole_pairs(joint_id, config["motor"]["pole_pairs"])
        time.sleep(delay_t)
        bus.write_motor_torque_constant(joint_id, config["motor"]["torque_constant"])
        time.sleep(delay_t)
        bus.write_motor_phase_order(joint_id, config["motor"]["phase_order"])
        time.sleep(delay_t)
        bus.write_motor_calibration_current(joint_id, config["motor"]["max_calibration_current"])
        time.sleep(delay_t)

        bus.write_encoder_cpr(joint_id, config["encoder"]["cpr"])
        time.sleep(delay_t)
        bus.write_encoder_position_offset(joint_id, config["encoder"]["position_offset"])
        time.sleep(delay_t)
        bus.write_encoder_velocity_filter_alpha(joint_id, config["encoder"]["velocity_filter_alpha"])
        time.sleep(delay_t)
        bus.write_encoder_flux_offset(joint_id, config["encoder"]["flux_offset"])
        time.sleep(delay_t)

        if store_to_flash:
            log.info("Saving to flash memory for %s...", joint_name)
            bus.store_settings_to_flash(joint_id)
            time.sleep(0.2)

        time.sleep(0.5)

    robot.stop()
    log.info("Motor configuration completed.")


def run_kinematic_solver(urdf_path: Optional[str] = None, rate_hz: float = 100.0) -> None:
    """Run the Pink/Pinocchio IK kinematic solver visualization."""
    from loop_rate_limiters import RateLimiter
    import meshcat_shapes
    import pink
    from pink.tasks import FrameTask
    from pink.utils import custom_configuration_vector
    from pink.visualization import start_meshcat_visualizer
    import pinocchio as pin
    import qpsolvers

    resolved_urdf = urdf_path or str(get_urdf_path(biped=False))
    urdf_package_path = os.path.dirname(resolved_urdf)
    robot = pin.RobotWrapper.BuildFromURDF(
        resolved_urdf,
        package_dirs=[urdf_package_path],
        root_joint=pin.JointModelFreeFlyer(),
    )

    init_base_position = np.array([0.0, 0.0, 0.0])
    visualizer = start_meshcat_visualizer(robot)
    viewer = visualizer.viewer

    end_effectors = ["arm_left_elbow_roll", "arm_right_elbow_roll"]
    end_effector_ids = [robot.model.getFrameId(name) for name in end_effectors]
    if any(ee_id < 0 or ee_id >= len(robot.model.frames) for ee_id in end_effector_ids):
        raise ValueError("End effector name not found in model")

    for name in end_effectors:
        meshcat_shapes.frame(viewer[name + "_target"], opacity=0.5)
        meshcat_shapes.frame(viewer[name + "_vive"], opacity=0.25)
        meshcat_shapes.frame(viewer[name], opacity=1.0)

    end_effector_tasks = [
        FrameTask(name, position_cost=1.0, orientation_cost=0.0, lm_damping=1.0)
        for name in end_effectors
    ]
    base_tasks = [FrameTask("base", position_cost=1000.0, orientation_cost=1000.0, lm_damping=1.0)]

    solver_name = "quadprog" if "quadprog" in qpsolvers.available_solvers else qpsolvers.available_solvers[0]
    last_update_time = time.perf_counter()

    q = custom_configuration_vector(
        robot,
        arm_left_shoulder_pitch_joint=0.0,
        arm_left_shoulder_roll_joint=0.0,
        arm_left_shoulder_yaw_joint=0.0,
        arm_left_elbow_pitch_joint=0.0,
        arm_left_elbow_roll_joint=0.0,
    )

    rate = RateLimiter(frequency=rate_hz, warn=False)
    log.info("Starting inverse kinematics visualization (URDF: %s)...", resolved_urdf)

    try:
        while True:
            delta_poses = [pin.SE3.Identity() for _ in range(len(end_effectors))]
            delta_poses[0].translation[0] = 0.2 + 0.1 * np.cos(2.0 * time.perf_counter())
            delta_poses[0].translation[1] = 0.2 + 0.1 * np.sin(2.0 * time.perf_counter())
            delta_poses[0].translation[2] = 0.6

            delta_poses[1].translation[0] = 0.2 + 0.1 * np.sin(2.0 * time.perf_counter())
            delta_poses[1].translation[1] = -0.2 - 0.1 * np.cos(2.0 * time.perf_counter())
            delta_poses[1].translation[2] = 0.6

            configuration = pink.Configuration(robot.model, robot.data, q)
            for i, name in enumerate(end_effectors):
                viewer[name + "_target"].set_transform(delta_poses[i].np)
                viewer[name].set_transform(
                    configuration.get_transform_frame_to_world(end_effector_tasks[i].frame).np
                )
                end_effector_tasks[i].transform_target_to_world = delta_poses[i]

            base_tasks[0].transform_target_to_world = pin.SE3(pin.Quaternion.Identity(), init_base_position)
            tasks = end_effector_tasks + base_tasks

            dt = time.perf_counter() - last_update_time
            velocity = pink.solve_ik(configuration, tasks, dt, solver=solver_name, safety_break=False)
            configuration.integrate_inplace(velocity, dt)

            visualizer.display(configuration.q)
            q = configuration.q
            rate.sleep()
    except KeyboardInterrupt:
        log.info("Kinematics visualization stopped.")
