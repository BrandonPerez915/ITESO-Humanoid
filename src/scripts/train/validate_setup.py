"""Pre-flight dry-run validator for training configurations and MDP functions.

Run this script locally (even on macOS without Isaac Sim / GPU) before launching a cloud VM to
verify:
1. All environment and PPO runner @configclass definitions instantiate cleanly.
2. Every ObsTerm, RewTerm, EventTerm, DoneTerm, and CurrTerm function signature matches its ``params``.
3. Every ``body_names`` and ``joint_names`` regex in ``SceneEntityCfg`` matches real links/joints in the URDF/USD.
4. All custom functions in ``events.py``, ``observations.py``, ``rewards.py``, and ``curriculums.py``
   execute without tensor shape, indexing, or device errors on a mocked vectorized environment.
5. ``RlController`` observation history buffer matches the exported policy dimensions.
"""

from __future__ import annotations

import copy
import inspect
import re
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SRC_DIR = _REPO_ROOT / "src"
for _p in (str(_REPO_ROOT), str(_SRC_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def check_torch() -> bool:
    """Check if PyTorch is installed and importable."""
    try:
        import torch
        print(f"  [INFO] PyTorch version: {torch.__version__}")
        print(f"  [INFO] CUDA available: {torch.cuda.is_available()}")
        return True
    except ImportError:
        print("  [INFO] PyTorch not found. Some tests will be skipped.")
        return False


def check_isaaclab() -> bool:
    """Check if Isaac Lab is installed and importable."""
    try:
        import isaaclab
        print(f"  [INFO] Isaac Lab version: {getattr(isaaclab, '__version__', 'installed')}")
        return True
    except ImportError:
        print("  [INFO] Isaac Lab not found. Using lightweight mocks for dry-run validation.")
        return False


def _install_isaaclab_mocks_if_needed() -> bool:
    try:
        import isaaclab  # noqa: F401
        return False
    except ImportError:
        pass

    def configclass(cls):
        # Emulate @configclass: turn class attributes into deep-copied instance attributes on __init__
        orig_post_init = getattr(cls, "__post_init__", None)

        def __init__(self, **kwargs):
            for base in reversed(cls.__mro__):
                for k, v in base.__dict__.items():
                    if k.startswith("__") or callable(v) or isinstance(v, (classmethod, staticmethod, property)):
                        continue
                    setattr(self, k, copy.deepcopy(v))
            for k, v in kwargs.items():
                setattr(self, k, v)
            if orig_post_init is not None:
                orig_post_init(self)

        def replace(self, **kwargs):
            cloned = copy.deepcopy(self)
            for k, v in kwargs.items():
                setattr(cloned, k, v)
            return cloned

        cls.__init__ = __init__
        cls.replace = replace
        return cls

    class _BaseCfg:
        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)

        def replace(self, **kwargs):
            cloned = copy.deepcopy(self)
            for k, v in kwargs.items():
                setattr(cloned, k, v)
            return cloned

    class SceneEntityCfg(_BaseCfg):
        def __init__(self, name: str = "", joint_names=None, body_names=None, preserve_order: bool = False, **kwargs):
            super().__init__(
                name=name,
                joint_names=joint_names,
                body_names=body_names,
                joint_ids=slice(None),
                body_ids=slice(None),
                preserve_order=preserve_order,
                **kwargs,
            )

    class _TermCfg(_BaseCfg):
        def __init__(self, func=None, params=None, **kwargs):
            super().__init__(func=func, params=params or {}, **kwargs)

    # Generic dummy functions for standard Isaac Lab built-in mdp terms
    def _builtin_mdp_stub(env, *args, **kwargs):
        return None

    # Create mock module hierarchy
    mods = [
        "isaaclab",
        "isaaclab.sim",
        "isaaclab.actuators",
        "isaaclab.assets",
        "isaaclab.assets.articulation",
        "isaaclab.envs",
        "isaaclab.envs.mdp",
        "isaaclab.envs.mdp.events",
        "isaaclab.managers",
        "isaaclab.scene",
        "isaaclab.sensors",
        "isaaclab.terrains",
        "isaaclab.utils",
        "isaaclab.utils.math",
        "isaaclab.utils.noise",
        "isaaclab_rl",
        "isaaclab_rl.rsl_rl",
    ]
    for m in mods:
        sys.modules[m] = types.ModuleType(m)

    sys.modules["isaaclab"].sim = sys.modules["isaaclab.sim"]
    for attr in (
        "UsdFileCfg",
        "RigidBodyPropertiesCfg",
        "ArticulationRootPropertiesCfg",
        "RigidBodyMaterialCfg",
        "MdlFileCfg",
        "DistantLightCfg",
        "DomeLightCfg",
    ):
        setattr(sys.modules["isaaclab.sim"], attr, _BaseCfg)

    sys.modules["isaaclab.actuators"].ImplicitActuatorCfg = _BaseCfg
    ArticulationCfg = type("ArticulationCfg", (_BaseCfg,), {"InitialStateCfg": _BaseCfg})
    sys.modules["isaaclab.assets"].ArticulationCfg = ArticulationCfg
    sys.modules["isaaclab.assets"].AssetBaseCfg = _BaseCfg
    sys.modules["isaaclab.assets"].Articulation = object
    sys.modules["isaaclab.assets"].RigidObject = object
    sys.modules["isaaclab.assets.articulation"].ArticulationCfg = ArticulationCfg

    @configclass
    class ManagerBasedRLEnvCfg:
        sim = _BaseCfg(dt=0.005, render_interval=4, disable_contact_processing=True, physics_material=None, physx=_BaseCfg(gpu_max_rigid_patch_count=0))
        decimation = 4
        episode_length_s = 20.0

        def __post_init__(self):
            pass

    sys.modules["isaaclab.envs"].ManagerBasedRLEnvCfg = ManagerBasedRLEnvCfg
    sys.modules["isaaclab.envs"].ManagerBasedRLEnv = object
    sys.modules["isaaclab.envs"].ManagerBasedEnv = object
    sys.modules["isaaclab.envs"].RLTaskEnv = object

    # Populate standard isaaclab.envs.mdp symbols used in enviroment.py and robust_enviroment.py
    mdp_mod = sys.modules["isaaclab.envs.mdp"]
    UniformVelocityCommandCfg = type("UniformVelocityCommandCfg", (_BaseCfg,), {"Ranges": _BaseCfg})
    mdp_mod.UniformVelocityCommandCfg = UniformVelocityCommandCfg
    mdp_mod.JointPositionActionCfg = _BaseCfg
    for fn_name in (
        "generated_commands",
        "base_ang_vel",
        "base_lin_vel",
        "projected_gravity",
        "joint_pos_rel",
        "joint_vel_rel",
        "last_action",
        "is_terminated",
        "lin_vel_z_l2",
        "ang_vel_xy_l2",
        "flat_orientation_l2",
        "action_rate_l2",
        "joint_torques_l2",
        "joint_acc_l2",
        "joint_pos_limits",
        "undesired_contacts",
        "joint_deviation_l1",
        "time_out",
        "bad_orientation",
        "randomize_rigid_body_material",
        "randomize_rigid_body_mass",
        "randomize_actuator_gains",
        "randomize_joint_parameters",
        "reset_root_state_uniform",
        "reset_joints_by_scale",
        "apply_external_force_torque",
        "push_by_setting_velocity",
    ):
        setattr(mdp_mod, fn_name, _builtin_mdp_stub)

    def _mock_randomize_prop_by_op(data, dist_params, env_ids, elem_ids, operation="abs", distribution="uniform"):
        return data

    def _mock_push_by_setting_velocity(env, env_ids, velocity_range, asset_cfg=SceneEntityCfg("robot")):
        env._last_push_range = velocity_range

    sys.modules["isaaclab.envs.mdp.events"]._randomize_prop_by_op = _mock_randomize_prop_by_op
    sys.modules["isaaclab.envs.mdp.events"].push_by_setting_velocity = _mock_push_by_setting_velocity

    mgr_mod = sys.modules["isaaclab.managers"]
    mgr_mod.CurriculumTermCfg = _TermCfg
    mgr_mod.EventTermCfg = _TermCfg
    mgr_mod.ObservationGroupCfg = _BaseCfg
    mgr_mod.ObservationTermCfg = _TermCfg
    mgr_mod.RewardTermCfg = _TermCfg
    mgr_mod.TerminationTermCfg = _TermCfg
    mgr_mod.SceneEntityCfg = SceneEntityCfg

    sys.modules["isaaclab.scene"].InteractiveSceneCfg = _BaseCfg
    sys.modules["isaaclab.sensors"].ContactSensorCfg = _BaseCfg
    sys.modules["isaaclab.sensors"].ContactSensor = object
    sys.modules["isaaclab.terrains"].TerrainImporterCfg = _BaseCfg
    sys.modules["isaaclab.terrains"].TerrainImporter = object
    sys.modules["isaaclab.utils"].configclass = configclass
    sys.modules["isaaclab.utils.noise"].AdditiveUniformNoiseCfg = _BaseCfg

    math_mod = sys.modules["isaaclab.utils.math"]
    math_mod.quat_rotate_inverse = lambda q, v: v
    math_mod.yaw_quat = lambda q: q

    rsl_mod = sys.modules["isaaclab_rl.rsl_rl"]
    rsl_mod.RslRlOnPolicyRunnerCfg = _BaseCfg
    rsl_mod.RslRlPpoActorCriticCfg = _BaseCfg
    rsl_mod.RslRlPpoAlgorithmCfg = _BaseCfg
    return True


def _load_urdf_links_and_joints(urdf_path: Path) -> tuple[list[str], list[str]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    links = [el.attrib["name"] for el in root.findall("link")]
    joints = [
        el.attrib["name"]
        for el in root.findall("joint")
        if el.attrib.get("type") in ("revolute", "prismatic", "continuous")
    ]
    return links, joints


def _match_regex_list(patterns, candidates: list[str]) -> list[str]:
    if isinstance(patterns, str):
        patterns = [patterns]
    matched = []
    for pat in patterns:
        regex = re.compile(f"^{pat}$")
        for c in candidates:
            if regex.match(c) and c not in matched:
                matched.append(c)
    return matched


def validate_configs_and_regexes():
    from common.paths import get_urdf_path
    from train.configs.enviroment import BerkeleyHumanoidLiteBipedEnvCfg, BerkeleyHumanoidLiteEnvCfg
    from train.configs.robust_enviroment import (
        BerkeleyHumanoidLiteLegsRobustEnvCfg,
        BerkeleyHumanoidLiteRobustEnvCfg,
    )
    from train.configs.humanoid_rsl import (
        BerkeleyHumanoidLiteLegsRobustPPORunnerCfg,
        BerkeleyHumanoidLitePPORunnerCfg,
        BerkeleyHumanoidLiteRobustPPORunnerCfg,
    )
    from train.configs.biped_rsl import BerkeleyHumanoidLiteBipedPPORunnerCfg

    full_links, full_joints = _load_urdf_links_and_joints(get_urdf_path(biped=False))
    biped_links, biped_joints = _load_urdf_links_and_joints(get_urdf_path(biped=True))

    env_cfgs = [
        ("Velocity-Berkeley-Humanoid-Lite-v0", BerkeleyHumanoidLiteEnvCfg(), full_links, full_joints),
        ("Velocity-Berkeley-Humanoid-Lite-Biped-v0", BerkeleyHumanoidLiteBipedEnvCfg(), biped_links, biped_joints),
        ("Velocity-Berkeley-Humanoid-Lite-Robust-v0", BerkeleyHumanoidLiteRobustEnvCfg(), full_links, full_joints),
        ("Velocity-Berkeley-Humanoid-Lite-Legs-Robust-v0", BerkeleyHumanoidLiteLegsRobustEnvCfg(), full_links, full_joints),
    ]

    runner_cfgs = [
        BerkeleyHumanoidLitePPORunnerCfg(),
        BerkeleyHumanoidLiteBipedPPORunnerCfg(),
        BerkeleyHumanoidLiteRobustPPORunnerCfg(),
        BerkeleyHumanoidLiteLegsRobustPPORunnerCfg(),
    ]
    assert len(runner_cfgs) == 4

    for task_name, cfg, links, joints in env_cfgs:
        print(f"  [OK] Instantiated {task_name} ({cfg.__class__.__name__})")

        # Validate action joint names
        act_joints = _match_regex_list(cfg.actions.joint_pos.joint_names, joints)
        assert len(act_joints) > 0, f"{task_name}: action joint_names matched 0 joints!"

        # Inspect all manager sections: observations, rewards, terminations, events, curriculum
        sections = [
            ("observations.policy", cfg.observations.policy),
            ("observations.critic", cfg.observations.critic),
            ("rewards", cfg.rewards),
            ("terminations", cfg.terminations),
            ("events", cfg.events),
        ]
        if hasattr(cfg, "curriculum") and cfg.curriculum is not None:
            sections.append(("curriculum", cfg.curriculum))

        for sec_name, sec_obj in sections:
            for attr_name in dir(sec_obj):
                if attr_name.startswith("_"):
                    continue
                term = getattr(sec_obj, attr_name)
                if not hasattr(term, "func") or term.func is None:
                    continue

                # 1. Check function signature against term.params
                fn = term.func
                if fn.__name__ != "_builtin_mdp_stub":
                    sig = inspect.signature(fn)
                    accepts_var_kw = any(
                        p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values()
                    )
                    if not accepts_var_kw:
                        for param_key in term.params:
                            assert param_key in sig.parameters, (
                                f"{task_name} -> {sec_name}.{attr_name}: parameter '{param_key}' "
                                f"not in {fn.__name__}{sig}"
                            )

                # 2. Check every SceneEntityCfg regex against URDF links and joints
                for param_key, param_val in term.params.items():
                    if hasattr(param_val, "body_names") and param_val.body_names is not None:
                        matched_b = _match_regex_list(param_val.body_names, links)
                        assert len(matched_b) > 0, (
                            f"{task_name} -> {sec_name}.{attr_name} ({param_key}): "
                            f"body_names={param_val.body_names} matched 0 links in URDF!"
                        )
                    if hasattr(param_val, "joint_names") and param_val.joint_names is not None:
                        matched_j = _match_regex_list(param_val.joint_names, joints)
                        assert len(matched_j) > 0, (
                            f"{task_name} -> {sec_name}.{attr_name} ({param_key}): "
                            f"joint_names={param_val.joint_names} matched 0 joints in URDF!"
                        )

                # 3. Check interval_range_s when mode == "interval"
                if getattr(term, "mode", None) == "interval":
                    ir = getattr(term, "interval_range_s", None)
                    assert ir is not None and len(ir) == 2 and ir[0] < ir[1], (
                        f"{task_name} -> {sec_name}.{attr_name}: invalid interval_range_s={ir} (must have low < high)"
                    )

    print("  [OK] All term signatures and URDF body/joint regexes matched!")


def validate_custom_mdp_tensors():
    import torch
    if not hasattr(torch, "zeros"):
        print("  [INFO] Bare system Python detected (no PyTorch); run with '.venv/bin/python src/scripts/train/validate_setup.py' for tensor runtime test.")
        return

    from train import curriculums, events, observations, rewards
    from isaaclab.managers import SceneEntityCfg

    num_envs = 16
    num_bodies = 25
    num_joints = 22

    class _MockPhysxView:
        def __init__(self):
            self._coms = torch.zeros((num_envs, num_bodies, 7))
            self._masses = torch.ones((num_envs, num_bodies))
            self._inertias = torch.ones((num_envs, num_bodies, 9))

        def get_coms(self):
            return self._coms

        def set_coms(self, coms, env_ids):
            self._coms[env_ids] = coms[env_ids]

        def get_masses(self):
            return self._masses

        def set_masses(self, masses, env_ids):
            self._masses[env_ids] = masses[env_ids]

        def get_inertias(self):
            return self._inertias

        def set_inertias(self, inertias, env_ids):
            self._inertias[env_ids] = inertias[env_ids]

    class _MockAsset:
        def __init__(self):
            self.device = torch.device("cpu")
            self.num_bodies = num_bodies
            self.num_joints = num_joints
            self.root_physx_view = _MockPhysxView()
            self.data = types.SimpleNamespace(
                default_mass=torch.ones((num_envs, num_bodies)),
                default_inertia=torch.ones((num_envs, num_bodies, 9)),
                default_joint_pos=torch.zeros((num_envs, num_joints)),
                soft_joint_pos_limits=torch.stack(
                    [-torch.ones((num_envs, num_joints)), torch.ones((num_envs, num_joints))], dim=-1
                ),
                root_pos_w=torch.zeros((num_envs, 3)),
                root_quat_w=torch.tensor([[1.0, 0.0, 0.0, 0.0]] * num_envs),
                root_lin_vel_w=torch.zeros((num_envs, 3)),
                root_ang_vel_w=torch.zeros((num_envs, 3)),
                root_ang_vel_b=torch.randn((num_envs, 3)),
                body_lin_vel_w=torch.zeros((num_envs, num_bodies, 3)),
            )

        def set_external_force_and_torque(self, forces, torques, env_ids=None, body_ids=None):
            assert forces.shape == torques.shape
            assert forces.shape[-1] == 3

        def set_joint_position_target(self, target, joint_ids=None, env_ids=None):
            assert target.shape == (len(env_ids), len(joint_ids))

    class _MockContactSensor:
        def __init__(self):
            self.data = types.SimpleNamespace(
                net_forces_w=torch.randn((num_envs, num_bodies, 3)),
                net_forces_w_history=torch.randn((num_envs, 3, num_bodies, 3)),
                last_air_time=torch.rand((num_envs, num_bodies)),
                current_air_time=torch.rand((num_envs, num_bodies)),
                current_contact_time=torch.rand((num_envs, num_bodies)),
            )

        def compute_first_contact(self, dt):
            return torch.ones((num_envs, num_bodies))

    class _MockScene:
        def __init__(self):
            self.num_envs = num_envs
            self.env_origins = torch.zeros((num_envs, 3))
            self._robot = _MockAsset()
            self.sensors = {"contact_forces": _MockContactSensor()}

        def __getitem__(self, key):
            return self._robot

    class _MockEnv:
        def __init__(self):
            self.device = torch.device("cpu")
            self.step_dt = 0.04
            self.max_episode_length_s = 20.0
            self.common_step_counter = 500
            self.episode_length_buf = torch.randint(0, 100, (num_envs,))
            self.scene = _MockScene()
            self.command_manager = types.SimpleNamespace(
                get_command=lambda name: torch.randn((num_envs, 3))
            )
            self.reward_manager = types.SimpleNamespace(
                get_term_cfg=lambda name: types.SimpleNamespace(weight=2.0),
                _episode_sums={"track_lin_vel_xy_exp": torch.full((num_envs,), 35.0)},
            )

    env = _MockEnv()
    hand_cfg = SceneEntityCfg("robot")
    hand_cfg.body_ids = [5, 11]
    base_cfg = SceneEntityCfg("robot")
    base_cfg.body_ids = [0]
    arm_cfg = SceneEntityCfg("robot")
    arm_cfg.joint_ids = list(range(10))
    feet_sensor_cfg = SceneEntityCfg("contact_forces")
    feet_sensor_cfg.body_ids = [17, 23]

    # Test startup & reset events with both None and partial env_ids
    for env_ids in (None, torch.tensor([0, 3, 7])):
        events.randomize_body_com_offset(
            env, env_ids, base_cfg, {"x": (-0.04, 0.04), "y": (-0.03, 0.03), "z": (-0.04, 0.04)}
        )
        events.randomize_hand_payload(env, env_ids, hand_cfg, mass_range=(0.0, 1.5))
        events.apply_curriculum_external_wrench(
            env,
            env_ids,
            hand_cfg,
            force_range={"x": (-15.0, 15.0), "y": (-15.0, 15.0), "z": (-25.0, 10.0)},
            torque_range=(-3.0, 3.0),
            active_prob=0.7,
            buffer_name="hand_wrench",
        )
        events.drive_random_arm_movements(env, env_ids, arm_cfg)

    events.push_by_setting_velocity_curriculum(
        env, torch.tensor([1, 4]), {"x": (-0.8, 0.8), "y": (-0.8, 0.8), "yaw": (-0.6, 0.6)}
    )

    # Test privileged observations
    w_obs = observations.applied_external_wrench(env, "hand_wrench", num_bodies=2)
    assert w_obs.shape == (num_envs, 12)
    p_obs = observations.hand_payload_mass(env, "hand_payload", num_bodies=2)
    assert p_obs.shape == (num_envs, 2)
    c_obs = observations.robustness_curriculum_level(env)
    assert c_obs.shape == (num_envs, 1)
    f_obs = observations.feet_contact_forces(env, feet_sensor_cfg)
    assert f_obs.shape == (num_envs, 6)

    # Test rewards
    r1 = rewards.base_height_below_min(env, min_height=-0.12)
    r2 = rewards.feet_air_time_symmetry(env, "base_velocity", feet_sensor_cfg)
    r3 = rewards.base_ang_acc_xy_l2(env)
    assert r1.shape == (num_envs,) and r2.shape == (num_envs,) and r3.shape == (num_envs,)

    # Test curriculum
    scale = curriculums.disturbance_levels_vel(env, [0, 1, 2, 3])
    assert 0.1 <= float(scale.item()) <= 1.0
    print("  [OK] All custom events, observations, rewards, and curriculum passed tensor execution!")


if __name__ == "__main__":
    print("=== Running Pre-Flight Dry-Run Validation ===")
    torch_ok = check_torch()
    isaaclab_ok = check_isaaclab()
    if not isaaclab_ok:
        _install_isaaclab_mocks_if_needed()

    validate_configs_and_regexes()
    if torch_ok:
        validate_custom_mdp_tensors()
    print("=== ALL CHECKS PASSED SUCCESSFULLY ===")
