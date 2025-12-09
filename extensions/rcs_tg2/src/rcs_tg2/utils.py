import logging
from pathlib import Path

import rcs
from rcs import common
from rcs.envs.utils import default_mujoco_cameraset_cfg, default_sim_gripper_cfg, default_sim_robot_cfg
from rcs_tg2._core import hw

# Scenes follow the shared repository layout: assets/scenes/<scene_name>/...
SCENES_DIR = Path(__file__).resolve().parents[4] / "assets" / "scenes"
logger = logging.getLogger(__name__)


def get_asset_path(name: str) -> str:
    """Return an absolute path to an asset inside assets/scenes."""
    return str((SCENES_DIR / name).resolve())


def _scene_file(scene: str, filename: str) -> str:
    """Helper to resolve a file within a scene directory."""
    return get_asset_path(f"{scene}/{filename}")


def default_tg2_hw_robot_cfg(async_control: bool = False) -> hw.TG2Config:
    cfg = hw.TG2Config()
    # Placeholder scene references; adjust once a TG2 scene is available
    cfg.robot_type = getattr(common, "RobotType", None) and getattr(common.RobotType, "UNKNOWN", None)
    cfg.kinematic_model_path = None
    cfg.tcp_offset = None
    cfg.attachment_site = None
    cfg.speed_factor = 0.1
    cfg.async_control = async_control
    return cfg


def default_tg2_hw_gripper_cfg(async_control: bool = False) -> hw.TG2GripperConfig:
    cfg = hw.TG2GripperConfig()
    cfg.async_control = async_control
    return cfg


def default_tg2_sim_robot_cfg(scene: str = "tg2_empty_world", idx: str = "0") -> rcs.sim.SimRobotConfig:
    """
    Build a SimRobotConfig for TG2 using the base defaults and overriding kinematic model / TCP attachment.
    Raises with guidance if the scene or required assets are missing.
    """
    if scene not in rcs.scenes:
        raise RuntimeError(
            f"Scene '{scene}' not registered in rcs.scenes. "
            f"Add it in python/rcs/__init__.py and ensure assets/scenes/{scene}/scene.mjb (or scene.xml) exists."
        )
    robot_cfg = rcs.sim.SimRobotConfig()
    robot_cfg.add_id(idx)

    mjb_path = Path(rcs.scenes[scene].mjb)
    if not mjb_path.exists():
        mjb_path = Path(_scene_file(scene, "scene.mjb"))
    if not mjb_path.exists():
        alt_xml = _scene_file(scene, "scene.xml")
        if not Path(alt_xml).exists():
            raise FileNotFoundError(
                f"Scene assets missing for '{scene}'. Expected {mjb_path} or {alt_xml}."
            )
        robot_cfg.mjcf_scene_path = alt_xml
    else:
        robot_cfg.mjcf_scene_path = str(mjb_path)

    model_path = _scene_file(scene, "robot.xml")
    if not Path(model_path).exists():
        raise FileNotFoundError(f"Kinematic model not found for '{scene}': {model_path}")

    # Require a dedicated TG2 enum
    if not hasattr(common.RobotType, "HumanoidTG2"):
        raise RuntimeError("RobotType.HumanoidTG2 not defined; add it to rcs.common RobotType enum first.")

    robot_cfg.robot_type = common.RobotType.HumanoidTG2
    robot_cfg.arm_collision_geoms = []  # no dedicated collision geoms defined in tg2 MJCF
    robot_cfg.base = "pelvis"
    robot_cfg.joints = [
        "waist_yaw_joint",
        "head_yaw_joint",
        "head_pitch_joint",
        "head_roll_joint",
        "shoulder_pitch_l_joint",
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint",
        "shoulder_pitch_r_joint",
        "shoulder_roll_r_joint",
        "shoulder_yaw_r_joint",
        "elbow_pitch_r_joint",
        "elbow_yaw_r_joint",
        "wrist_pitch_r_joint",
        "wrist_roll_r_joint",
    ]
    robot_cfg.actuators = [
        "waist_yaw",
        "head_yaw",
        "head_pitch",
        "head_roll",
        "shoulder_pitch_l",
        "shoulder_roll_l",
        "shoulder_yaw_l",
        "elbow_pitch_l",
        "elbow_yaw_l",
        "wrist_pitch_l",
        "wrist_roll_l",
        "shoulder_pitch_r",
        "shoulder_roll_r",
        "shoulder_yaw_r",
        "elbow_pitch_r",
        "elbow_yaw_r",
        "wrist_pitch_r",
        "wrist_roll_r",
    ]
    robot_cfg.attachment_site = "attachment_site_0"
    robot_cfg.mjcf_scene_path = str(mjb_path) if mjb_path.exists() else _scene_file(scene, "scene.xml")
    robot_cfg.kinematic_model_path = model_path
    return robot_cfg


def default_tg2_sim_gripper_cfg(idx: str = "0") -> rcs.sim.SimGripperConfig:
    # No TG2 gripper actuators defined; disable gripper in sim by returning None.
    return None  # type: ignore[return-value]


def default_tg2_mujoco_cameraset_cfg(scene: str = "tg2_empty_world") -> dict:
    # Scene is accepted for symmetry; current cameras mirror the global defaults.
    return default_mujoco_cameraset_cfg()
