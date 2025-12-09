from rcs_tg2._core import hw

import rcs
from rcs import common


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
