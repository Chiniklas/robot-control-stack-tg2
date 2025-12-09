"""Stub hardware layer for TG2.

This mirrors the interface style of rcs_fr3._core.hw but does not talk to real hardware.
It allows upper layers (env creators, CLI) to import and be wired before native bindings exist.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

import rcs
from rcs import common


class NotImplementedHardware(RuntimeError):
    """Raised when a stub hardware call is made."""


@dataclass
class TG2Config(common.RobotConfig):
    robot_type: object | None = getattr(common, "RobotType", None) and getattr(common.RobotType, "UNKNOWN", None)
    robot_platform: object | None = getattr(common, "RobotPlatform", None) and getattr(
        common.RobotPlatform, "HARDWARE", None
    )
    kinematic_model_path: str | None = None
    attachment_site: str | None = None
    tcp_offset: Optional[common.Pose] = None
    speed_factor: float = 0.1
    async_control: bool = False


@dataclass
class TG2State(common.RobotState):
    pass


class TG2(common.Robot):
    def __init__(self, ip: str, ik: Optional[common.Kinematics] = None):
        super().__init__()
        self.ip = ip
        self.cfg = TG2Config()
        self._state = TG2State()
        self._ik = ik

    def set_config(self, cfg: TG2Config) -> bool:
        self.cfg = cfg
        return True

    def get_config(self) -> TG2Config:
        return self.cfg

    def get_state(self) -> TG2State:
        return self._state

    def get_cartesian_position(self) -> common.Pose:
        raise NotImplementedHardware("TG2 hardware driver not implemented")

    def set_cartesian_position(self, pose: common.Pose) -> None:
        raise NotImplementedHardware("TG2 hardware driver not implemented")

    def set_joint_position(self, q: common.VectorXd) -> None:
        raise NotImplementedHardware("TG2 hardware driver not implemented")

    def get_joint_position(self) -> common.VectorXd:
        raise NotImplementedHardware("TG2 hardware driver not implemented")

    def move_home(self) -> None:
        raise NotImplementedHardware("TG2 hardware driver not implemented")

    def reset(self) -> None:
        raise NotImplementedHardware("TG2 hardware driver not implemented")

    def close(self) -> None:
        # No-op stub
        return None

    def get_ik(self) -> Optional[common.Kinematics]:
        return self._ik


@dataclass
class TG2GripperConfig(common.GripperConfig):
    speed: float = 0.05
    force: float = 5.0
    async_control: bool = False


dataclass
class TG2GripperState(common.GripperState):
    width: float = 0.0
    is_grasped: bool = False
    is_moving: bool = False


class TG2Hand(common.Gripper):
    def __init__(self, cfg: TG2GripperConfig | None = None):
        super().__init__()
        self.cfg = cfg or TG2GripperConfig()
        self.state = TG2GripperState()

    def set_config(self, cfg: TG2GripperConfig) -> None:
        self.cfg = cfg

    def get_config(self) -> TG2GripperConfig:
        return self.cfg

    def get_state(self) -> TG2GripperState:
        return self.state

    def set_normalized_width(self, width: float, force: float = 0) -> None:
        self.state.width = float(np.clip(width, 0.0, 1.0))
        self.state.is_moving = False

    def get_normalized_width(self) -> float:
        return self.state.width

    def is_grasped(self) -> bool:
        return self.state.is_grasped

    def grasp(self) -> None:
        self.state.is_grasped = True

    def open(self) -> None:
        self.state.is_grasped = False
        self.state.width = 1.0

    def shut(self) -> None:
        self.state.is_grasped = False
        self.state.width = 0.0

    def close(self) -> None:
        self.state.is_grasped = True
        self.state.width = 0.0

    def reset(self) -> None:
        self.state = TG2GripperState()


class exceptions:  # mirror naming used by fr3
    class TG2ControlException(NotImplementedHardware):
        pass
