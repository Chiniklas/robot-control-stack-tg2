import logging
from os import PathLike

import gymnasium as gym
import numpy as np
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    GripperWrapper,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.creators import RCSHardwareEnvCreator
from rcs_tg2._core import hw
from rcs_tg2.envs import TG2HW
from rcs_tg2.utils import default_tg2_hw_gripper_cfg, default_tg2_hw_robot_cfg

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSTG2EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        ip: str,
        control_mode: ControlMode,
        robot_cfg: hw.TG2Config,
        collision_guard: str | PathLike | None = None,
        gripper_cfg: hw.TG2GripperConfig | None = None,
        camera_set: object | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        """
        Creates a hardware environment for the TG2 robot (stub implementation).
        Replace stub classes with real drivers to enable actual motion.
        """
        ik = None
        if robot_cfg.kinematic_model_path:
            ik = rcs.common.Pin(
                robot_cfg.kinematic_model_path,
                robot_cfg.attachment_site or "",
                urdf=robot_cfg.kinematic_model_path.endswith(".urdf"),
            )

        robot = hw.TG2(ip, ik)
        robot.set_config(robot_cfg)

        env: gym.Env = RobotEnv(robot, ControlMode.JOINTS if collision_guard is not None else control_mode)
        env = TG2HW(env)

        if isinstance(gripper_cfg, hw.TG2GripperConfig):
            gripper = hw.TG2Hand(gripper_cfg)
            env = GripperWrapper(env, gripper, binary=True)

        if camera_set is not None:
            try:
                camera_set.start()
                camera_set.wait_for_frames()
                logger.info("CameraSet started")
                env = CameraSetWrapper(env, camera_set)
            except Exception:  # pragma: no cover - placeholder for future camera APIs
                logger.warning("CameraSet provided but start/wait_for_frames failed; skipping wrapper")

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env


class RCSTG2MultiEnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        name2ip: dict[str, str],
        control_mode: ControlMode,
        robot_cfg: hw.TG2Config,
        gripper_cfg: hw.TG2GripperConfig | None = None,
        camera_set: object | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        ik = None
        if robot_cfg.kinematic_model_path:
            ik = rcs.common.Pin(
                robot_cfg.kinematic_model_path,
                robot_cfg.attachment_site or "",
                urdf=robot_cfg.kinematic_model_path.endswith(".urdf"),
            )

        robots: dict[str, hw.TG2] = {}
        for key, ip in name2ip.items():
            robots[key] = hw.TG2(ip, ik)
            robots[key].set_config(robot_cfg)

        envs: dict[str, gym.Env] = {}
        for key, ip in name2ip.items():
            env: gym.Env = RobotEnv(robots[key], control_mode)
            env = TG2HW(env)
            if gripper_cfg is not None:
                gripper = hw.TG2Hand(gripper_cfg)
                env = GripperWrapper(env, gripper, binary=True)

            if max_relative_movement is not None:
                env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)
            envs[key] = env

        env = MultiRobotWrapper(envs)
        if camera_set is not None:
            try:
                camera_set.start()
                camera_set.wait_for_frames()
                logger.info("CameraSet started")
                env = CameraSetWrapper(env, camera_set)
            except Exception:
                logger.warning("CameraSet provided but start/wait_for_frames failed; skipping wrapper")
        return env


class RCSTG2DefaultEnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        robot_ip: str,
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        delta_actions: bool = True,
        camera_set: object | None = None,
        gripper: bool = True,
    ) -> gym.Env:
        return RCSTG2EnvCreator()(
            ip=robot_ip,
            camera_set=camera_set,
            control_mode=control_mode,
            robot_cfg=default_tg2_hw_robot_cfg(),
            collision_guard=None,
            gripper_cfg=default_tg2_hw_gripper_cfg() if gripper else None,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
        )
