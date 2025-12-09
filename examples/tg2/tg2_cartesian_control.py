import logging
from time import sleep

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator

import rcs
from rcs import sim as rcs_sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
The example shows how to create a TG2 environment with Cartesian control
and a relative action space. Only simulation is supported in this demo.
"""

ROBOT_INSTANCE = RobotPlatform.SIMULATION
SCENE = "tg2_empty_world"


def main():

    if ROBOT_INSTANCE != RobotPlatform.SIMULATION:
        raise RuntimeError("Only simulation is supported for the TG2 demo.")

    robot_cfg = rcs_sim.SimRobotConfig()
    robot_cfg.base = "pelvis"
    robot_cfg.robot_type = rcs.common.RobotType.HumanoidTG2
    robot_cfg.attachment_site = "attachment_site_0"
    robot_cfg.arm_collision_geoms = []
    robot_cfg.joints = [
        "shoulder_pitch_l_joint",
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint",
    ]
    robot_cfg.actuators = [
        "shoulder_pitch_l",
        "shoulder_roll_l",
        "shoulder_yaw_l",
        "elbow_pitch_l",
        "elbow_yaw_l",
        "wrist_pitch_l",
        "wrist_roll_l",
    ]
    robot_cfg.mjcf_scene_path = rcs.scenes[SCENE].mjcf_scene
    robot_cfg.kinematic_model_path = rcs.scenes[SCENE].mjcf_robot
    env_rel = SimEnvCreator()(
        robot_cfg=robot_cfg,
        control_mode=ControlMode.CARTESIAN_TQuat,
        gripper_cfg=None,
        cameras=None,
        max_relative_movement=0.5,
        relative_to=RelativeTo.LAST_STEP,
        collision_guard=False,
    )
    sleep(3)  # wait for gui to open
    env_rel.get_wrapper_attr("sim").open_gui()
    obs, info = env_rel.reset()
    logger.info("Initial TCP pose (world): %s", env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore
    # logger.info("Relative action space tquat limits: %s", env_rel.action_space["tquat"])

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward)
            prev_pose = env_rel.unwrapped.robot.get_cartesian_position()  # type: ignore
            # Use identity quaternion so the relative wrapper keeps the current orientation unchanged
            act = {"tquat": [0.00, 0, 0, 0, 0, 0, 1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            curr_pose = env_rel.unwrapped.robot.get_cartesian_position()  # type: ignore
            logger.info(
                "Forward step TCP pose: %s | delta: %s",
                curr_pose,
                curr_pose.translation() - prev_pose.translation(),
            )
            input("press Enter to continue...")
            # TODO: even zero movement makes the robot do crazy things; investigate further
            # NOTE: I think it could because I attached the TCP wrongly

            # prev_pose = curr_pose
            # ori_q = prev_pose.rotation_q()
            # act = {"tquat": [-0.01, 0, 0, *ori_q]}
            # obs, reward, terminated, truncated, info = env_rel.step(act)
            # curr_pose = env_rel.unwrapped.robot.get_cartesian_position()  # type: ignore
            # logger.info(
            #     "Backward step TCP pose: %s | delta: %s",
            #     curr_pose,
            #     curr_pose.translation() - prev_pose.translation(),
            # )

            # input("press Enter to continue...")

if __name__ == "__main__":
    main()
