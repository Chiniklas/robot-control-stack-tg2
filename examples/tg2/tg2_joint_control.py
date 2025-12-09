"""
TG2 simulation demo using joint-space control.

Requires TG2 sim assets under assets/scenes/tg2_empty_world/.
Opens the MuJoCo GUI and sends small joint offsets to verify actuation.
"""

import logging
import time

import rcs
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs import sim as rcs_sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_INSTANCE = RobotPlatform.SIMULATION
SCENE = "tg2_empty_world"

# Joint names and actuator names from assets/scenes/tg2_empty_world/robot.xml
JOINT_NAMES = [
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

ACTUATOR_NAMES = [
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

# Subsets to target left/right/both arms
ARM_SELECTIONS = {
    "left": {
        "joints": [
            "shoulder_pitch_l_joint",
            "shoulder_roll_l_joint",
            "shoulder_yaw_l_joint",
            "elbow_pitch_l_joint",
            "elbow_yaw_l_joint",
            "wrist_pitch_l_joint",
            "wrist_roll_l_joint",
        ],
        "actuators": [
            "shoulder_pitch_l",
            "shoulder_roll_l",
            "shoulder_yaw_l",
            "elbow_pitch_l",
            "elbow_yaw_l",
            "wrist_pitch_l",
            "wrist_roll_l",
        ],
    },
    "right": {
        "joints": [
            "shoulder_pitch_r_joint",
            "shoulder_roll_r_joint",
            "shoulder_yaw_r_joint",
            "elbow_pitch_r_joint",
            "elbow_yaw_r_joint",
            "wrist_pitch_r_joint",
            "wrist_roll_r_joint",
        ],
        "actuators": [
            "shoulder_pitch_r",
            "shoulder_roll_r",
            "shoulder_yaw_r",
            "elbow_pitch_r",
            "elbow_yaw_r",
            "wrist_pitch_r",
            "wrist_roll_r",
        ],
    },
    "both": {
        "joints": [
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
        ],
        "actuators": [
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
        ],
    },
}


def main(arm: str = "left", joint_index: int = 0):
    if ROBOT_INSTANCE != RobotPlatform.SIMULATION:
        raise RuntimeError("Only simulation is supported in this demo.")

    if arm not in ARM_SELECTIONS:
        raise ValueError(f"arm must be one of {list(ARM_SELECTIONS.keys())}")
    selection = ARM_SELECTIONS[arm]

    robot_cfg = rcs_sim.SimRobotConfig()
    robot_cfg.joints = selection["joints"]
    robot_cfg.actuators = selection["actuators"]
    robot_cfg.base = "pelvis"
    robot_cfg.robot_type = rcs.common.RobotType.HumanoidTG2
    robot_cfg.attachment_site = "attachment_site_0"
    robot_cfg.arm_collision_geoms = []
    robot_cfg.mjcf_scene_path = rcs.scenes[SCENE].mjcf_scene
    robot_cfg.kinematic_model_path = rcs.scenes[SCENE].mjcf_robot

    env = SimEnvCreator()(
        robot_cfg=robot_cfg,
        control_mode=ControlMode.JOINTS,
        collision_guard=False,
        gripper_cfg=None,
        cameras=None,
        max_relative_movement=None,
        relative_to=RelativeTo.LAST_STEP,
    )
    env.get_wrapper_attr("sim").open_gui()

    obs, info = env.reset()
    logger.info("Reset complete.")
    input("ready to send joint commands, press Enter to continue...")

    if joint_index < 0 or joint_index >= len(robot_cfg.joints):
        raise ValueError(f"joint_index must be in [0, {len(robot_cfg.joints)-1}] for arm={arm}")
    act_vec = [0.0] * len(robot_cfg.joints)
    for step in range(10):
        target = (step + 1) * -0.1  # 0.1 -> 1.0 absolute targets
        act_vec[joint_index] = target
        act = {"joints": act_vec}
        logger.info(
            "Step %d: setting joint %d (%s) to %.2f",
            step + 1,
            joint_index,
            robot_cfg.joints[joint_index],
            target,
        )
        obs, *_ = env.step(act)
        time.sleep(0.5)

    env.close()


if __name__ == "__main__":
    main()
