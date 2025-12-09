import os
os.environ.setdefault("MUJOCO_GL", "egl")  # offscreen; avoid viewer
from rcs.envs.base import ControlMode
from rcs_tg2.creators import RCSTG2SimEnvCreator

env = RCSTG2SimEnvCreator()(
    control_mode=ControlMode.JOINTS,
    scene="tg2_empty_world",
    max_relative_movement=None,
    relative_to=None,
    gripper_cfg=None,
    cameras=None,  # no camera wrapper -> no viewer
)
sim = env.get_wrapper_attr("sim")
obs, _ = env.reset()
print("joints before:", obs["joints"])
env.step({"joints": [0.5]*18})
print("ctrl after set:", sim.d.ctrl[:18])
env.step({"joints": [0.5]*18})
obs, *_ = env.step({"joints": [0.5]*18})
print("joints after:", obs["joints"])
env.close()

