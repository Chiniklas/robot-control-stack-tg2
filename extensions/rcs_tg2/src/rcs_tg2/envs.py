import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
from rcs.envs.base import RobotEnv
from rcs_tg2._core import hw

_logger = logging.getLogger(__name__)


class TG2HW(gym.Wrapper):
    """Stub wrapper to mirror FR3 hardware wrapper.

    Currently only intercepts hypothetical control exceptions and marks episodes as truncated.
    """

    def __init__(self, env):
        super().__init__(env)
        self.unwrapped: RobotEnv
        assert isinstance(self.unwrapped.robot, hw.TG2), "Robot must be a hw.TG2 instance."
        self.hw_robot = cast(hw.TG2, self.unwrapped.robot)

    def step(self, action: Any) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict]:
        try:
            return super().step(action)
        except hw.exceptions.TG2ControlException as e:
            _logger.error("TG2ControlException: %s", e)
            return dict(self.unwrapped.get_obs()), 0, False, True, {}

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        return super().reset(seed=seed, options=options)

    def close(self):
        self.hw_robot.close()
        super().close()
