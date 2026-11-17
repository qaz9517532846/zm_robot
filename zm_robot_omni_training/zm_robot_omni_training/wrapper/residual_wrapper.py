# SPDX-License-Identifier: Apache-2.0
#
# COMPASS-style residual action for the SB3 path: the command applied to the robot is
#     final = clip(base + residual)
# where `base` is the frozen X-Mobility's own [vx, 0, wz] (carried in obs["xmobility"][2048:2051])
# and `residual` is what SAC outputs. SAC's action space becomes the residual range
# (`scale` x the full command range), so SB3 stores/learns the residual, not the final command.

from __future__ import annotations

import gymnasium as gym
import numpy as np
from stable_baselines3.common.vec_env import VecEnvWrapper


class ResidualActionVecEnv(VecEnvWrapper):

    def __init__(self, venv, low, high, scale: float, obs_key: str = "xmobility",
                 base_slice: slice = slice(2048, 2051)):
        self.low, self.high = np.asarray(low, np.float32), np.asarray(high, np.float32)
        self.obs_key, self.base_slice = obs_key, base_slice
        super().__init__(venv,
                         action_space=gym.spaces.Box(low=self.low * scale,
                                                     high=self.high * scale,
                                                     dtype=np.float32))
        self._base = np.zeros((venv.num_envs, 3), np.float32)

    def _update_base(self, obs):
        self._base = np.asarray(obs[self.obs_key][:, self.base_slice], np.float32)

    def reset(self):
        obs = self.venv.reset()
        self._update_base(obs)
        return obs

    def step_async(self, actions):
        final = np.clip(self._base + np.asarray(actions, np.float32), self.low, self.high)
        self.venv.step_async(final)

    def step_wait(self):
        obs, rew, dones, infos = self.venv.step_wait()
        self._update_base(obs)    # obs of the NEW state (already reset where an episode ended)
        return obs, rew, dones, infos
