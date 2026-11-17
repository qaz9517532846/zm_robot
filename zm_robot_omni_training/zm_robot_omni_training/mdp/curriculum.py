# SPDX-License-Identifier: Apache-2.0
#
# Ported (logic-equivalent, no COMPASS import) from mobility_es/mdp/curriculum.py.

from __future__ import annotations

from collections.abc import Sequence

from zm_robot_omni_training.wrapper.env_wrapper import ZmRobotEnvWrapper


def increase_command_minimum_distance_prob(
        env: ZmRobotEnvWrapper,
        env_ids: Sequence[int],    # pylint: disable=unused-argument
        command_name: str = 'goal_pose',
        minimum_distance_prob_range=(0.1, 1.0),
        total_iterations=1000,
        num_steps_per_iteration=256):
    """Linearly ramp the goal_pose command's minimum_distance_prob over training."""
    cur_iteration = env.common_step_counter // num_steps_per_iteration
    start_prob, end_prob = minimum_distance_prob_range
    prob_range = end_prob - start_prob
    minimum_distance_prob = start_prob + prob_range * (cur_iteration / total_iterations)
    env.command_manager.get_term(command_name).cfg.minimum_distance_prob = minimum_distance_prob
