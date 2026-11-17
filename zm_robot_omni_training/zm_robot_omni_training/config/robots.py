# SPDX-License-Identifier: Apache-2.0
#
# zm_robot's ArticulationCfg, split out from NVIDIA COMPASS's mobility_es extension so
# zm_robot-specific code doesn't live inside the vendored COMPASS tree. Mirrors the
# style of mobility_es/config/robots.py's `carter` entry.

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

zm_robot = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/zm_robot/zm_robot.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.02),
        rot=(0.0, 0.0, 0.0, 1),
        joint_pos={
            "wheel_joint_[1-4]": 0.0,
            "roller_[1-4]_[1-9]_joint": 0.0,
        },
        joint_vel={
            "wheel_joint_[1-4]": 0.0,
            "roller_[1-4]_[1-9]_joint": 0.0,
        },
    ),
    actuators={
        # Driven wheels and the passive mecanum rollers are both velocity-mode / torque-free
        # here (stiffness=0, damping=0): the HolonomicPerfectControlAction kinematic shortcut
        # never actually drives these joints, this just keeps the articulation well-formed.
        "wheels":
            ImplicitActuatorCfg(
                joint_names_expr=["wheel_joint_[1-4]"],
                velocity_limit_sim=360.0,
                stiffness=0.0,
                damping=0.0,
            ),
        "rollers":
            ImplicitActuatorCfg(
                joint_names_expr=["roller_[1-4]_[1-9]_joint"],
                velocity_limit_sim=360.0,
                stiffness=0.0,
                damping=0.0,
            ),
    },
)
