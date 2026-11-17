# SPDX-License-Identifier: Apache-2.0
#
# zm_robot's full RL environment config - self-contained, no COMPASS/mobility_es import.
# Scene/Observations/Commands/Events/Rewards/Terminations/Curriculum structure is
# logic-equivalent to mobility_es/config/env_cfg.py's GoalReachingEnvCfg (ported, not
# inherited, since that base class only existed to share code across COMPASS's several
# embodiments - zm_robot is the only one here).

from __future__ import annotations

from isaaclab.envs import mdp
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensorCfg, MultiMeshRayCasterCfg
from isaaclab.sensors.ray_caster import patterns
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_physx.physics import PhysxCfg

from zm_robot_omni_training.config import environments
from zm_robot_omni_training.config import scene_assets
from zm_robot_omni_training.config import robots
from zm_robot_omni_training.mdp import commands as zm_commands
from zm_robot_omni_training.mdp.action.holonomic_perfect_control_action import (
    HolonomicPerfectControlAction, HolonomicPerfectControlActionCfg)
from zm_robot_omni_training.mdp.observations import (XMobilityObs, camera_depth_u8, camera_img,
                                                      camera_img_chw, root_speed,
                                                      root_yaw_rate, routing_to_goal_simplified,
                                                      relative_goal_heading, camera_to_world,
                                                      camera_intrinsics, lidar_occupancy_grid,
                                                      lidar_polar_ranges)
from zm_robot_omni_training.mdp.rewards import (goal_reaching, goal_progress, goal_heading_alignment,
                                                goal_heading_progress,
                                                route_heading_lock, masked_action_rate_l2,
                                                masked_action_l2)
from zm_robot_omni_training.mdp.termination import goal_reached, nan_pose, illegal_contact_after_grace
from zm_robot_omni_training.mdp.events import reset_root_state_uniform_collision_free
from zm_robot_omni_training.mdp.curriculum import increase_command_minimum_distance_prob

EPISODE_LENGTH_S = 100

# zm_robot's chassis is named "base_link" (not "chassis_link" like COMPASS's Carter),
# and its RGB-D camera / 2 SICK LiDAR mount points are bare Xform links in the USD (no
# sensor schema authored there) - see zm_robot_omni_training/usd/zm_robot/zm_robot.usd.
ZM_ROBOT_BASE_LINK = "base_link"
ZM_ROBOT_CAMERA_MOUNT = "kinect_v2"
ZM_ROBOT_LIDAR_MOUNTS = ("sick_lidar0", "sick_lidar1")
# 2D scan: single channel, full horizontal sweep. Placeholder FOV/resolution - not yet
# matched to the real SICK lidar's datasheet (DESIGN.md section 9, item 1).
_LIDAR_PATTERN = patterns.LidarPatternCfg(channels=1,
                                          vertical_fov_range=(0.0, 0.0),
                                          horizontal_fov_range=(-180.0, 180.0),
                                          horizontal_res=2.0)
# Plain RayCasterCfg only supports ONE mesh prim total (isaaclab/sensors/ray_caster/
# ray_caster.py: "RayCaster currently only supports one mesh prim"), so hitting both the
# ground AND the per-env office furniture needs MultiMeshRayCasterCfg instead - it
# accepts several RaycastTargetCfg entries and supports {ENV_REGEX_NS} templating for
# per-env-replicated geometry (which plain RayCaster's single shared mesh cache does
# not - fine for the always-global ground plane, not for a scene asset that's a
# separate copy per env). merge_prim_meshes=True combines every sub-mesh under the
# office prim (walls, furniture, ...) into one raycast target.
_LIDAR_MESH_TARGETS = [
    "/World/ground",
    MultiMeshRayCasterCfg.RaycastTargetCfg(
        prim_expr="{ENV_REGEX_NS}/SimpleOffice",
        merge_prim_meshes=True,
        is_shared=True,    # same office asset in every env, just at a different origin
        track_mesh_transforms=False,    # static scene geometry, never moves after spawn
    ),
]


@configclass
class SceneCfg(InteractiveSceneCfg):
    """zm_robot scene: ground, light, robot, RGB-D camera, 2 LiDARs, contact sensor."""

    terrain = scene_assets.terrain
    environment = environments.simple_office
    light = scene_assets.light
    robot = robots.zm_robot.replace(prim_path="{ENV_REGEX_NS}/Robot")
    camera = scene_assets.camera.replace(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{ZM_ROBOT_CAMERA_MOUNT}/front_cam")
    lidar_0 = MultiMeshRayCasterCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{ZM_ROBOT_LIDAR_MOUNTS[0]}",
        update_period=0.0,
        mesh_prim_paths=_LIDAR_MESH_TARGETS,
        pattern_cfg=_LIDAR_PATTERN,
        ray_alignment="yaw",
    )
    lidar_1 = MultiMeshRayCasterCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{ZM_ROBOT_LIDAR_MOUNTS[1]}",
        update_period=0.0,
        mesh_prim_paths=_LIDAR_MESH_TARGETS,
        pattern_cfg=_LIDAR_PATTERN,
        ray_alignment="yaw",
    )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*",
                                      history_length=3,
                                      track_air_time=True)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """1D/vector terms only - concatenated into one flat tensor (RSL-RL's MLP
        branch needs a single [B, D] tensor per group, see mdp/observations.py's
        camera_img_chw docstring for why camera/lidar are split into their own groups)."""
        base_speed = ObsTerm(func=root_speed, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_yaw_rate = ObsTerm(func=root_yaw_rate, noise=Unoise(n_min=-0.1, n_max=0.1))
        actions = ObsTerm(func=mdp.last_action, params={"action_name": "drive_joints"})
        route = ObsTerm(func=routing_to_goal_simplified)
        goal_heading = ObsTerm(func=relative_goal_heading)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class CameraCfg(ObsGroup):
        """2D/image group for RSL-RL's CNN branch: [B, C, H, W], channels-first."""
        camera_rgb_img = ObsTerm(func=camera_img_chw, params={"data_type": "rgb"})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class LidarCfg(ObsGroup):
        """2D/image group for RSL-RL's CNN branch: [B, 1, GRID_SIZE, GRID_SIZE]."""
        lidar_occupancy_grid = ObsTerm(func=lidar_occupancy_grid)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class PrivilegedCfg(ObsGroup):
        """Extra privileged observations - not yet wired into the RSL-RL obs_groups
        (DESIGN.md section 10 step 5: symmetric actor-critic first, asymmetric later)."""
        camera_depth_img = ObsTerm(func=camera_img, params={"data_type": "depth"})
        camera_to_world = ObsTerm(func=camera_to_world)
        camera_intrinsics = ObsTerm(func=camera_intrinsics)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class EvalCfg(ObsGroup):
        fall_down = ObsTerm(
            func=mdp.illegal_contact,
            params={
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[ZM_ROBOT_BASE_LINK]),
                "threshold": 1.0
            },
        )
        goal_reached = ObsTerm(func=goal_reached)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()
    camera: CameraCfg = CameraCfg()
    lidar: LidarCfg = LidarCfg()
    # PrivilegedCfg/EvalCfg are defined above but NOT registered as active groups here:
    # their concatenate_terms=False (nested-dict) terms made RSL-RL's check_nan() choke
    # (it expects every top-level group to resolve to one flat Tensor already, it
    # walks obs.items() over every group that exists regardless of whether obs_groups
    # references it). Re-enable once asymmetric critic / eval diagnostics are wired up.


@configclass
class SACObservationsCfg:
    """Observation layout for the SB3/SAC path (see config/agents/sb3_sac_cfg.py).

    Isaac Lab's isaaclab_rl.sb3.Sb3VecEnvWrapper only reads obs_dict["policy"] - unlike
    RSL-RL, it has no notion of separate named groups (camera/lidar) alongside it. So
    every term that PPO's ObservationsCfg splits across policy/camera/lidar groups has
    to live in this one "policy" group instead, with concatenate_terms=False so SB3's
    wrapper exposes it as a gym.spaces.Dict (one entry per term) that MultiInputPolicy's
    CombinedExtractor fans out across a CNN per image-shaped term and a flatten+concat
    for the vector ones - no custom feature extractor needed.
    """

    @configclass
    class PolicyCfg(ObsGroup):
        base_speed = ObsTerm(func=root_speed, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_yaw_rate = ObsTerm(func=root_yaw_rate, noise=Unoise(n_min=-0.1, n_max=0.1))
        actions = ObsTerm(func=mdp.last_action, params={"action_name": "drive_joints"})
        route = ObsTerm(func=routing_to_goal_simplified)
        goal_heading = ObsTerm(func=relative_goal_heading)
        # clip=(0, 255) is load-bearing, not cosmetic: Isaac Lab always declares obs
        # spaces as float32 Box(-inf, inf) unless a term sets `clip`, and
        # Sb3VecEnvWrapper's image detection requires the declared bounds to be exactly
        # (0, 255) (or (-1, 1)) before it will treat a 3D term as an image and cast it.
        camera_rgb_img = ObsTerm(func=camera_img_chw,
                                 params={
                                     "data_type": "rgb",
                                     "normalize": False
                                 },
                                 clip=(0.0, 255.0))
        lidar_occupancy_grid = ObsTerm(func=lidar_occupancy_grid,
                                      params={"normalize": False},
                                      clip=(0.0, 255.0))

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class SACStateObservationsCfg:
    """Vector-only observations (no camera / LiDAR) - a diagnostic variant to check that
    SAC can learn goal-seeking at all, separately from learning a CNN from pixels
    (exp 2 learned nothing after 38k steps with images; DESIGN.md section 10 step 8)."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_speed = ObsTerm(func=root_speed, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_yaw_rate = ObsTerm(func=root_yaw_rate, noise=Unoise(n_min=-0.1, n_max=0.1))
        actions = ObsTerm(func=mdp.last_action, params={"action_name": "drive_joints"})
        route = ObsTerm(func=routing_to_goal_simplified)
        goal_heading = ObsTerm(func=relative_goal_heading)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class SACStateLidarObservationsCfg:
    """SACStateObservationsCfg + a 36-sector LiDAR range vector (no camera)."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_speed = ObsTerm(func=root_speed, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_yaw_rate = ObsTerm(func=root_yaw_rate, noise=Unoise(n_min=-0.1, n_max=0.1))
        actions = ObsTerm(func=mdp.last_action, params={"action_name": "drive_joints"})
        route = ObsTerm(func=routing_to_goal_simplified)
        goal_heading = ObsTerm(func=relative_goal_heading)
        lidar_ranges = ObsTerm(func=lidar_polar_ranges)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class SACStateLidarXMobObservationsCfg:
    """SACStateLidarObservationsCfg + frozen X-Mobility features (RGB) + a depth image.

    Dict observation (concatenate_terms=False) so SB3's MultiInputPolicy sends the depth
    image through a CNN and flattens/concatenates everything else."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_speed = ObsTerm(func=root_speed, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_yaw_rate = ObsTerm(func=root_yaw_rate, noise=Unoise(n_min=-0.1, n_max=0.1))
        actions = ObsTerm(func=mdp.last_action, params={"action_name": "drive_joints"})
        route = ObsTerm(func=routing_to_goal_simplified)
        goal_heading = ObsTerm(func=relative_goal_heading)
        lidar_ranges = ObsTerm(func=lidar_polar_ranges)
        xmobility = ObsTerm(func=XMobilityObs)
        # clip=(0, 255): see SACObservationsCfg.camera_rgb_img for why this is load-bearing.
        depth_img = ObsTerm(func=camera_depth_u8, clip=(0.0, 255.0))

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    goal_pose = zm_commands.UniformCollisionFreePose2dCommandCfg(
        asset_name="robot",
        resampling_time_range=(EPISODE_LENGTH_S, EPISODE_LENGTH_S),
        debug_vis=False,
        simple_heading=False,
        ranges=zm_commands.UniformCollisionFreePose2dCommandCfg.Ranges(
            pos_x=(-3, 3),    # was +-5: nearer goals are reachable by early exploration
            pos_y=(-3, 3),
            heading=(-3.14, 3.14),
        ),
        minimum_distance=1.5,
        minimum_distance_prob=0.9)


@configclass
class ZmRobotActionsCfg:
    """Action specifications for the MDP."""

    drive_joints = HolonomicPerfectControlActionCfg(class_type=HolonomicPerfectControlAction,
                                                     asset_name="robot")


@configclass
class EventCfg:
    """Configuration for events."""

    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=ZM_ROBOT_BASE_LINK),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTerm(
        func=reset_root_state_uniform_collision_free,
        mode="reset",
        params={
            "pose_range": {
                "x": (-5, 5),
                "y": (-5, 5),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )
    # No reset_robot_joints term: the mecanum wheels/rollers are continuous, velocity-mode
    # joints with no meaningful "position" to reset to (same choice COMPASS made for carter).


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # Progress-per-step instead of -distance (see rewards.goal_progress). The reward
    # manager multiplies by step_dt (0.1 s), so weight 100 -> up to ~0.6/step at full
    # speed (0.06 m/step), i.e. ~40 for a 4 m trip, vs +200 (2000*dt) on arrival and -100
    # (-1000*dt) on contact.
    dist_to_goal = RewTerm(func=goal_progress, params={"offset": (0.0, 0.0)}, weight=100.0)
    goal_reached_reward = RewTerm(
        func=mdp.is_terminated_term,
        weight=2000.0,
        params={"term_keys": "goal_reached"},
    )
    base_contact_penalty = RewTerm(
        func=mdp.is_terminated_term,
        weight=-1000.0,
        params={"term_keys": "base_contact"},
    )
    # DESIGN.md section 6.1: the 4 side-stepping-inducing mechanisms (items 1-3; item 4
    # is scene/curriculum, handled separately - see the module docstring note below).
    # Weights are untuned first-pass starting points, not validated against training.
    route_heading_lock = RewTerm(func=route_heading_lock, weight=0.5)    # item 1 (key mechanism)
    # item 2, exp 9: progress form instead of the per-step penalty (see
    # rewards.goal_heading_progress). weight 30 * step_dt 0.1 -> a full pi turn near the
    # goal is worth ~ +9, comparable to distance progress, far below the +200 arrival.
    goal_heading_progress = RewTerm(func=goal_heading_progress, weight=30.0)
    # item 3: symmetric smoothness penalty across (vx, vy, wz) - no idx_masks needed now
    # (action_dim is already just 3), unlike COMPASS's masked_action_*.
    action_rate_penalty = RewTerm(func=masked_action_rate_l2, weight=0.01)
    action_penalty = RewTerm(func=masked_action_l2, weight=0.01)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        # illegal_contact_after_grace, not the plain isaaclab mdp.illegal_contact: the
        # HolonomicPerfectControlAction kinematic reset produces a spurious multi-
        # hundred-thousand-newton contact transient for ~2 steps after every reset
        # (confirmed by direct measurement), which would otherwise instantly end
        # nearly every episode. See termination.py's docstring for the trace.
        func=illegal_contact_after_grace,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[ZM_ROBOT_BASE_LINK]),
            "threshold": 1.0
        },
    )
    goal_reached = DoneTerm(func=goal_reached)
    nan_pose = DoneTerm(func=nan_pose)


@configclass
class CurriculumCfg:
    command_min_distance_prob = CurrTerm(func=increase_command_minimum_distance_prob,
                                         params={
                                             "minimum_distance_prob_range": (0.1, 0.9),
                                             "total_iterations": 1000,
                                             "num_steps_per_iteration": 256
                                         })


@configclass
class ZmRobotGoalReachingEnvCfg(ManagerBasedRLEnvCfg):
    """zm_robot reaching a 2D target pose, via holonomic (vx, vy, wz) control."""

    scene = SceneCfg(num_envs=16, env_spacing=20.0)
    observations = ObservationsCfg()
    commands = CommandsCfg()
    actions = ZmRobotActionsCfg()
    events = EventCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    curriculum = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 20
        self.episode_length_s = EPISODE_LENGTH_S
        self.num_rerenders_on_reset = 1
        self.sim.dt = 0.005
        self.sim.physics = PhysxCfg(bounce_threshold_velocity=0.2)
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.restitution = 0.0
        self.sim.render_interval = self.decimation
        self.sim.render.enable_dl_denoiser = True
        self.sim.render.antialiasing_mode = 'DLAA'

        self.scene.replicate_physics = self.scene.environment.replicate_physics
        self.scene.env_spacing = self.scene.environment.env_spacing
        self.events.reset_base.params["pose_range"] = self.scene.environment.pose_sample_range

        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt


@configclass
class ZmRobotGoalReachingSACEnvCfg(ZmRobotGoalReachingEnvCfg):
    """Same scene/actions/rewards/terminations/commands/curriculum as the PPO env -
    only the observation layout differs (see SACObservationsCfg for why)."""

    observations = SACObservationsCfg()


@configclass
class ZmRobotGoalReachingSACStateEnvCfg(ZmRobotGoalReachingEnvCfg):
    """State-only variant: vector observations, camera removed (no rendering needed, so
    many more parallel envs fit on the GPU and runs are much faster). LiDAR sensors stay
    in the scene but are not observed."""

    observations = SACStateObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.camera = None


@configclass
class ZmRobotGoalReachingSACStateLidarEnvCfg(ZmRobotGoalReachingSACStateEnvCfg):
    """Vector observations + LiDAR sector ranges; camera still removed."""

    observations = SACStateLidarObservationsCfg()


@configclass
class ZmRobotGoalReachingSACStateLidarHybridEnvCfg(ZmRobotGoalReachingSACStateLidarEnvCfg):
    """Hybrid goal-heading reward: exp 9's progress term (weight 30, unchanged) PLUS a
    quarter-strength exp 7 penalty (goal_heading_alignment, weight 1.0 instead of 4.0).

    Motivation (5-seed comparison, DESIGN.md): progress-only reaches the goal but ends with
    poor heading; penalty-only ends with good heading but stays away from the goal. The
    penalty is kept small on purpose: at full weight it summed to -190..-270 per timed-out
    episode and rewarded staying away (exp 7). Only the penalty is added - one variable."""

    def __post_init__(self):
        super().__post_init__()
        self.rewards.goal_heading_alignment = RewTerm(func=goal_heading_alignment, weight=1.0)


@configclass
class ZmRobotGoalReachingSACStateLidarXMobEnvCfg(ZmRobotGoalReachingEnvCfg):
    """State + LiDAR vectors (as exp 9) + frozen X-Mobility features from the RGB camera
    + a depth image. Camera kept (X-Mobility needs the 320x512 render)."""

    observations = SACStateLidarXMobObservationsCfg()


@configclass
class ZmRobotGoalReachingSACStateLidarXMobPenaltyEnvCfg(ZmRobotGoalReachingSACStateLidarXMobEnvCfg):
    """XMob variant with the exp 7 goal-heading penalty instead of the progress reward
    (same swap as ZmRobotGoalReachingSACStateLidarPenaltyEnvCfg)."""

    def __post_init__(self):
        super().__post_init__()
        self.rewards.goal_heading_progress = None
        self.rewards.goal_heading_alignment = RewTerm(func=goal_heading_alignment, weight=4.0)


@configclass
class ZmRobotGoalReachingSACStateLidarXMobHybridEnvCfg(ZmRobotGoalReachingSACStateLidarXMobEnvCfg):
    """XMob variant with the hybrid goal-heading reward (progress weight 30 + penalty weight 1.0),
    i.e. ZmRobotGoalReachingSACStateLidarHybridEnvCfg's rewards plus X-Mobility + depth."""

    def __post_init__(self):
        super().__post_init__()
        self.rewards.goal_heading_alignment = RewTerm(func=goal_heading_alignment, weight=1.0)


@configclass
class ZmRobotGoalReachingSACStateLidarPenaltyEnvCfg(ZmRobotGoalReachingSACStateLidarEnvCfg):
    """Control for exp 9: identical to the state+LiDAR env EXCEPT the goal-heading reward
    is the old per-step penalty (goal_heading_alignment, weight 4.0 - what exp 7 used)
    instead of goal_heading_progress (weight 30). Used to test whether the progress form
    helps, with from-scratch training and fixed seeds on both sides."""

    def __post_init__(self):
        super().__post_init__()
        self.rewards.goal_heading_progress = None    # None disables a term
        self.rewards.goal_heading_alignment = RewTerm(func=goal_heading_alignment, weight=4.0)

