# SPDX-License-Identifier: Apache-2.0
#
# Env scene assets, logic-equivalent to mobility_es/config/environments.py but pointing
# at USD files copied into this repo's own usd/ dir (not referencing COMPASS's copies).
# Only `simple_office` is ported for now - the rack/warehouse scenes needed for the
# "narrow aisle" curriculum (DESIGN.md section 6.1 item 4) are a follow-up.

import os

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils import configclass

USD_PATHS = {
    'SimpleOffice': os.path.join(os.path.dirname(__file__), "../usd/office/office.usd"),
}


@configclass
class EnvSceneAssetCfg(AssetBaseCfg):
    """Adds pose-sampling / spacing / replication parameters to AssetBaseCfg."""
    pose_sample_range = {"x": (-5, 5), "y": (-5, 5), "yaw": (-3.14, 3.14)}
    env_spacing = 50
    replicate_physics = True


simple_office = EnvSceneAssetCfg(
    prim_path="{ENV_REGEX_NS}/SimpleOffice",
    init_state=AssetBaseCfg.InitialStateCfg(
        pos=(0, 0, 0.01),
        rot=(0.0, 0.0, 0.0, 1.0),
    ),
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATHS['SimpleOffice'],
        scale=(1.0, 1.0, 1.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=None,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),
    ),
    env_spacing=15,
)
