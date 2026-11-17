# SPDX-License-Identifier: Apache-2.0
#
# Ported verbatim (no COMPASS import involved - these were never X-Mobility/COMPASS
# specific, just generic Isaac Lab scene assets) from mobility_es/config/scene_assets.py.

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.sensors import CameraCfg, TiledCameraCfg
from isaaclab.terrains import TerrainImporterCfg

terrain = TerrainImporterCfg(
    prim_path="/World/ground",
    terrain_type="plane",
    debug_vis=False,
)

light = AssetBaseCfg(
    prim_path="/World/light",
    spawn=sim_utils.DistantLightCfg(color=(1.0, 1.0, 1.0), intensity=1000.0),
)

camera = TiledCameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/chassis_link/front_cam",
    update_period=0.0,
    height=320,
    width=512,
    data_types=["rgb", "depth"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=10.0,
                                     focus_distance=400.0,
                                     horizontal_aperture=20.955,
                                     clipping_range=(0.1, 20.0)),
    offset=CameraCfg.OffsetCfg(pos=(0.10434, 0.0, 0.37439),
                               # Verified by rendering (scripts/diag_cam_rot.py): the previous (-0.5, 0.5, -0.5, 0.5) was a
                               # COMPASS-Carter value that rendered zm_robot's view rolled by 90 deg
                               # (floor on the left); this one is upright and forward-looking.
                               rot=(0.0, 0.70710678, 0.0, 0.70710678),
                               convention="ros"),
)
