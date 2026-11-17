# SPDX-License-Identifier: Apache-2.0
#
# Standalone replacement for COMPASS's mobility_es.wrapper.env_wrapper.RLESEnvWrapper.
# That wrapper adds exactly one thing on top of plain isaaclab.envs.ManagerBasedRLEnv: a
# `.collision_checker` property used by the collision-aware reset/command terms. COMPASS's
# version backs it with an occupancy-map reader; this stub is always "not initialized",
# which is the same degraded (plain uniform sampling, no collision avoidance) behavior
# COMPASS itself falls back to whenever a scene has no occupancy map registered.

from __future__ import annotations

import os

import numpy as np
import torch
import yaml
from PIL import Image

from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg


class NullCollisionChecker:
    """Collision checker that's always "not initialized" - i.e. no collision avoidance."""

    def is_initialized(self) -> bool:
        return False

    def is_in_collision(self, points, distance: float = 0.75):
        raise NotImplementedError("NullCollisionChecker is never initialized; callers must "
                                  "check is_initialized() first (same contract as COMPASS's "
                                  "OccupancyMapCollisionChecker).")


class OccupancyMapCollisionChecker:
    """Collision checker backed by a ROS-format occupancy map (occupancy_map.yaml + png,
    e.g. usd/office/omap/). Precomputes a Euclidean distance-to-nearest-obstacle field, so
    is_in_collision(points, distance) is a single lookup.

    Why this exists (measured with scripts/spawn_test.py): with plain uniform sampling,
    23% of 240 robot spawns in simple_office landed inside furniture, producing 2e5-6e5 N
    contact forces and an instant base_contact termination. Against the 55 measured bad
    spawns, an inflation distance of 0.35 m gives zero misses (it is conservative: it also
    rejects ~40 spawns that happened to be fine, e.g. under tables), 0.3 m misses 1.

    Points are in the env-local frame (callers subtract env_origins). Points outside the
    map count as in collision.
    """

    def __init__(self, yaml_path: str, device: str):
        meta = yaml.safe_load(open(yaml_path))
        img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path),
                                               meta["image"])).convert("L"))
        self.resolution = float(meta["resolution"])
        self.origin_x = float(meta["origin"][0])
        h, w = img.shape
        # ROS map origin is the lower-left corner of the image; the origin y stored in
        # this repo's map is its top edge (verified empirically against measured contacts).
        self.origin_y = float(meta["origin"][1]) - h * self.resolution
        occupied = torch.from_numpy(img < 128).to(device)    # negate=0: dark = occupied
        occ_xy = torch.nonzero(occupied).float()    # [K, 2] (row, col)
        rows = torch.arange(h, device=device).float()
        cols = torch.arange(w, device=device).float()
        grid = torch.stack(torch.meshgrid(rows, cols, indexing="ij"), dim=-1).reshape(-1, 2)
        dist = torch.empty(grid.shape[0], device=device)
        for i in range(0, grid.shape[0], 4096):    # chunked brute-force EDT, one-time cost
            dist[i:i + 4096] = torch.cdist(grid[i:i + 4096], occ_xy).min(dim=1)[0]
        self.dist_m = (dist * self.resolution).reshape(h, w)
        self.h, self.w = h, w
        self.device = device

    def is_initialized(self) -> bool:
        return True

    def is_in_collision(self, points, distance: float = 0.35):
        """points: [N, 2] env-local xy. Returns an int tensor [N]: 1 = in collision."""
        col = torch.floor((points[:, 0] - self.origin_x) / self.resolution).long()
        row = self.h - 1 - torch.floor((points[:, 1] - self.origin_y) / self.resolution).long()
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        d = torch.zeros(points.shape[0], device=points.device)
        d[inside] = self.dist_m[row[inside], col[inside]]
        return ((~inside) | (d < distance)).long()


class ZmRobotEnvWrapper(ManagerBasedRLEnv):
    """Env wrapper of ManagerBasedRLEnv, exposing a `.collision_checker` property."""

    def __init__(self, cfg: ManagerBasedRLEnvCfg, render_mode=None):
        self._collision_checker = NullCollisionChecker()
        # Must exist BEFORE super().__init__: UniformCollisionFreePoseCommand grabs
        # env.collision_checker in its own __init__, which runs inside super().__init__.
        # Falls back to the Null checker (plain uniform sampling, no collision avoidance)
        # if the scene has no omap.
        try:
            usd_path = cfg.scene.environment.spawn.usd_path
            yaml_path = os.path.join(os.path.dirname(usd_path), "omap", "occupancy_map.yaml")
            if os.path.isfile(yaml_path):
                self._collision_checker = OccupancyMapCollisionChecker(yaml_path, cfg.sim.device)
                print(f"[INFO] Collision checker: occupancy map {yaml_path}")
        except AttributeError:
            pass
        super().__init__(cfg, render_mode)

    def reset(self, seed=None, env_ids=None, options=None):
        """Isaac Lab only runs command_manager.compute() at the end of step(), never in
        reset(). So right after reset() the goal-pose command buffer is still all zeros
        ("goal = where the robot is, heading error 0") and the first step() evaluates
        the goal_reached termination against that - every env's first episode (and any
        env_ids passed to an explicit reset) ended instantly with a fake success
        (measured by scripts/diag_spawn.py). Refresh the command + observations here."""
        obs, extras = super().reset(seed=seed, env_ids=env_ids, options=options)
        self.command_manager.compute(dt=0.0)
        self.obs_buf = self.observation_manager.compute(update_history=True)
        return self.obs_buf, extras

    @property
    def collision_checker(self):
        return self._collision_checker
