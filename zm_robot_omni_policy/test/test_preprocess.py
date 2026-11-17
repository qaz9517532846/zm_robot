# SPDX-License-Identifier: Apache-2.0
"""Offline check of the numpy pre-processing against the observations recorded in the
training simulator (skipped when the reference rollout is not present)."""
import os

import numpy as np
import pytest

from zm_robot_omni_policy import preprocess as pp

REF = os.path.join(os.path.dirname(__file__), '..', 'models', 'validation', 'ref_hybrid.npz')


def test_route_padding():
    r = pp.route_to_goal(2.5, 0.0)
    assert r.shape == (10, 4)
    np.testing.assert_allclose(r[0], [0, 0, 2.5 / 3, 0], atol=1e-6)
    np.testing.assert_allclose(r[3:], np.tile([2.5, 0, 2.5, 0], (7, 1)), atol=1e-6)


def test_adaptive_pool_matches_uniform_case():
    img = np.arange(16, dtype=np.float32).reshape(4, 4)
    np.testing.assert_allclose(pp.adaptive_avg_pool2d(img, (2, 2)), [[2.5, 4.5], [10.5, 12.5]])


@pytest.mark.skipif(not os.path.exists(REF), reason='reference rollout not available')
def test_against_recorded_observations():
    d = np.load(REF)
    for k in range(0, len(d['final']), 7):
        yaw = pp.yaw_from_quat_xyzw(d['root_quat'][k])
        px, py = d['root_pos'][k][:2]
        c, s = np.cos(-yaw), np.sin(-yaw)
        gx, gy = d['goal_pos_w'][k][:2] - np.array([px, py])
        np.testing.assert_allclose(pp.route_to_goal(c * gx - s * gy, s * gx + c * gy).reshape(-1),
                                   d['obs_route'][k], atol=1e-5)
        pts, val = [], []
        for i in (0, 1):
            lp, hits = d[f'lidar{i}_pos'][k], d[f'lidar{i}_hits'][k]
            lyaw = pp.yaw_from_quat_xyzw(d[f'lidar{i}_quat'][k])
            fin = np.isfinite(hits).all(axis=1)
            r = np.where(fin, np.hypot(hits[:, 0] - lp[0], hits[:, 1] - lp[1]), np.inf)
            dx, dy = lp[0] - px, lp[1] - py
            p, v = pp.scan_to_body_points(r, -np.pi - lyaw, np.radians(2.0), 0.0, 100.0,
                                          c * dx - s * dy, s * dx + c * dy, pp.wrap_pi(lyaw - yaw))
            pts.append(p)
            val.append(v)
        np.testing.assert_allclose(pp.lidar_sector_ranges(np.concatenate(pts), np.concatenate(val)),
                                   d['obs_lidar_ranges'][k], atol=1e-5)
        diff = np.abs(pp.depth_to_obs(d['depth'][k]) - d['obs_depth_img'][k])
        assert diff.max() <= 1.0 and (diff > 0).mean() < 1e-3    # float rounding at integer edges only
