# SPDX-License-Identifier: Apache-2.0
#
# Pure-numpy re-implementation of the observation terms the residual policy was trained
# with (zm_robot_omni_training/mdp/observations.py). No ROS and no torch in here, so the
# same code can be checked offline against a recorded simulator rollout.

import numpy as np

IMAGE_HW = (320, 512)    # X-Mobility input (H, W)
DEPTH_HW = (64, 104)    # depth observation (H, W)
MAX_DEPTH_M = 5.0
LIDAR_MAX_RANGE_M = 5.0
NUM_SECTORS = 36
ROUTE_SEGMENTS = 10
ROUTE_D_MAX = 1.0


def yaw_from_quat_xyzw(q):
    """Yaw of a planar pose from a (x, y, z, w) quaternion."""
    x, y, z, w = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap_pi(a):
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def route_to_goal(gx, gy, d_max=ROUTE_D_MAX, n_segments=ROUTE_SEGMENTS):
    """Straight line from the robot (0, 0) to the goal (robot frame) as n_segments
    [x1, y1, x2, y2] rows of length <= d_max, padded with the goal point.
    Mirrors upsample_segments() / routing_to_goal_simplified(). Returns [n_segments, 4]."""
    e = np.array([gx, gy], dtype=np.float32)
    dist = float(np.sqrt(np.float32(gx) ** 2 + np.float32(gy) ** 2))
    num = int(np.ceil(dist / d_max))
    t = np.linspace(0.0, 1.0, num + 1, dtype=np.float32)[:, None]
    pts = (1.0 - t) * np.zeros(2, np.float32) + t * e
    seg = np.concatenate([pts[:-1], pts[1:]], axis=1)
    if num < n_segments:
        pad = np.tile(np.concatenate([e, e])[None, :], (n_segments - num, 1))
        seg = np.concatenate([seg, pad], axis=0)
    else:
        seg = seg[:n_segments]
    return seg.astype(np.float32)


def scan_to_body_points(ranges, angle_min, angle_increment, range_min, range_max, tx, ty, tyaw):
    """LaserScan -> hit points in the robot base frame. (tx, ty, tyaw) is the planar pose
    of the LiDAR frame in the base frame. Returns (xy [N, 2], valid [N])."""
    r = np.asarray(ranges, dtype=np.float32)
    a = angle_min + angle_increment * np.arange(r.shape[0], dtype=np.float32)
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    rr = np.where(valid, r, 0.0)
    lx, ly = rr * np.cos(a), rr * np.sin(a)
    c, s = np.cos(tyaw), np.sin(tyaw)
    x = tx + c * lx - s * ly
    y = ty + s * lx + c * ly
    return np.stack([x, y], axis=1).astype(np.float32), valid


def lidar_sector_ranges(points_xy, valid, num_sectors=NUM_SECTORS, max_range=LIDAR_MAX_RANGE_M):
    """Nearest obstacle distance (from the base origin) in each of num_sectors equal bearing
    sectors, sector 0 starting at -pi, normalised to [0, 1] (1 = nothing within max_range).
    Mirrors lidar_polar_ranges(). points_xy: all scans merged, in the base frame."""
    out = np.full(num_sectors, max_range, dtype=np.float32)
    if points_xy.shape[0]:
        p = points_xy[valid]
        rng = np.minimum(np.hypot(p[:, 0], p[:, 1]), max_range)
        ang = np.arctan2(p[:, 1], p[:, 0])
        idx = np.clip(((ang + np.pi) / (2.0 * np.pi) * num_sectors).astype(np.int64), 0, num_sectors - 1)
        np.minimum.at(out, idx, rng)
    return out / np.float32(max_range)


def _adaptive_bounds(n_in, n_out):
    i = np.arange(n_out)
    return (i * n_in) // n_out, -((-(i + 1) * n_in) // n_out)    # floor, ceil


def adaptive_avg_pool2d(img, out_hw):
    """Same windows as torch.nn.functional.adaptive_avg_pool2d (they may overlap)."""
    h, w = img.shape
    ii = np.zeros((h + 1, w + 1), dtype=np.float64)
    ii[1:, 1:] = np.cumsum(np.cumsum(img.astype(np.float64), axis=0), axis=1)
    y0, y1 = _adaptive_bounds(h, out_hw[0])
    x0, x1 = _adaptive_bounds(w, out_hw[1])
    s = ii[y1][:, x1] - ii[y0][:, x1] - ii[y1][:, x0] + ii[y0][:, x0]
    area = (y1 - y0)[:, None] * (x1 - x0)[None, :]
    return (s / area).astype(np.float32)


def depth_to_obs(depth_m, max_depth=MAX_DEPTH_M, out_hw=DEPTH_HW, render_hw=IMAGE_HW):
    """Depth in metres [H, W] -> [1, 64, 104] float32 on [0, 255] (255 = >= max_depth or no
    return), truncated to whole numbers. Mirrors camera_depth_u8() followed by the uint8 cast
    the SB3 wrapper applies to image observations (verified on a recorded rollout: the policy
    saw integer values). Inputs that are not 320x512 are resized first."""
    d = np.asarray(depth_m, dtype=np.float32)
    d = np.nan_to_num(d, nan=max_depth, posinf=max_depth, neginf=0.0)
    d = np.clip(d, 0.0, max_depth)
    if d.shape != tuple(render_hw):
        import cv2
        d = cv2.resize(d, (render_hw[1], render_hw[0]), interpolation=cv2.INTER_AREA)
    out = adaptive_avg_pool2d(d, out_hw) / np.float32(max_depth) * np.float32(255.0)
    return np.floor(np.clip(out, 0.0, 255.0))[None].astype(np.float32)


def rgb_to_input(rgb_hwc_uint8, image_hw=IMAGE_HW):
    """RGB uint8 [H, W, 3] -> [1, 3, 320, 512] float32 on [0, 1] (X-Mobility input)."""
    img = rgb_hwc_uint8
    if img.shape[:2] != tuple(image_hw):
        import cv2
        img = cv2.resize(img, (image_hw[1], image_hw[0]), interpolation=cv2.INTER_LINEAR)
    return np.ascontiguousarray(img.transpose(2, 0, 1)[None].astype(np.float32) / np.float32(255.0))
