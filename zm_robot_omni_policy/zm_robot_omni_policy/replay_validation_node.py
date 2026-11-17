#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Deployment validation by replay: publish a rollout recorded in the Isaac Lab training
# environment (scripts/record_deploy_ref.py in zm_robot_omni_training) as the ROS2 topics
# the real robot provides, step by step, and compare what zm_robot_omni_policy publishes
# with what the PyTorch policy produced on the same data.
#
# Lock-step: for each recorded step, every message carries the same stamp; the node waits
# for the policy's cmd_vel_stamped with that stamp before publishing the next step.
#
# This is OPEN-LOOP: the sensor data are the recording's, so it checks the ROS2 interface,
# the pre-processing and the TensorRT engines - not closed-loop driving.

import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from zm_robot_omni_policy import preprocess as pp
from zm_robot_omni_policy.interface import (DEBUG_FIELDS, DEPTH_TOPIC, GOAL_TOPIC, ODOM_TOPIC, RGB_TOPIC,
                                            SCAN_TOPICS)
from zm_robot_omni_policy.paths import resolve_model_dir

POLICY_NODE = 'zm_robot_omni_policy'
ODOM_FRAME, BASE_FRAME, CAMERA_FRAME = 'odom', 'base_link', 'kinect_v2'
LIDAR_FRAMES = ('sick_lidar0', 'sick_lidar1')
RAY_INCREMENT = np.radians(2.0)    # the simulator's LiDAR pattern: 180 rays, 2 deg apart


def quat_msg_from_yaw(msg_q, yaw):
    msg_q.x, msg_q.y, msg_q.z, msg_q.w = 0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))


class ReplayValidation(Node):

    def __init__(self):
        super().__init__('zm_robot_omni_replay_validation')
        self.declare_parameter('ref_path', '')
        self.declare_parameter('report_path', '')
        self.declare_parameter('step_timeout', 10.0)
        self.declare_parameter('tolerance', 0.02)    # pass threshold on |cmd error| (m/s, rad/s)
        # lockstep: wait for the command of each step (policy in sync_mode 'stamp', exact comparison).
        # realtime: publish at the recorded rate without waiting (policy in sync_mode 'timer', smoke test).
        self.declare_parameter('mode', 'lockstep')
        ref = self.get_parameter('ref_path').value or os.path.join(resolve_model_dir(), 'validation',
                                                                   'ref_hybrid.npz')
        self.get_logger().info(f'loading reference rollout {ref}')
        self.ref_path = ref
        self.d = dict(np.load(ref))
        self.n = len(self.d['final'])
        self.dt = float(self.d['step_dt'])

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.rgb_pub = self.create_publisher(Image, RGB_TOPIC, qos)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, qos)
        self.scan_pubs = [self.create_publisher(LaserScan, t, qos) for t in SCAN_TOPICS]
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, qos)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.cmd = {}    # stamp_ns -> (vx, vy, wz)
        self.cmd_times = []    # arrival time of every command (realtime mode)
        self.last_cmd = None
        self.last_debug = None
        self.create_subscription(TwistStamped, f'/{POLICY_NODE}/cmd_vel_stamped', self._on_cmd, 50)
        self.create_subscription(Float32MultiArray, f'/{POLICY_NODE}/debug', self._on_debug, 50)

    def _on_cmd(self, msg):
        t = msg.twist
        self.last_cmd = (t.linear.x, t.linear.y, t.angular.z)
        self.cmd[Time.from_msg(msg.header.stamp).nanoseconds] = self.last_cmd
        self.cmd_times.append(time.monotonic())

    def _on_debug(self, msg):
        self.last_debug = np.asarray(msg.data, np.float32)

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    # ------------------------------------------------------------------ message builders
    def _pose(self, k):
        x, y = self.d['root_pos'][k][:2]
        return float(x), float(y), pp.yaw_from_quat_xyzw(self.d['root_quat'][k])

    def _lidar_mount(self, i, k=0):
        """LiDAR frame in the base frame (x, y, z, yaw) from the recording."""
        x, y, yaw = self._pose(k)
        lp = self.d[f'lidar{i}_pos'][k]
        c, s = np.cos(-yaw), np.sin(-yaw)
        dx, dy = lp[0] - x, lp[1] - y
        return (float(c * dx - s * dy), float(s * dx + c * dy), float(lp[2] - self.d['root_pos'][k][2]),
                pp.wrap_pi(pp.yaw_from_quat_xyzw(self.d[f'lidar{i}_quat'][k]) - yaw))

    def _publish_static_tf(self):
        tfs = []
        for i, frame in enumerate(LIDAR_FRAMES):
            x, y, z, yaw = self._lidar_mount(i)
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = BASE_FRAME, frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            quat_msg_from_yaw(t.transform.rotation, yaw)
            tfs.append(t)
            self.get_logger().info(f'{frame} in {BASE_FRAME}: x={x:.4f} y={y:.4f} z={z:.4f} yaw={yaw:.4f}')
        self.static_tf_pub.sendTransform(tfs)

    def _publish_step(self, k, stamp):
        d = self.d
        x, y, yaw = self._pose(k)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, ODOM_FRAME, BASE_FRAME
        tf.transform.translation.x, tf.transform.translation.y = x, y
        quat_msg_from_yaw(tf.transform.rotation, yaw)
        self.tf_pub.sendTransform(tf)

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, ODOM_FRAME, BASE_FRAME
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        quat_msg_from_yaw(od.pose.pose.orientation, yaw)
        vx, vy = d['lin_vel_w'][k][:2]
        c, s = np.cos(-yaw), np.sin(-yaw)    # world -> body
        od.twist.twist.linear.x, od.twist.twist.linear.y = float(c * vx - s * vy), float(s * vx + c * vy)
        od.twist.twist.angular.z = float(d['ang_vel_w'][k][2])
        self.odom_pub.publish(od)

        for i, frame in enumerate(LIDAR_FRAMES):
            lp, hits = d[f'lidar{i}_pos'][k], d[f'lidar{i}_hits'][k]
            lyaw = pp.yaw_from_quat_xyzw(d[f'lidar{i}_quat'][k])
            fin = np.isfinite(hits).all(axis=1)
            sc = LaserScan()
            sc.header.stamp, sc.header.frame_id = stamp, frame
            # The simulator's rays are world-aligned (first ray at -180 deg in the world), so in
            # the sensor frame the sweep starts at -pi - sensor_yaw.
            sc.angle_min = float(-np.pi - lyaw)
            sc.angle_increment = float(RAY_INCREMENT)
            sc.angle_max = float(sc.angle_min + RAY_INCREMENT * (len(hits) - 1))
            sc.range_min, sc.range_max = 0.0, 100.0
            r = np.where(fin, np.hypot(hits[:, 0] - lp[0], hits[:, 1] - lp[1]), np.inf)
            sc.ranges = r.astype(np.float32).tolist()
            self.scan_pubs[i].publish(sc)

        dm = Image()
        dm.header.stamp, dm.header.frame_id = stamp, CAMERA_FRAME
        depth = np.ascontiguousarray(d['depth'][k], np.float32)
        dm.height, dm.width, dm.encoding, dm.is_bigendian, dm.step = depth.shape[0], depth.shape[1], '32FC1', 0, depth.shape[1] * 4
        dm.data = depth.tobytes()
        self.depth_pub.publish(dm)

        im = Image()
        im.header.stamp, im.header.frame_id = stamp, CAMERA_FRAME
        rgb = np.ascontiguousarray(d['rgb'][k])
        im.height, im.width, im.encoding, im.is_bigendian, im.step = rgb.shape[0], rgb.shape[1], 'rgb8', 0, rgb.shape[1] * 3
        im.data = rgb.tobytes()
        self.rgb_pub.publish(im)

    def _publish_goal(self, k, stamp):
        g = PoseStamped()
        g.header.stamp, g.header.frame_id = stamp, ODOM_FRAME
        g.pose.position.x, g.pose.position.y = float(self.d['goal_pos_w'][k][0]), float(self.d['goal_pos_w'][k][1])
        quat_msg_from_yaw(g.pose.orientation, float(self.d['goal_heading_w'][k]))
        self.goal_pub.publish(g)

    def _reference_inconsistent(self):
        """Steps where the SIMULATOR's own observation was not self-consistent, so its command
        is not a valid reference for a real sensor stream:
          * the first step of the run: the simulator fed X-Mobility an all-zero route there
            (checked with scripts/check_engines_offline.py --zero_route_first_step);
          * the first step after an episode reset: the LiDAR was still at its pre-reset pose
            (recorded sensor pose several metres from the robot), i.e. stale ray hits.
        Returns (bad [n], excluded [n]); `excluded` also covers the next two steps, because
        the command of a bad step feeds the next ones (previous-command input, RSSM state)."""
        ref = np.array(self._lidar_mount(0, 0)[:2] + self._lidar_mount(1, 0)[:2])
        bad = np.zeros(self.n, bool)
        bad[0] = True
        for k in range(self.n):
            cur = np.array(self._lidar_mount(0, k)[:2] + self._lidar_mount(1, k)[:2])
            bad[k] |= bool(np.abs(cur - ref).max() > 0.01)
        excluded = bad.copy()
        for k in np.nonzero(bad)[0]:
            excluded[k:k + 3] = True
        return bad, excluded

    # ------------------------------------------------------------------ run
    def run(self):
        self.get_logger().info('waiting for the policy node ...')
        t0 = time.monotonic()
        while self.rgb_pub.get_subscription_count() == 0 or self.goal_pub.get_subscription_count() == 0:
            self._spin_for(0.1)
            if time.monotonic() - t0 > 120.0:
                self.get_logger().error('policy node did not appear within 120 s')
                return False
        self._publish_static_tf()
        self._spin_for(1.0)

        d, timeout = self.d, float(self.get_parameter('step_timeout').value)
        realtime = self.get_parameter('mode').value == 'realtime'
        t_start = time.monotonic()
        got = np.full((self.n, 3), np.nan, np.float32)
        dbg = np.full((self.n, len(DEBUG_FIELDS)), np.nan, np.float32)
        latency = np.full(self.n, np.nan)
        for k in range(self.n):
            stamp_ns = int(round((k + 1) * self.dt * 1e9))
            stamp = Time(nanoseconds=stamp_ns).to_msg()
            if d['ep_step'][k] == 0:    # new episode: new goal -> the policy resets its state
                self._publish_goal(k, stamp)
                self._spin_for(0.3)
            self.last_debug = None
            t_pub = time.monotonic()
            self._publish_step(k, stamp)
            if realtime:    # no waiting: the policy runs on its own timer
                self._spin_for(max(0.0, self.dt - (time.monotonic() - t_pub)))
                if self.last_cmd is not None:
                    got[k] = self.last_cmd
                if self.last_debug is not None and len(self.last_debug) == len(DEBUG_FIELDS):
                    dbg[k] = self.last_debug
                continue
            while stamp_ns not in self.cmd and time.monotonic() - t_pub < timeout:
                rclpy.spin_once(self, timeout_sec=0.002)
            if stamp_ns not in self.cmd:
                self.get_logger().error(f'step {k}: no command within {timeout} s')
                break
            latency[k] = time.monotonic() - t_pub
            self._spin_for(0.01)    # let the debug message of this step arrive
            got[k] = self.cmd[stamp_ns]
            if self.last_debug is not None and len(self.last_debug) == len(DEBUG_FIELDS):
                dbg[k] = self.last_debug
            if k % 25 == 0:
                self.get_logger().info(f'step {k + 1}/{self.n}: cmd {np.round(got[k], 3).tolist()} '
                                       f'ref {np.round(d["final"][k], 3).tolist()}')
        if realtime:
            return self._report_realtime(got, time.monotonic() - t_start)
        return self._report(got, dbg, latency)

    def _report_realtime(self, got, elapsed):
        d = self.d
        n_before = len(self.cmd_times)
        self._spin_for(1.5)    # sensors stopped: the policy must fall back to zero velocity
        stale_stop = self.last_cmd is not None and all(abs(v) < 1e-9 for v in self.last_cmd)
        ok = np.isfinite(got).all(axis=1)
        err = np.abs(got - d['final'])[ok]
        rep = {
            'mode': 'realtime (policy sync_mode=timer); commands are NOT step-aligned with the recording',
            'steps_total': self.n, 'steps_with_command': int(ok.sum()),
            'commands_received': n_before, 'command_rate_hz': float(n_before / elapsed),
            'cmd_vs_pytorch_mean_abs': float(err.mean()) if len(err) else None,
            'cmd_vs_pytorch_median_abs': float(np.median(err)) if len(err) else None,
            'zero_velocity_after_sensors_stop': bool(stale_stop),
        }
        rep['passed'] = bool(ok.mean() > 0.9 and 8.0 <= rep['command_rate_hz'] <= 12.0 and stale_stop)
        path = self.get_parameter('report_path').value or os.path.join(os.path.dirname(self.ref_path),
                                                                       'replay_report_realtime.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=1)
        (self.get_logger().info if rep['passed'] else self.get_logger().error)(
            f"{'PASS' if rep['passed'] else 'FAIL'} (realtime smoke test): {json.dumps(rep)}; report -> {path}")
        return rep['passed']

    def _report(self, got, dbg, latency):
        d = self.d
        done = int(np.isfinite(got).all(axis=1).sum())
        col = {name: i for i, name in enumerate(DEBUG_FIELDS)}
        sl = lambda prefix: [col[f'{prefix}_{a}'] for a in ('vx', 'vy', 'wz')]    # noqa: E731

        bad, excluded = self._reference_inconsistent()
        keep = ~excluded[:done]

        def stats(a, b):
            e = np.abs(np.asarray(a, np.float64) - np.asarray(b, np.float64))[:done]
            ek = e[keep]
            return {'max_abs': float(np.nanmax(ek)), 'mean_abs': float(np.nanmean(ek)),
                    'per_axis_max_abs': np.nanmax(ek, axis=0).tolist() if e.ndim == 2 else None,
                    'all_steps_max_abs': float(np.nanmax(e)), 'all_steps_mean_abs': float(np.nanmean(e))}

        goal_ref = d['obs_route'][:, -2:]    # last route point = goal in the robot frame
        gh_ref = np.arctan2(d['obs_goal_heading'][:, 1], d['obs_goal_heading'][:, 0])
        gh_err = np.abs((dbg[:, col['goal_heading']] - gh_ref + np.pi) % (2 * np.pi) - np.pi)[:done][keep]
        lidar_cols = [col[f'lidar_{i}'] for i in range(pp.NUM_SECTORS)]
        tol = float(self.get_parameter('tolerance').value)
        rep = {
            'reference': self.ref_path,
            'steps_total': self.n,
            'steps_compared': done,
            'episodes': int(d['episode'].max()) + 1,
            'units': 'vx, vy in m/s; wz in rad/s',
            'note': 'max_abs / mean_abs are over the steps whose simulator reference is self-consistent; '
                    'all_steps_* include the excluded ones (see reference_inconsistent_steps).',
            'reference_inconsistent_steps': {
                'first_step_of_run_zero_route_or_stale_lidar_after_reset': np.nonzero(bad)[0].tolist(),
                'excluded_from_pass_criterion_incl_two_carry_over_steps': np.nonzero(excluded)[0].tolist(),
                'steps_used': int(keep.sum()),
            },
            'final_cmd_vs_pytorch': stats(got, d['final']),
            'base_cmd_vs_pytorch': stats(dbg[:, sl('base')], d['base']),
            'residual_vs_pytorch': stats(dbg[:, sl('residual')], d['residual']),
            'observations_vs_simulator': {
                'base_speed': stats(dbg[:, col['base_speed']], d['obs_base_speed'][:, 0]),
                'base_yaw_rate': stats(dbg[:, col['base_yaw_rate']], d['obs_base_yaw_rate'][:, 0]),
                'goal_xy_robot_frame_m': stats(dbg[:, [col['goal_x'], col['goal_y']]], goal_ref),
                'goal_heading_rad': {'max_abs': float(np.nanmax(gh_err)), 'mean_abs': float(np.nanmean(gh_err))},
                'lidar_sectors_normalised': stats(dbg[:, lidar_cols], d['obs_lidar_ranges']),
                'depth_obs_mean': stats(dbg[:, col['depth_obs_mean']], d['obs_depth_img'].reshape(self.n, -1).mean(1)),
                'step_in_episode': stats(dbg[:, col['step_in_episode']], d['ep_step']),
            },
            'policy_infer_ms': {'note': 'pre-processing + both engines, measured inside the policy node',
                                'mean': float(np.nanmean(dbg[:done, col['infer_ms']])),
                                'p95': float(np.nanpercentile(dbg[:done, col['infer_ms']], 95)),
                                'max': float(np.nanmax(dbg[:done, col['infer_ms']]))},
            'publish_to_cmd_latency_ms': {'mean': float(1000 * np.nanmean(latency[:done])),
                                          'p95': float(1000 * np.nanpercentile(latency[:done], 95)),
                                          'max': float(1000 * np.nanmax(latency[:done]))},
            'tolerance': tol,
        }
        rep['passed'] = bool(done == self.n and rep['final_cmd_vs_pytorch']['max_abs'] <= tol)
        path = self.get_parameter('report_path').value or os.path.join(os.path.dirname(self.ref_path),
                                                                       'replay_report.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=1)
        np.savez_compressed(os.path.splitext(path)[0] + '_trace.npz', cmd=got, debug=dbg, latency=latency,
                            ref_final=d['final'], ref_base=d['base'], ref_residual=d['residual'],
                            debug_fields=np.array(DEBUG_FIELDS))
        log = self.get_logger().info if rep['passed'] else self.get_logger().error
        log(f"{'PASS' if rep['passed'] else 'FAIL'}: {done}/{self.n} steps replayed, {int(keep.sum())} with a consistent "
            f"reference: final cmd max |err| {rep['final_cmd_vs_pytorch']['max_abs']:.5f} (tolerance {tol}), "
            f"mean {rep['final_cmd_vs_pytorch']['mean_abs']:.5f}; all steps max "
            f"{rep['final_cmd_vs_pytorch']['all_steps_max_abs']:.5f}; "
            f"inference {rep['policy_infer_ms']['mean']:.1f} ms mean; report -> {path}")
        return rep['passed']


def main(args=None):
    rclpy.init(args=args)
    node = ReplayValidation()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
