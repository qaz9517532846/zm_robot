#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# zm_robot omnidirectional navigation policy (residual SAC on a frozen X-Mobility base).
#
#   final cmd (vx, vy, wz) = clip( X-Mobility base [vx, 0, wz]  +  SAC residual )
#
# Two TensorRT engines run per control step (see models/export_meta.json):
#   xmobility.engine                  RGB + speed + route + previous cmd + RSSM state
#                                     -> 2048-d policy state, 6-d twist, new RSSM state
#   sac_residual_actor_<name>.engine  robot state + route + goal heading + 36 LiDAR sectors
#                                     + X-Mobility output + 64x104 depth -> residual
#
# Inputs : RGB image, depth image, two LaserScans, /odom, /goal_pose, TF (goal frame and the
#          LiDAR frames -> base frame), optional /emergency_stop (std_msgs/Bool from any
#          external safety source; nothing in this package publishes it).
# Output : /cmd_vel (geometry_msgs/Twist, linear.x / linear.y / angular.z).
#
# sync_mode "timer" (default, real robot): run at control_rate on the latest messages.
# sync_mode "stamp" (simulation / replay): run exactly once per sensor time stamp, when the
#          RGB, depth, both scans and odom all carry the same stamp - deterministic.

import os
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from zm_robot_omni_policy import preprocess as pp
from zm_robot_omni_policy.interface import (CMD_TOPIC, DEPTH_TOPIC, ESTOP_TOPIC, GOAL_TOPIC, ODOM_TOPIC,
                                            RGB_TOPIC, SCAN_TOPICS)
from zm_robot_omni_policy.paths import resolve_model_dir
from zm_robot_omni_policy.trt_runner import TrtEngine


class OmniPolicyNode(Node):

    def __init__(self):
        super().__init__('zm_robot_omni_policy')
        p = self.declare_parameter
        p('model_dir', '')
        p('actor', 'hybrid')    # which sac_residual_actor_<actor>.engine to run
        p('sync_mode', 'timer')    # 'timer' | 'stamp'
        p('control_rate', 10.0)    # Hz; the policy was trained at 10 Hz (0.1 s control period)
        p('base_frame', 'base_link')
        p('sensor_qos', 'best_effort')    # 'best_effort' | 'reliable'
        p('max_linear_x', 0.6)
        p('max_linear_y', 0.6)
        p('max_angular_z', 1.57)
        p('lidar_max_range', pp.LIDAR_MAX_RANGE_M)
        p('max_depth', pp.MAX_DEPTH_M)
        p('stale_timeout', 0.5)    # s, timer mode: stop if a sensor has been silent this long
        p('stop_at_goal', True)
        p('goal_distance_tolerance', 0.5)    # m   (training success: < 0.5 m)
        p('goal_heading_tolerance', 0.5)    # rad (training success: < 0.5 rad)
        # Acceleration limits on the published /cmd_vel (timer mode). Training applied exactly
        # these limits between the policy's command and the robot (2.0 m/s^2, 3.14 rad/s^2), so
        # the policy never saw its velocity command take effect as a step. 0 disables the ramp.
        p('max_linear_accel', 2.0)
        p('max_angular_accel', 3.14)
        p('cmd_publish_rate', 50.0)    # Hz of /cmd_vel while ramping towards the policy's command
        g = lambda n: self.get_parameter(n).value    # noqa: E731
        self.sync_mode = g('sync_mode')
        self.base_frame = g('base_frame')
        self.high = np.array([g('max_linear_x'), g('max_linear_y'), g('max_angular_z')], np.float32)
        self.low = -self.high

        # Engines first: by the time the subscriptions exist the node can actually infer.
        model_dir = resolve_model_dir(g('model_dir'))
        actor_path = os.path.join(model_dir, f"sac_residual_actor_{g('actor')}.engine")
        self.get_logger().info(f'loading engines from {model_dir} (actor: {g("actor")})')
        self.xmob = TrtEngine(os.path.join(model_dir, 'xmobility.engine'))
        self.actor = TrtEngine(actor_path)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rgb = self.depth = self.odom = self.goal = None    # (stamp_ns, rx_time, data)
        self.scans = [None, None]
        self.estop = False
        self.reached = False
        self.last_stamp_done = None
        self._reset_state()
        self.max_lin_acc = float(g('max_linear_accel'))
        self.max_ang_acc = float(g('max_angular_accel'))
        self.ramp_dt = 1.0 / max(float(g('cmd_publish_rate')), 1e-3)
        # stamp mode publishes the raw policy command (it is what the replay validation compares)
        self.ramp_enabled = self.sync_mode != 'stamp' and self.max_lin_acc > 0.0 and self.max_ang_acc > 0.0
        self.target = np.zeros(3, np.float32)    # the policy's latest command
        self.applied = np.zeros(3, np.float32)    # what was last published on /cmd_vel
        self.ramping = False
        self.silent_after_stop = False    # goal reached: ramp to zero, then stop publishing

        qos = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
               if g('sensor_qos') == 'reliable' else qos_profile_sensor_data)
        self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos)
        self.create_subscription(LaserScan, SCAN_TOPICS[0], lambda m: self._on_scan(0, m), qos)
        self.create_subscription(LaserScan, SCAN_TOPICS[1], lambda m: self._on_scan(1, m), qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos)
        self.create_subscription(PoseStamped, GOAL_TOPIC, self._on_goal, 10)
        self.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.cmd_stamped_pub = self.create_publisher(TwistStamped, '~/cmd_vel_stamped', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '~/debug', 10)

        if self.sync_mode == 'stamp':
            self.create_timer(0.005, self._try_stamp_step)    # poll: TF may arrive after the sensors
        else:
            self.create_timer(1.0 / float(g('control_rate')), self._timer_step)
            if self.ramp_enabled:
                self.create_timer(self.ramp_dt, self._ramp_step)
        self.get_logger().info(
            f'ready (sync_mode={self.sync_mode}, cmd_vel ramp: '
            + (f'{self.max_lin_acc} m/s^2, {self.max_ang_acc} rad/s^2 at {1.0 / self.ramp_dt:.0f} Hz)'
               if self.ramp_enabled else 'off)'))

    # ------------------------------------------------------------------ state / callbacks
    def _reset_state(self):
        self.history = np.zeros((1, 1024), np.float32)
        self.sample = np.zeros((1, 512), np.float32)
        self.prev_cmd = np.zeros(3, np.float32)    # last command applied (vx, vy, wz)
        self.step_in_episode = 0
        self.reached = False
        self.silent_after_stop = False

    def _rx(self, msg):
        return (Time.from_msg(msg.header.stamp).nanoseconds, time.monotonic())

    def _on_rgb(self, msg):
        self.rgb = (*self._rx(msg), self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))

    def _on_depth(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if d.dtype == np.uint16:    # 16UC1: millimetres, 0 = no return
            d = np.where(d == 0, np.inf, d.astype(np.float32) / 1000.0)
        self.depth = (*self._rx(msg), np.asarray(d, np.float32))

    def _on_scan(self, i, msg):
        self.scans[i] = (*self._rx(msg), msg)

    def _on_odom(self, msg):
        t = msg.twist.twist
        self.odom = (*self._rx(msg), (float(np.hypot(t.linear.x, t.linear.y)), float(t.angular.z)))

    def _on_goal(self, msg):
        self.goal = msg
        self._reset_state()    # new goal = new episode: zero the RSSM state and previous command
        self.get_logger().info(
            f'new goal in "{msg.header.frame_id}": ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('emergency stop asserted: publishing zero velocity')
        self.estop = bool(msg.data)
        if self.estop and (self.ramping or self.applied.any()):
            self._stop_now()    # do not wait for the next control tick, and never ramp an e-stop

    # ------------------------------------------------------------------ scheduling
    def _inputs(self):
        return [self.rgb, self.depth, self.scans[0], self.scans[1], self.odom]

    def _timer_step(self):
        if self.goal is None or self.reached:
            return    # no goal, or goal reached: publish nothing until a new goal arrives
        if self.estop:
            self._publish(np.zeros(3, np.float32), None, immediate=True)
            return
        names = ['rgb', 'depth', 'scan0', 'scan1', 'odom']
        missing = [n for n, v in zip(names, self._inputs()) if v is None]
        if missing:
            self.get_logger().info(f'waiting for inputs: {missing}', throttle_duration_sec=2.0)
            return
        now, limit = time.monotonic(), float(self.get_parameter('stale_timeout').value)
        stale = [n for n, v in zip(names, self._inputs()) if now - v[1] > limit]
        if stale:
            self.get_logger().warn(f'stale inputs {stale}: publishing zero velocity', throttle_duration_sec=2.0)
            self._publish(np.zeros(3, np.float32), None, immediate=True)
            return
        self._step(Time(), None)

    def _try_stamp_step(self):
        if self.goal is None or self.reached or any(v is None for v in self._inputs()):
            return
        stamps = {v[0] for v in self._inputs()}
        if len(stamps) != 1:
            return
        stamp_ns = stamps.pop()
        if stamp_ns == self.last_stamp_done:
            return
        if self._step(Time(nanoseconds=stamp_ns, clock_type=self.get_clock().clock_type), stamp_ns):
            self.last_stamp_done = stamp_ns

    # ------------------------------------------------------------------ one control step
    def _lookup_planar(self, frame, when):
        tf = self.tf_buffer.lookup_transform(self.base_frame, frame, when)
        q = tf.transform.rotation
        return tf, (tf.transform.translation.x, tf.transform.translation.y,
                    pp.yaw_from_quat_xyzw((q.x, q.y, q.z, q.w)))

    def _step(self, when, stamp_ns):
        """Returns True if a command was published for this time."""
        try:
            goal_tf, _ = self._lookup_planar(self.goal.header.frame_id, when)
            lidar_tf = [self._lookup_planar(s[2].header.frame_id, Time())[1] for s in self.scans]
        except TransformException as ex:
            self.get_logger().warn(f'TF not available: {ex}', throttle_duration_sec=2.0)
            return False
        g = do_transform_pose(self.goal.pose, goal_tf)
        gx, gy = g.position.x, g.position.y
        o = g.orientation
        goal_heading = pp.yaw_from_quat_xyzw((o.x, o.y, o.z, o.w))

        if self.estop:
            self._publish(np.zeros(3, np.float32), stamp_ns, immediate=True)
            return True
        if (self.get_parameter('stop_at_goal').value
                and np.hypot(gx, gy) < self.get_parameter('goal_distance_tolerance').value
                and abs(goal_heading) < self.get_parameter('goal_heading_tolerance').value):
            # Goal reached: bring the base to a stop (ramping down within the acceleration limit,
            # ending with a zero-velocity command), then stay silent - no more cmd_vel - until a
            # new goal arrives (_on_goal() clears `reached`).
            self.reached = True
            self.silent_after_stop = True
            self._publish(np.zeros(3, np.float32), stamp_ns)
            self.get_logger().info(f'goal reached (dist {np.hypot(gx, gy):.2f} m, heading {goal_heading:.2f} rad): '
                                   'stopping, cmd_vel publishing paused until a new goal')
            return True

        t0 = time.perf_counter()
        base_speed, base_yaw_rate = self.odom[2]
        route = pp.route_to_goal(gx, gy)
        max_range = float(self.get_parameter('lidar_max_range').value)
        pts, valid = [], []
        for (_, _, scan), (tx, ty, tyaw) in zip(self.scans, lidar_tf):
            xy, ok = pp.scan_to_body_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                            scan.range_min, scan.range_max, tx, ty, tyaw)
            pts.append(xy)
            valid.append(ok)
        lidar = pp.lidar_sector_ranges(np.concatenate(pts), np.concatenate(valid), max_range=max_range)
        depth_obs = pp.depth_to_obs(self.depth[2], max_depth=float(self.get_parameter('max_depth').value))

        xo = self.xmob({
            'image': pp.rgb_to_input(self.rgb[2]),
            'speed': np.array([[base_speed]], np.float32),
            'route': route[None],
            'action_input': np.array([[self.prev_cmd[0], self.prev_cmd[1], 0, 0, 0, self.prev_cmd[2]]], np.float32),
            'history_input': self.history,
            'sample_input': self.sample,
        })
        self.history, self.sample = xo['history_output'], xo['sample_output']
        a6 = xo['action_output'].reshape(6)
        # X-Mobility was trained on differential-drive robots: it never commands vy.
        base = np.array([np.clip(a6[0], -0.6, 0.6), 0.0, np.clip(a6[5], -1.57, 1.57)], np.float32)
        residual = self.actor({
            'actions': self.prev_cmd[None],
            'base_speed': np.array([[base_speed]], np.float32),
            'base_yaw_rate': np.array([[base_yaw_rate]], np.float32),
            'goal_heading': np.array([[np.cos(goal_heading), np.sin(goal_heading)]], np.float32),
            'lidar_ranges': lidar[None],
            'route': route.reshape(1, -1),
            'xmobility': np.concatenate([xo['policy_state'].reshape(-1), base])[None],
            'depth_img': depth_obs[None],
        })['residual'].reshape(3)
        final = np.clip(base + residual, self.low, self.high).astype(np.float32)
        infer_ms = 1000.0 * (time.perf_counter() - t0)

        dbg = Float32MultiArray()
        dbg.data = [float(self.step_in_episode), *final.tolist(), *base.tolist(), *residual.tolist(),
                    base_speed, base_yaw_rate, gx, gy, goal_heading, *lidar.tolist(),
                    float(depth_obs.mean()), infer_ms]
        self.debug_pub.publish(dbg)
        self.prev_cmd = final
        self.step_in_episode += 1
        self._publish(final, stamp_ns)
        return True

    def _publish(self, cmd, stamp_ns, immediate=False):
        """Hand a command to the output. ~/cmd_vel_stamped always carries the command as given
        (the raw policy output); /cmd_vel is that command either directly (stamp mode, ramp
        off, or immediate=True for emergency / stale-sensor stops) or through the ramp."""
        ts = TwistStamped()
        ts.header.frame_id = self.base_frame
        ts.header.stamp = (Time(nanoseconds=stamp_ns).to_msg() if stamp_ns is not None
                           else self.get_clock().now().to_msg())
        ts.twist.linear.x, ts.twist.linear.y, ts.twist.angular.z = float(cmd[0]), float(cmd[1]), float(cmd[2])
        self.cmd_stamped_pub.publish(ts)
        self.target = np.asarray(cmd, np.float32).copy()
        if immediate or not self.ramp_enabled:
            self.applied = self.target.copy()
            self.ramping = False
            self._send(self.applied)
        else:
            self.ramping = True    # _ramp_step() publishes /cmd_vel

    def _send(self, cmd):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = float(cmd[0]), float(cmd[1]), float(cmd[2])
        self.cmd_pub.publish(twist)

    def _stop_now(self):
        self.target = np.zeros(3, np.float32)
        self.applied = np.zeros(3, np.float32)
        self.ramping = False
        self._send(self.applied)

    def _ramp_step(self):
        """Move the published command towards the policy's command by at most accel * dt per
        axis - the same limiter the training environment applied (per physics step there)."""
        if not self.ramping or self.estop:
            return
        delta = self.target - self.applied
        dv, dw = self.max_lin_acc * self.ramp_dt, self.max_ang_acc * self.ramp_dt
        delta[:2] = np.clip(delta[:2], -dv, dv)
        delta[2] = np.clip(delta[2], -dw, dw)
        self.applied = (self.applied + delta).astype(np.float32)
        self._send(self.applied)
        if self.silent_after_stop and not self.applied.any():
            self.ramping = False    # stopped at the goal: the zero command above was the last one
            self.silent_after_stop = False


def main(args=None):
    rclpy.init(args=args)
    node = OmniPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C can arrive again while cleaning up (launch forwards SIGINT too): never let
        # that turn a normal shutdown into a traceback.
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())    # leave the robot stopped
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, Exception):    # pylint: disable=broad-except
            pass


if __name__ == '__main__':
    main()
