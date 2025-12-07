from geometry_msgs.msg import PoseStamped
from zm_robot_programing.zm_robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class zm_robot_cmd():
    def __init__(self):
        self.navigator = BasicNavigator()
        self.initialPose = PoseStamped()
        self.goal_pose = PoseStamped()

    def waitMove(self):
        i = 0
        while not self.navigator.isTaskComplete():
            # Some navigation timeout to demo cancellation
            feedback = self.navigator.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def initial_pose(self, x, y, theta):
        self.initialPose.header.frame_id = 'map'
        self.initialPose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initialPose.pose.position.x = x
        self.initialPose.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        self.initialPose.pose.orientation.x = q[0]
        self.initialPose.pose.orientation.y = q[1]
        self.initialPose.pose.orientation.z = q[2]
        self.initialPose.pose.orientation.w = q[3]
        self.navigator.setInitialPose(self.initialPose)


    def move_map(self, x , y, theta):
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        self.goal_pose.pose.orientation.x = q[0]
        self.goal_pose.pose.orientation.y = q[1]
        self.goal_pose.pose.orientation.z = q[2]
        self.goal_pose.pose.orientation.w = q[3]
        self.navigator.goToPose(self.goal_pose)
        self.waitMove()

    def move_base(self, x , y, theta):
        self.goal_pose.header.frame_id = 'base_link'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        self.goal_pose.pose.orientation.x = q[0]
        self.goal_pose.pose.orientation.y = q[1]
        self.goal_pose.pose.orientation.z = q[2]
        self.goal_pose.pose.orientation.w = q[3]
        self.navigator.goToPose(self.goal_pose)
        self.waitMove()
