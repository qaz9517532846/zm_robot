#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math


def doneCb(state, result):
    rospy.loginfo(state)
    rospy.loginfo(result)
    rospy.loginfo(client.get_goal_status_text())


def activeCb():
    rospy.loginfo("Goal just went active")


def feedbackCb(feedback):
    pass
    #rospy.loginfo(feedback)


def goalCB(msg):
    rospy.loginfo(msg)
    goal = MoveBaseGoal()
    goal.target_pose = msg
    rospy.loginfo(goal)
    client.send_goal(goal, doneCb, activeCb, feedbackCb)


if __name__ == '__main__':
    rospy.init_node('client')
    rospy.loginfo(rospy.get_name() + " start")
    action_name = "zm_robot_move_relative"
    client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
    rospy.loginfo("wait_for_server: " + action_name)
    client.wait_for_server()
    rospy.loginfo("get server: " + action_name)
    rviz_goal_Subscriber = rospy.Subscriber("/move_base_simple/goal",
                                            PoseStamped, goalCB)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
       rospy.loginfo("input goal: ")
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id = raw_input("frame_id:") or "base_link"
       goal.target_pose.pose.position.x = float(raw_input("x (meter): "))
       goal.target_pose.pose.position.y = float(raw_input("y (meter): "))
       goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.radians(float(raw_input("theta (degree): ")))))
       rospy.loginfo(goal)
       client.send_goal(goal, doneCb, activeCb, feedbackCb)
