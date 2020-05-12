#!/usr/bin/env python

import sys
import rospy
import numpy as np
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander, roscpp_initialize
from math import pi

rospy.init_node('moveit_tests', anonymous=False)
roscpp_initialize(sys.argv)

group = MoveGroupCommander('manipulator','/iiwa/robot_description','/iiwa')
print group.get_current_pose()
print group.get_current_joint_values()

joint_goal = np.zeros(7)
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3
joint_goal[6] = 0

print group.go(joint_goal)

print group.get_current_pose()
print group.get_current_joint_values()

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.8
group.set_pose_target(pose_goal)

group.set_pose_target(pose_goal)
print group.go(wait=True)
group.stop()
group.clear_pose_targets()

print group.get_current_pose()
print group.get_current_joint_values()

