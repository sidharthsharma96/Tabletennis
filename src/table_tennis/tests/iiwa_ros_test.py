#!/usr/bin/env python

import rospy
import math
from actionlib import SimpleActionClient
from actionlib import TerminalState
from trajectory_msgs.msg import *
from control_msgs.msg import *
from table_tennis.srv import *


def main():
    rospy.loginfo("Initializing...")
    rospy.init_node("iiwa_commander", argv=sys.argv)

    rospy.loginfo("Connecting to IK services...")
    posIKClient = rospy.ServiceProxy("/table_tennis/ik/position", SolveIKPose)
    velIKClient = rospy.ServiceProxy("/table_tennis/ik/velocity", SolveIKVelocity)

    rospy.loginfo("Connecting to trajectory controller...")
    trajClient = SimpleActionClient(
        "/iiwa/EffortJointInterface_trajectory_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction)
    trajClient.wait_for_server()

    rospy.loginfo("Connection succeeded.")

    x = 2.069104
    y = 0.037088
    z = -0.124427
    vx = 0
    vy = 0
    vz = 0

    # parse params
    if rospy.has_param("~x"): x = rospy.get_param("~x")
    if rospy.has_param("~y"): y = rospy.get_param("~y")
    if rospy.has_param("~z"): z = rospy.get_param("~z")
    if rospy.has_param("~vx"): vx = rospy.get_param("~vx")
    if rospy.has_param("~vy"): vy = rospy.get_param("~vy")
    if rospy.has_param("~vz"): vz = rospy.get_param("~vz")


    rospy.loginfo("Sending IK pose service request")

    req = SolveIKPoseRequest()
    req.initialState = [0, 0, 0, 0, 0, 0, 0]
    req.x = x
    req.y = y
    req.z = z
    req.rotx = [0.995641, -0.093079, -0.005926]
    req.roty = [0.091050, 0.983780, -0.154551]
    req.rotz = [0.020215, 0.153338, 0.987967]

    res = posIKClient.call(req)
    errorcode = res.error
    position = res.solution

    rospy.loginfo("IK pose service result: " + str(errorcode))
    rospy.loginfo(position)


    rospy.loginfo("Sending IK velocity service request")
    req = SolveIKVelocityRequest()
    req.initialState = [0, 0, 0, 0, 0, 0, 0]
    req.angular = [0, 0, 0]
    req.linear = [vx, vy, vz]

    res = velIKClient.call(req)
    errorcode = res.error
    velocity = res.solution

    rospy.loginfo("IK velocity service result: " + str(errorcode))
    rospy.loginfo(velocity)

    
    point1 = JointTrajectoryPoint()
    point1.positions = position
    point1.velocities = velocity
    point1.time_from_start = rospy.Duration(1)

    trajectory = JointTrajectory()
    trajectory.joint_names = \
        [
            "iiwa_joint_1",
            "iiwa_joint_2",
            "iiwa_joint_3",
            "iiwa_joint_4",
            "iiwa_joint_5",
            "iiwa_joint_6",
            "iiwa_joint_7"
        ]
    trajectory.points = [point1]

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    trajClient.send_goal(goal)
    trajClient.wait_for_result()

    result = trajClient.get_result()

    if result >=0:
        rospy.loginfo("Action success!")
    else:
        rospy.loginfo("Action failed")


if __name__ == '__main__':
    main()
