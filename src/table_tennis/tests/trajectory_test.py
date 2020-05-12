#!/usr/bin/env python

import rospy
import math
import time
from actionlib import SimpleActionClient
from actionlib import TerminalState
from trajectory_msgs.msg import *
from control_msgs.msg import *
from table_tennis.srv import *
from sensor_msgs.msg import JointState

stateReceived = False
jointState = JointState()

def jointStateCallback(msg):
    global jointState
    global stateReceived
    jointState = msg
    stateReceived = True


def main():
    rospy.loginfo("Initializing...")
    rospy.init_node("trajectory_test", argv=sys.argv)

    rospy.loginfo("Connecting to IK services...")
    posFKClient = rospy.ServiceProxy("/table_tennis/fk/pose", SolveFKPose)
    posIKClient = rospy.ServiceProxy("/table_tennis/ik/position", SolveIKPose)
    velIKClient = rospy.ServiceProxy(
        "/table_tennis/ik/velocity", SolveIKVelocity)

    rospy.loginfo("Connecting to trajectory controller...")
    trajClient = SimpleActionClient(
        "/iiwa/EffortJointInterface_trajectory_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction)
    trajClient.wait_for_server()

    rospy.loginfo("Connection succeeded.")

    # Subscribe to robot joint states
    jointSub = rospy.Subscriber("/iiwa/joint_states",
                                JointState,
                                jointStateCallback)

    dx = 0.05
    dy = 0.0
    dz = -0.05

    # parse params
    if rospy.has_param("~dx"):
        dx = rospy.get_param("~dx")
    if rospy.has_param("~dy"):
        dy = rospy.get_param("~dy")
    if rospy.has_param("~dz"):
        dz = rospy.get_param("~dz")

    while (not stateReceived):
        time.sleep(0.01)

    # Get current cartesian positions
    rospy.loginfo("Get current task position.")
    req = SolveFKPoseRequest()
    req.jointPositions = jointState.position
    res = posFKClient.call(req)
    errorcode = res.error
    position = res.position
    rotx = res.rotx
    roty = res.roty
    rotz = res.rotz
    rospy.loginfo("FK pose service result: " + str(errorcode))


    if (errorcode < 0):
        rospy.logerr("FK pose service failed.")
        return

    rospy.loginfo("Sending IK pose service request")

    req = SolveIKPoseRequest()
    req.initialState = position
    req.x = position[0] + dx
    req.y = position[1] + dy
    req.z = position[2] + dz
    req.rotx = rotx
    req.roty = roty
    req.rotz = rotz

    res = posIKClient.call(req)
    errorcode = res.error
    position = res.solution

    rospy.loginfo("IK pose service result: " + str(errorcode))
    rospy.loginfo(position)

    if (errorcode < 0):
        rospy.logerr("IK pose service failed.")
        return

    # rospy.loginfo("Sending IK velocity service request")
    # req = SolveIKVelocityRequest()
    # req.initialState = [0, 0, 0, 0, 0, 0, 0]
    # req.angular = [0, 0, 0]
    # req.linear = [vx, vy, vz]

    # res = velIKClient.call(req)
    # errorcode = res.error
    # velocity = res.solution

    # rospy.loginfo("IK velocity service result: " + str(errorcode))
    # rospy.loginfo(velocity)

    point1 = JointTrajectoryPoint()
    point1.positions = position
    # point1.velocities = velocity
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

    if result >= 0:
        rospy.loginfo("Action success!")
    else:
        rospy.loginfo("Action failed")


if __name__ == '__main__':
    main()
