#!/usr/bin/env python

import rospy
import rospkg
import math
import numpy as np
import torch
import random

from actionlib import SimpleActionClient
from actionlib import TerminalState
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import *
from control_msgs.msg import *
from sensor_msgs.msg import JointState
from table_tennis.srv import *
from dqn import ActionNN


class TableTennisAgent():
    def __init__(self):
        # Initialize robot parameters
        self.initRobot()

        # Initialize ball state info
        self.ballPosition = np.array((3, 1))
        self.ballVelocity = np.array((3, 1))

        # Initialize pytorch model
        rospy.loginfo("Initializing pytorch model...")
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('table_tennis')
        
        self.model = torch.load(pkgpath + r"/nn/action_net.pt")
        self.model.to(self.device)
        self.model.eval()

        # Initialize joint-trajectory action client
        self.client = SimpleActionClient(
            "/iiwa/EffortJointInterface_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)

        # Wait for joint-trajectory server
        rospy.loginfo("Waiting for server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Server connected.")

        # Subscibe to ball state
        self.ballSub = rospy.Subscriber("/table_tennis/ball/state",
                                        Float32MultiArray,
                                        self.ballStateCallback)

        # Subscribe to robot joint states
        self.jointSub = rospy.Subscriber("/iiwa/joint_states",
                                         JointState,
                                         self.jointStateCallback)

        # Create agent command service server
        self.agentCommandSrv = rospy.Service("/table_tennis/agent/command",
                                             CommandAgent,
                                             self.handleAgentCommand)

        # Create ik request service client
        self.ikPoseSolver = rospy.ServiceProxy("/table_tennis/ik/position",
                                               SolveIKPose)
        self.ikVelocitySolver = rospy.ServiceProxy("/table_tennis/ik/velocity",
                                                   SolveIKVelocity)

        # Create world state service client
        self.getWorldStateSrv = rospy.ServiceProxy("table_tennis/dqn/state",
                                                   GetWorldState)

        rospy.loginfo("Agent ready.")

    def initRobot(self):
        self.jointNames = [
            "iiwa_joint_1",
            "iiwa_joint_2",
            "iiwa_joint_3",
            "iiwa_joint_4",
            "iiwa_joint_5",
            "iiwa_joint_6",
            "iiwa_joint_7"
        ]

        self.maxJointVel = [
            1.71,
            1.71,
            1.74,
            2.27,
            2.44,
            3.14,
            3.14
        ]

        self.robotOrigin = np.array([[-1.5], [0], [0.5]])
        self.robotReach = 0.7
        self.robotState = 0
        self.jointState = JointState()

    def handleAgentCommand(self, req):
        rospy.loginfo("Received " + req.command + " command.")
        if (req.command == "home"):
            rospy.loginfo("Home command received.")
            self.home()
            self.robotState = 0
            return CommandAgentResponse(True)

        elif (req.command == "receive"):
            rospy.loginfo("Receive command received.")
            self.robotState = 1

            # Get world state
            state = self.getWorldStateSrv("").state

            if (len(state) < 7):
                rospy.logerr("World state not initialized.")
                return CommandAgentResponse(False)

            xd = state[3]
            yd = state[4]
            zd = state[5]
            rospy.loginfo("World state: " + str(state))

            # Calculate ball receive state
            rospy.loginfo("Calculating receive state...")
            (x, y, z, vx, vy, vz, t) = self.calcBallReceiveState(state)
            z = z + 0.05 # adjustment
            #y = y + 0.05 # adjustment
            rospy.loginfo("Ball receive position: (%f, %f, %f)" % (x, y, z))
            rospy.loginfo("Ball receive velocity: (%f, %f, %f)" % (vx, vy, vz))
            rospy.loginfo("Ball receive time: %f" % t)

            # Choose paddle orientation and velocity
            rospy.loginfo("Requesting action from dqn..")
            input = torch.tensor([x, y, z, xd, yd, zd, vx, vy, vz],
                                 dtype=torch.float,
                                 device=self.device)
            output = self.model(input)
            pvel = output[:3].cpu().data.numpy()
            angles = output[3:].cpu().data.numpy()
            rospy.loginfo("Action: (" + str(pvel) + "), (" + str(angles) + ")")
            pvel = [1,0,0.2]

            # Calculate required positions and velocities
            d = 0.05
            #rot = self.anglesToRot(angles)

            rot = np.matrix([[0,1,0], [0,0,-1], [-1,0,0]])


            velmat = np.matrix(np.resize(pvel, (3,1)))
            p0 = np.matrix([[x], [y], [z]]) - rot * \
                velmat/np.linalg.norm(velmat)*d - self.robotOrigin
            p1 = np.matrix([[x], [y], [z]]) - self.robotOrigin
            p2 = np.matrix([[x], [y], [z]]) + rot * \
                velmat/np.linalg.norm(velmat)*d - self.robotOrigin
            
            rospy.loginfo("Return Position relative to robot: " + str(p1))

            # Check if robot can reach without colliding with table
            if (z <= 1):
               rospy.logerr("Can't reach ball, quitting.")
               return CommandAgentResponse(False) 

            # Calculate trajectory
            rospy.loginfo("Calculating trajectory...")
            rospy.loginfo("Performing IK on: " + str(p0) + ", " + str(rot))
            point1 = self.getTargetPoint(p0, rot)
            rospy.loginfo("Performing IK on: " + str(p1) + ", " + str(rot) + ", " + str(pvel))
            point2 = self.getTargetPoint(p1, rot, velmat)
            rospy.loginfo("Performing IK on: " + str(p2) + ", " + str(rot))
            point3 = self.getTargetPoint(p2, rot)
            if (point1 is None or point2 is None or point3 is None):
                rospy.loginfo("Failed to calculate trajectory.")
                return CommandAgentResponse(False)

            rospy.loginfo("Points are valid.")
            prepTime = self.estimateTime(self.getCurrentPoint(), point1)
            rospy.loginfo("Prep time: " + str(prepTime))

            if (prepTime > t):
                prepTime = t - 0.25

            point1.time_from_start = rospy.Duration(t - 0.25)
            point2.time_from_start = rospy.Duration(t - 0.1)
            point3.time_from_start = rospy.Duration(t + 0.5)

            # Perform trajectory
            rospy.loginfo("Performing trajectory...")
            success = self.runTrajectory([point1, point2, point3])

            if (success):
                rospy.loginfo("Trajectory complete.")
            else:
                rospy.loginfo("Trajectory failed.")

            self.robotState = 0

            return CommandAgentResponse(True)

        elif (req.command == "ready"):
            rospy.loginfo("Moving to ready position.")
            success = self.ready()

            if (success):
                rospy.loginfo("Trajectory successful.")
            else:
                rospy.loginfo("Trajectory failed.")
                return CommandAgentResponse(False)   

            return CommandAgentResponse(True)   


        elif (req.command == "demo"):
            rospy.loginfo("Starting demo trajectory...")
            p0 = np.matrix([[0.3], [-0.3], [0.9]])
            p1 = np.matrix([[0.325], [-0.3], [0.9]])
            p2 = np.matrix([[0.35], [-0.3], [0.9]])

            rot = np.matrix([[0,1,0], [0,0,-1], [-1,0,0]])
            pvel = [1, 0, 0]

            point1 = self.getTargetPoint(p0, rot)
            point2 = self.getTargetPoint(p1, rot, pvel)
            point3 = self.getTargetPoint(p2, rot)

            if (point1 is None or point2 is None or point3 is None):
                rospy.loginfo("Failed to calculate trajectory.")
                return CommandAgentResponse(False)

            point1.time_from_start = rospy.Duration(0.3)
            point2.time_from_start = rospy.Duration(0.5)
            point3.time_from_start = rospy.Duration(0.75)

            success = self.runTrajectory([point1, point2, point3])

            if (success):
                rospy.loginfo("Trajectory successful.")
            else:
                rospy.loginfo("Trajectory failed.")
                return CommandAgentResponse(False)   

            return CommandAgentResponse(True)            

        rospy.loginfo("Invalid command.")

        return CommandAgentResponse(False)

    def ballStateCallback(self, msg):
        data = msg.data
        self.ballPosition = np.array(data[:3])
        self.ballVelocity = np.array(data[3:])

    def jointStateCallback(self, msg):
        self.jointState = msg

    def runTrajectory(self, points):
        self.client.cancel_all_goals()

        trajectory = JointTrajectory()
        trajectory.joint_names = self.jointNames
        trajectory.points = points

        goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        goal.goal_time_tolerance = rospy.Duration(0.02)

        result = self.client.send_goal_and_wait(goal)
        return result

    def home(self):
        home = self.getHomePoint()
        current = self.getCurrentPoint()

        time = self.estimateTime(home, current, 1.5)
        home.time_from_start = rospy.Duration(time)

        return self.runTrajectory([home])

    def ready(self):
        rospy.loginfo("Moving to ready position.")
        p = np.matrix([[0.3], [-0.3], [0.9]])
        rot = np.matrix([[0,1,0], [0,0,-1], [-1,0,0]])

        current = self.getCurrentPoint()
        point = self.getTargetPoint(p, rot)

        if (point is None):
            rospy.loginfo("Failed to calculate ready position.")
            return False

        time = self.estimateTime(current, point, 1.25)
        point.time_from_start = rospy.Duration(time)

        return self.runTrajectory([point])

    def getHomePoint(self):
        home = JointTrajectoryPoint()
        home.positions = [0, 0, 0, 0, 0, 0, 0]
        return home

    def getCurrentPoint(self):
        current = JointTrajectoryPoint()
        current.positions = self.jointState.position
        return current

    def estimateTime(self, a, b, scale=1):
        maxTime = 0
        for i in range(7):
            time = (b.positions[i] - a.positions[i]) / self.maxJointVel[i]
            if (time > maxTime):
                maxTime = time
        return maxTime * scale

    def calcBallReceiveState(self, state):
        [x0, y0, z0] = state[:3]
        [vx0, vy0, vz0] = state[6:]
        [xr, yr, zr] = self.robotOrigin

        table = 0.76
        Kr = 0.82
        dt = 0.01
        g = -9.81

        bounce = False
        t = 0
        tb = 0
        vz = vz0
        vzb = vz0

        while (True):
            x = x0 + vx0*t
            y = y0 + vy0*t

            if (bounce):
                z = table + vzb*tb + 0.5*g*tb**2
                tb += dt
                vz = vzb + g*t
            else:
                vz = vz0 + g*t
                z = z0 + vz0*t + 0.5*g*t**2

            r = math.sqrt((x - xr)**2 + (y - yr)**2 + (z - zr)**2)

            t += dt

            if (z <= table and not bounce):
                vz = -1*vz*Kr
                vzb = vz
                tb = 0
                bounce = True
            #elif (r <= self.robotReach):
            #    break
            elif (x <= self.robotOrigin[0] + 0.325):
                break
            elif (z <= table - 0.1 and bounce):
                break

            

        return (x, y, z, vx0, vy0, vz, t)

    def anglesToRot(self, angles):
        phi = angles[0]
        theta = angles[1]
        psi = angles[2]

        Rx = np.matrix([[1, 0, 0],
                        [0, math.cos(phi), -math.sin(phi)],
                        [0, math.sin(phi), math.cos(phi)]])
        Ry = np.matrix([[math.cos(theta), 0, math.sin(theta)],
                        [0, 1, 0],
                        [-math.sin(theta), 0, math.cos(theta)]])
        Rz = np.matrix([[math.cos(psi), -math.sin(psi), 0],
                        [math.sin(psi), math.cos(psi), 0],
                        [0, 0, 1]])
        return Rz*Ry*Rx

    def IKPose(self, pos, rot):
        req = SolveIKPoseRequest()
        req.x = pos[0]
        req.y = pos[1]
        req.z = pos[2]
        req.rotx = np.array(rot[:3, 0])
        req.roty = np.array(rot[:3, 1])
        req.rotz = np.array(rot[:3, 2])
        req.initialState = self.jointState.position

        res = self.ikPoseSolver(req)

        return res

    def IKVelocity(self, linear, angular):
        req = SolveIKVelocityRequest()
        req.linear = linear
        req.angular = angular
        req.initialState = self.jointState.position

        res = self.ikVelocitySolver(req)

        return res

    def getTargetPoint(self, pos, rot=None, linvel=None, angvel=None, time=None):
        # Inputs
        if rot is None:
            rot = np.identity(4)
        if linvel is None:
            linvel = [0, 0, 0]
        if angvel is None:
            angvel = [0, 0, 0]
        if time is None:
            time = 0

        # Solve IK for joints
        pose = self.IKPose(pos, rot)
        pose_err = pose.error
        position = pose.solution

        vel = self.IKVelocity(linvel, angvel)
        vel_err = vel.error
        velocity = vel.solution

        if (pose_err*vel_err < 0):
            return None

        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = velocity
        point.time_from_start = rospy.Duration(time)

        return point


if __name__ == '__main__':

    # Initialize ros node
    rospy.loginfo("Initializing table tennis agent...")
    rospy.init_node("table_tennis_agent")

    # Start TableTennis Agent
    agent = TableTennisAgent()
    rospy.spin()
