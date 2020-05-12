#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <thread>
#include <signal.h>

namespace tabletennis
{
class TableTennisAgent
{
public:
    TableTennisAgent(ros::NodeHandle *node)
    {
        // Initiate robot parametes
        this->robotOrigin[0] = -1.5;
        this->robotOrigin[1] = 0;
        this->robotOrigin[2] = 0.84;

        // Get node handle
        this->node = node;

        // Initialize joint-trajectory action client
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client(
            "/iiwa/EffortJointInterface_trajectory_controller/follow_joint_trajectory",
            true);
        this->trajectoryClient = &client;

        // Wait for joint-trajectory server
        ROS_INFO("Waiting for server to start.");
        trajectoryClient->waitForServer();

        // Subscribe to ball state
        ros::Subscriber ballStateSubscriber = node->subscribe(
            "/table_tennis/ball/state", 30, &TableTennisAgent::ballStateCallback, this);

        // Subscribe to robot commands from world
        ros::Subscriber commandSubscriber = node->subscribe(
            "/table_tennis/agent/command", 10, &TableTennisAgent::agentCommandCallback, this);

        // Subscribe to robot joint states
        ros::Subscriber jointStateSubscriber = node->subscribe(
            "/iiwa/joint_states", 30, &TableTennisAgent::jointStateCallback, this);

        // Publish robot state
        //this->statePublisher = node->advertise("/table_tennis/agent/state", 10);

        // Spin up main robot thread
        this->mainThread = std::thread(
            std::bind(&TableTennisAgent::mainThread, this)
        );
    }

    void Stop()
    {
        this->running = false;
    }

private:
#pragma region Callbacks
    void agentCommandCallback(std_msgs::StringConstPtr &msg)
    {
        std::string command = msg->data;

        if (command == "home")
        {
            home();
        }
        else if (command == "receive")
        {
            this->robotState = 1;
        }
    }

    void ballStateCallback(std_msgs::Float32MultiArrayConstPtr &msg)
    {
        std::vector<float> state = msg->data;

        ballPosition[0] = state[0];
        ballPosition[1] = state[1];
        ballPosition[2] = state[2];

        ballVelocity[0] = state[3];
        ballVelocity[1] = state[4];
        ballVelocity[2] = state[5];

        if (this->robotState == 1) // Commanded to receive
        {
            // Calculate ball recieve position
            std::vector<double> receive = this->calcBallReceivePos();
            double x = receive[0];
            double y = receive[1];
            double z = receive[2];
            double t = receive[3];

            // Choose paddle orientation and velocity

            // Move robot to receive position

            // Start trajectory to receive

        }
    }

    void jointStateCallback(sensor_msgs::JointStateConstPtr &msg)
    {
        this->jointState = *msg;
    }

    void trajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                const control_msgs::FollowJointTrajectoryResultConstPtr &result)
    {
    }

    void trajectoryActiveCallback()
    {
    }

    void trajectoryFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback)
    {
    }
#pragma endregion

#pragma region Table Tennis
    void mainThread()
    {
        while (running)
        {
            
        }
        
    }

    std::vector<double> calcBallReceivePos()
    {
        double x0 = this->ballPosition[0];
        double y0 = this->ballPosition[1];
        double z0 = this->ballPosition[2];
        double vx0 = this->ballVelocity[0];
        double vy0 = this->ballVelocity[1];
        double vz0 = this->ballVelocity[2];
        double xr = this->robotOrigin[0];
        double yr = this->robotOrigin[1];
        double zr = this->robotOrigin[2];

        const double table = 0.76;
        const double Kr = 0.82;
        const double dt = 0.01;
        const double g = -9.81;

        bool bounce = false;
        double t = 0;
        double tb = 0;
        double vz = vz0;

        double x = 0;
        double y = 0;
        double z = 0;

        while (true)
        {
            x = x0 + vx0*t;
            y = y0 + vy0*t;
            
            if (bounce)
            {
                z = table + vz*tb + 0.5*g*pow(t, 2);
                tb += dt;
            }
            else
            {
                vz = vz0 + g*t;
                z = z0 + vz0*t + 0.5*g*pow(t, 2);
            }
            
            double r = sqrt(pow(x - xr, 2) + pow(y - yr, 2) + pow(z - zr, 2));

            if (z <= table and not bounce)
            {
                vz = -1*vz * Kr;
                tb = 0;
                bounce = true;
            }
            else if (r <= this->robotReach)
            {
                break;
            }
            else if (z <= table and bounce)
            {
                break;
            }

            t += dt;
        }

        return std::vector { x, y, z, t };
    }
#pragma endregion

#pragma region Trajectory Control
    bool runTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> points)
    {
        trajectoryClient->cancelAllGoals();

        trajectory_msgs::JointTrajectory trajectory;
        trajectory.joint_names = this->jointNames;
        trajectory.points = points;

        control_msgs::FollowJointTrajectoryActionGoal goal;
        goal.goal.trajectory = trajectory;

        trajectoryClient->sendGoal(goal.goal,
                                   &trajectoryDoneCallback,
                                   &trajectoryActiveCallback,
                                   &trajectoryFeedbackCallback);

        bool success = trajectoryClient->waitForResult(ros::Duration(5));

        return success;
    }

    bool home()
    {
        trajectory_msgs::JointTrajectoryPoint home = this->getHomePoint();
        trajectory_msgs::JointTrajectoryPoint current = this->getCurrentPoint();

        double time = this->estimateTime(current, home);
        home.time_from_start = ros::Duration(time);

        std::vector<trajectory_msgs::JointTrajectoryPoint> points{home};

        return this->runTrajectory(points);
    }

    trajectory_msgs::JointTrajectoryPoint getHomePoint()
    {
        std::vector<double> positions{0, 0, 0, 0, 0, 0, 0};
        std::vector<double> velocities{0, 0, 0, 0, 0, 0, 0};

        trajectory_msgs::JointTrajectoryPoint home;
        home.positions = positions;
        home.velocities = velocities;

        return home;
    }

    trajectory_msgs::JointTrajectoryPoint getCurrentPoint()
    {
        trajectory_msgs::JointTrajectoryPoint current;
        current.positions = jointState.position;

        return current;
    }

    double estimateTime(trajectory_msgs::JointTrajectoryPoint a,
                        trajectory_msgs::JointTrajectoryPoint b)
    {
        double maxTime = 0;
        for (int i = 0; i < 7; ++i)
        {
            double time = (b.positions[i] - a.positions[i]) / this->maxJointVelocities[i];
            if (time > maxTime)
                maxTime = time;
        }

        return maxTime;
    }
#pragma endregion

#pragma region Communication
    ros::NodeHandle *node;

    ros::Publisher statePublisher;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectoryClient;
#pragma endregion

#pragma region Ball
    // Ball state info
    double ballPosition[3];
    double ballVelocity[3];
#pragma endregion

#pragma region Robot
    // Robot joint state
    sensor_msgs::JointState jointState;

    int robotState = 0;

    // Robot main control thread
    std::thread mainThread;
    bool running = true;

    // Robot parameters
    const std::vector<std::string> jointNames{
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"};

    const std::vector<float> maxJointVelocities{
        1.71,
        1.71,
        1.74,
        2.27,
        2.44,
        3.14,
        3.14};
    
    double robotOrigin[3];

    const double robotReach = 0.8;
};
#pragma endregion

} // namespace tabletennis

int main(int argc, char **argv)
{
    // Initialize ros
    ROS_INFO("Initializing Table Tennis Agent...");
    ros::init(argc, argv, "table_tennis_agent");

    // Get node handle
    ros::NodeHandle node;

    // Start agent
    tabletennis::TableTennisAgent agent(&node);

    // Spin
    ros::spin();


}