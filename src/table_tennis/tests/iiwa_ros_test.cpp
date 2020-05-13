#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "kdl/chainiksolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"

void DoneCallback(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
    ROS_INFO("Action completed");
    ROS_INFO("State: %s", state.toString().c_str());
    ROS_INFO("Result: %s", result.get()->error_string.c_str());
}


void ActiveCallback()
{
    ROS_INFO("Goal went active");
}

void FeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
    double desired_2 = feedback->desired.velocities[1];
    double actual_2 = feedback->actual.velocities[1];
    double error_2 = feedback->error.velocities[1];
    double desired_4 = feedback->desired.velocities[3];
    double actual_4 = feedback->actual.velocities[3];
    double error_4 = feedback->error.velocities[3];
    int32_t time = feedback->actual.time_from_start.nsec;

    ROS_INFO("Time: %d, Desired: (%f, %f), Actual: (%f, %f), Error: (%f, %f)", 
        time, desired_2, desired_4, actual_2, actual_4, error_2, error_4);
}

int main(int argc, char **argv)
{
    ROS_INFO("Initializing...");

    ros::init(argc, argv, "iiwa_commander");
    
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client(
        "/iiwa/EffortJointInterface_trajectory_controller/follow_joint_trajectory", 
        true
    );

    ROS_INFO("Waiting for server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    std::vector<std::string> joint_names {
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"
    };

    std::vector<double> point1_positions {
        0,
        3.14 / 4,
        0,
        3.14 / 4,
        0,
        0,
        0
    };

    std::vector<double> point1_velocities {
        0,
        2,
        0,
        2,
        0,
        0,
        0
    };

    std::vector<double> point2_positions {
        0,
        3.14 / 2,
        0,
        3.14 / 2,
        0,
        0,
        0
    };

    std::vector<double> point2_velocities {
        0,
        0,
        0,
        0,
        0,
        0,
        0
    };
    
    trajectory_msgs::JointTrajectoryPoint point1;
    point1.positions = point1_positions;
    point1.velocities = point1_velocities;
    point1.time_from_start = ros::Duration(0.5);

    trajectory_msgs::JointTrajectoryPoint point2;
    point2.positions = point2_positions;
    point2.velocities = point2_velocities;
    point2.time_from_start = ros::Duration(1);

    control_msgs::JointTolerance tol;
    tol.velocity = 0.01;

    std::vector<control_msgs::JointTolerance> tolerances {
        tol,
        tol,
        tol,
        tol,
        tol,
        tol,
        tol
    };

    std::vector<trajectory_msgs::JointTrajectoryPoint> points {
        point1,
        point2
    };

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points = points;
    
    control_msgs::FollowJointTrajectoryActionGoal goal;
    goal.goal.trajectory = trajectory;
    //goal.goal.goal_tolerance = tolerances;
    
    client.sendGoal(goal.goal, &DoneCallback, &ActiveCallback, &FeedbackCallback);

    bool success = client.waitForResult(ros::Duration(5));

    if (success)
    {
        boost::shared_ptr<const control_msgs::FollowJointTrajectoryResult> result = client.getResult();
        actionlib::SimpleClientGoalState state = client.getState();

        ROS_INFO("Action success!");
    }
    else
    {
        ROS_INFO("Action failed");
    }

    Eigen::Matrix<double, 7, 1> mat;

    std::vector<double> state { 0, 0, 0, 0, 0, 0, 0 };

    const KDL::JntArray jntarr(7);
    KDL::JntArray jntposin (7);
    KDL::JntArray jntposout (7);

    //jntarr.data = Eigen::Map<const Eigen::VectorXd>(state.data(), state.size());
    //jntposin = jntarr;

    KDL::Vector vel = KDL::Vector(0.2, 0.2, 0.2);
    KDL::Vector rv = KDL::Vector(0, 0, 0);
    KDL::Rotation rot = KDL::Rotation(0, 0, 0, 0, 0, 0, 0, 0, 0);

    KDL::RotationVel rotvel = KDL::RotationVel(rot);
    KDL::VectorVel vecvel = KDL::VectorVel(vel);
    KDL::FrameVel framevel = KDL::FrameVel(rotvel, vecvel);
    const KDL::Twist twist = KDL::Twist(vel ,rv);

    
    const KDL::Chain chain = KDL::Chain();
    
    //KDL::ChainIkSolverVel_pinv slv = KDL::ChainIkSolverVel_pinv();

    //int succ = KDL::ChainIkSolverVel::CartToJnt(jntposin, twist, jntposout);

}