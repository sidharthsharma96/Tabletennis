#include <ros/ros.h>
#include <table_tennis/SolveIKPose.h>
#include <table_tennis/SolveIKVelocity.h>


int main(int argc, char **argv)
{
    ROS_INFO("Starting test...");
    ros::init(argc, argv, "kdl_test");

    ROS_INFO("Initializing clients...");
    ros::NodeHandle node;
    ros::ServiceClient posClient = node.serviceClient<table_tennis::SolveIKPose>("/table_tennis/ik/position");
    ros::ServiceClient velClient = node.serviceClient<table_tennis::SolveIKVelocity>("/table_tennis/ik/velocity");

    ROS_INFO("Attempting inverse pose kinematics...");
    table_tennis::SolveIKPose psrv;
    table_tennis::SolveIKPoseRequest preq;
    preq.initialState = std::vector<double> { 0, 0, 0, 0, 0, 0, 0 };
    preq.x = 0.4;
    preq.y = 0;
    preq.z = -1;
    preq.rotx = std::vector<double> { 1, 0, 0 };
    preq.roty = std::vector<double> { 0, 1, 0 };
    preq.rotz = std::vector<double> { 0, 0, 1 };
    psrv.request = preq;
    
    bool psuccess = posClient.call(psrv);
    std::vector<double> psol = psrv.response.solution;
    int perror = psrv.response.error;

    if (psuccess)
    {
        ROS_INFO("Result: %d", perror);
        for (int i = 0; i < psol.size(); ++i)
        {
            ROS_INFO("%f", psol[i]);
        }
    }
    else
    {
        ROS_INFO("Failed to send request.");
    }
    
    ROS_INFO("Attempting inverse velocity kinematics...");
    table_tennis::SolveIKVelocity vsrv;
    table_tennis::SolveIKVelocityRequest vreq;
    vreq.initialState = std::vector<double> { 0, 0, 0, 0, 0, 0, 0 };
    vreq.linear = std::vector<double> { 0.2, 0.2, 0.2 };
    vreq.angular = std::vector<double> { 0, 0, 0 };
    vsrv.request = vreq;
    
    bool vsuccess = velClient.call(vsrv);
    std::vector<double> vsol = vsrv.response.solution;
    int verror = psrv.response.error;

    if (vsuccess)
    {
        ROS_INFO("Result: %d", verror);
        for (int i = 0; i < vsol.size(); ++i)
        {
            ROS_INFO("%f", vsol[i]);
        }
    }
    else
    {
        ROS_INFO("Failed to send request.");
    }
    
    node.shutdown();
}