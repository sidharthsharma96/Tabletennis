#include <stdio.h>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    std::cout << "Initializing Gazebo client...\n";

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    std::cout << "Initializing communication node...\n";

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    std::cout << "Advertising ball_state_cmd topic...\n";

    // Publish to the velodyne topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<gazebo::msgs::Pose>("~/ping_pong_ball/ball_state_cmd");

    std::cout << "Waiting for connection...\n";

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    std::cout << "Connection succeeded.\n";

    // Create a a pose message
    gazebo::msgs::Pose msg;

    std::cout << "Parsing inputs...\n";

    // Parse args
    double x = atof(_argv[1]);
    double y = atof(_argv[2]);
    double z = atof(_argv[3]);
    double vx = atof(_argv[4]);
    double vy = atof(_argv[5]);
    double vz = atof(_argv[6]);

    std::cout << "Generating pose...\n";

    // Create pose
    ignition::math::Pose3d pose (x, y, z, 0, vx, vy, vz);

    std::string pose_str = "Position: " + 
        std::to_string(pose.Pos().X()) + ", " + 
        std::to_string(pose.Pos().Y()) + ", " + 
        std::to_string(pose.Pos().Z()) + ", Velocity: " +
        std::to_string(pose.Rot().X()) + ", " + 
        std::to_string(pose.Rot().Y()) + ", " + 
        std::to_string(pose.Rot().Z());
    std::cout << pose_str << '\n';

    std::cout << "Generating message...\n";

    // Set the velocity in the x-component
    gazebo::msgs::Set(&msg, pose);

    std::cout << "Sending message...\n";

    try
    {
        // Send the message
        pub->Publish(msg);
        std::cout << "Message sent.\n";
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    catch(const gazebo::common::Exception& e)
    {
        std::cerr << e.GetErrorStr() << '\n';
    }

    std::cout << "Shutting down...\n";

    // Make sure to shut everything down.
    gazebo::client::shutdown();

    std::cout << "Shut down.\n";
}