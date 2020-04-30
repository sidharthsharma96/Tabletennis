#include <functional>
#include <thread>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
class TrainingReturnPlugin : public WorldPlugin
{
public:
    TrainingReturnPlugin() : WorldPlugin()
    {
    }

    ~TrainingReturnPlugin()
    {
        this->rosNode->shutdown();
        this->rosSubQueueThread.join();
        this->rosPubQueueThread.join();
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        std::cout << "Loading world plugin...\n";
        this->world = _world;

        // Initialize communication node
        std::cout << "Initializing gazebo nodes...\n";
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(_world->GetName());

        // Create and subscribe to ball state topic
        std::string ballStateTopicName = "~/ping_pong_ball/ball_state";
        this->ballStateSubscriber = this->node->Subscribe(ballStateTopicName,
            &TrainingReturnPlugin::OnBallStateRetrieved, this);
        
        // Create and subscribe to net contact sensor topic
        std::string netContactSensorTopicName = 
            "~/net/ping_pong_table_net/body/net_contact";
        this->netContactSubscriber = this->node->Subscribe(netContactSensorTopicName,
            &TrainingReturnPlugin::OnNetContact, this);

        // Create and subscribe to table contact sensor topic
        std::string tableContactSensorTopicName = 
            "~/ping_pong_table/ping_pong_table_no_net/body/table_contact";
        this->tableContactSubscriber = this->node->Subscribe(tableContactSensorTopicName,
            &TrainingReturnPlugin::OnTableContact, this);

        // Create and advertise ball state command topic
        std::string ballStateCmdTopicName = "~/ping_pong_ball/ball_state_cmd";
        ballStateCmdPublisher = this->node->Advertise<gazebo::msgs::Pose>(ballStateCmdTopicName);

        // Initialize ros
        if (!ros::isInitialized())
        {
            std::cout << "ROS was not initialized, initializing...\n";
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
        }

        // Create ros node
        std::cout << "Initializing ros nodes...\n";
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create and subscribe to ros command topic
        ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::String>(
                "/table_tennis/cmd",
                1,
                boost::bind(&TrainingReturnPlugin::OnRosCommand, this, _1),
                ros::VoidPtr(),
                &this->rosSubQueue
            );
        this->rosCmdSubscriber = this->rosNode->subscribe(so);

        // Spin up sub queue helper thread
        this->rosSubQueueThread = std::thread(
            std::bind(&TrainingReturnPlugin::SubQueueThread, this)
        );

        // Create and publish to ros state cmd topic
        ros::AdvertiseOptions ao =
            ros::AdvertiseOptions::create<std_msgs::Float32MultiArray>(
                "/table_tennis/state",
                10,
                boost::bind(&TrainingReturnPlugin::OnRosConnected, this),
                boost::bind(&TrainingReturnPlugin::OnRosDisconnected, this),
                ros::VoidPtr(),
                &this->rosPubQueue
            );

        this->rosStatePublisher = this->rosNode->advertise(ao);

        // Spin up pub queue helper thread
        this->rosPubQueueThread = std::thread(
            std::bind(&TrainingReturnPlugin::PubQueueThread, this)
        );

        this->ballPosition = ignition::math::Vector3d();
        this->ballVelocity = ignition::math::Vector3d();

        std::cout << "World plugin loaded.\n";
    }

    #pragma region Training Environment
    void Reset()
    {
        // pause world
        world->SetPaused(true);

        // reset robot

        // set random ball trajectory
        SetRandomTrajectory();
    }

    void Step()
    {
        // unpause world
        world->SetPaused(false);

        // control robot

        // collect state info

        // calculate reward
    }
    #pragma endregion

    #pragma region Ball Control
    void SetBallState(double x, double y, double z, 
        double vx, double vy, double vz)
    {
        // Skip if no connections to topic
        if (!ballStateCmdPublisher->HasConnections())
        {
            return;
        }

        // Set pose
        ignition::math::Pose3d pose (x, y, z, 0, vx, vy, vz);

        // Create a a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);

        // Publish to ball state cmd topic
        ballStateCmdPublisher->Publish(msg);
    }

    void SetRandomTrajectory()
    {
        // Table size
        const double length = 2.74101;
        const double width = 1.525;

        // Left side bounds
        const double x_left_lower = -1*length / 2;
        const double x_left_upper = -0.25;

        // Right side bounds
        const double x_right_lower = 0.25;
        const double x_right_upper = 1*length / 2;

        // y bounds
        const double y_lower = -1*width/2;
        const double y_upper = width/2;

        // z bounds
        const double z_table = 0.760;
        const double z_lower = 1;
        const double z_upper = 1.25;

        // z velocity bounds
        const double z_vel_lower = 0;
        const double z_vel_upper = 3;

        // Generate random bounce position
        double xb = ignition::math::Rand::DblUniform(x_left_lower, x_left_upper);
        double yb = ignition::math::Rand::DblUniform(y_lower, y_upper);
        double zb = z_table;

        // Generate random starting position
        double x0 = ignition::math::Rand::DblUniform(x_right_lower, x_right_upper);
        double y0 = ignition::math::Rand::DblUniform(y_lower, y_upper);
        double z0 = ignition::math::Rand::DblUniform(z_lower, z_upper);

        // Generate velocity
        double vz = ignition::math::Rand::DblUniform(z_vel_lower, z_vel_upper);
        math::Vector3 vel = GenerateTrajectory(x0, y0, z0, xb, yb, zb, vz);

        // Set ball state
        SetBallState(x0, y0, z0, vel.x, vel.y, vel.z);

        std::string message = "Setting ball to p:{ " + 
            std::to_string(x0) + ", " +
            std::to_string(y0) + ", " +
            std::to_string(z0) + "} v:{ " +
            std::to_string(vel.x) + ", " +
            std::to_string(vel.y) + ", " +
            std::to_string(vel.z) + "}, targeting: b:{ " +
            std::to_string(xb) + ", " +
            std::to_string(yb) + ", " +
            std::to_string(zb) + "}";
        std::cout << message << '\n';
    }
    #pragma endregion

private:
    #pragma region Callbacks
    // Store ball state info when published
    void OnBallStateRetrieved(ConstPosePtr &msg)
    {
        // Extract sub-messages from message
        const msgs::Pose* pose_msg = msg.get();
        msgs::Vector3d pos_msg = pose_msg->position();
        msgs::Quaternion vel_msg = pose_msg->orientation();

        // Extract and store data from messages
        ballPosition.Set(
            pose_msg->position().x(),
            pose_msg->position().y(),
            pose_msg->position().z()
        );
        ballVelocity.Set(
            vel_msg.x(),
            vel_msg.y(),
            vel_msg.z()
        );

        // Publish to ros
        if (this->rosStatePublisher.getNumSubscribers() > 0)
        {
            std_msgs::Float32MultiArray rosmsg;
            rosmsg.data = std::vector<float> {
                (float)ballPosition.X(),
                (float)ballPosition.Y(),
                (float)ballPosition.Z(),
                (float)ballVelocity.X(),
                (float)ballVelocity.Y(),
                (float)ballVelocity.Z()
                };
            this->rosStatePublisher.publish(rosmsg);
        }

        // If z height under threshold, reset
        if (ballPosition.Z() < z_threshold)
        {
            Reset();
        }
    }

    void OnNetContact(ConstContactsPtr &msg)
    {
        if (msg != NULL)
        {
            if (msg->contact_size() > 0)
            {
                Reset();
            }
        }
    }

    void OnRacketContact(ConstContactPtr &msg)
    {
        
    }

    void OnTableContact(ConstContactsPtr &msg)
    {
        if (msg != NULL)
        {
            if (msg->contact_size() > 0)
            {
                // do something?
            }
        }
    }

    void OnRosCommand(const std_msgs::StringConstPtr &_msg)
    {
        std::string command = _msg->data;
        if (command.compare("reset") == 0)
        {
            Reset();
        }
        else if (command.compare("step") == 0)
        {
            Step();
        }
        else
        {
            std::cout << "Invalid command received.\n";
        }
        
    }

    void OnRosConnected()
    {
        std::cout << "ROS connected.\n";
    }

    void OnRosDisconnected()
    {
        std::cout << "ROS disconnected.\n";
    }
    #pragma endregion

    #pragma region Ros Queues
    void SubQueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosSubQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void PubQueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosPubQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
    #pragma endregion

    #pragma region Math
    math::Vector3 GenerateTrajectory(double x0, double y0, double z0,
        double xb, double yb, double zb, double vz)
    {
        // Formula for getting velocity given initial and bounce pos
        double g = -9.81;
        double tb = -vz/g - sqrt(pow(vz, 2) - 2*g*(z0 - zb))/g;
        double vx = (xb - x0)/tb;
        double vy = (yb - y0)/tb;
        
        return math::Vector3(vx, vy, vz);
    }
    #pragma endregion

    #pragma region State info
    // Ball state info
    ignition::math::Vector3d ballPosition;
    ignition::math::Vector3d ballVelocity;

    // World z threshold
    const double z_threshold = 0.6;
    #pragma endregion

    #pragma region Gazebo transport
    // Reference to world pointer
    physics::WorldPtr world;

    // Node for communicating with models
    transport::NodePtr node;

    // Gazebo Publishers
    transport::PublisherPtr ballStateCmdPublisher;

    // Gazebo Subscribers
    transport::SubscriberPtr ballStateSubscriber;
    transport::SubscriberPtr netContactSubscriber;
    transport::SubscriberPtr tableContactSubscriber;
    transport::SubscriberPtr racketContactSubscriber;
    #pragma endregion

    #pragma region ROS transport
    // Node for communicating with ROS
    std::unique_ptr<ros::NodeHandle> rosNode;

    // ROS Publishers
    ros::Publisher rosStatePublisher;

    // ROS Subscribers
    ros::Subscriber rosCmdSubscriber;

    // Queue for subscribed ROS callbacks
    ros::CallbackQueue rosSubQueue;

    // Queue for published ROS callbacks
    ros::CallbackQueue rosPubQueue;

    // Thread for running rosSubQueue
    std::thread rosSubQueueThread;

    // Thread for running rosPubQueue
    std::thread rosPubQueueThread;
    #pragma endregion
};

GZ_REGISTER_WORLD_PLUGIN(TrainingReturnPlugin)
}