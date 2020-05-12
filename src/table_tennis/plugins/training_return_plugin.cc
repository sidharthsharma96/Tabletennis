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
#include "table_tennis/CommandAgent.h"
#include "table_tennis/ResetWorld.h"
#include "table_tennis/StepWorld.h"
#include "table_tennis/GetWorldState.h"

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
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            std::cout << "Loading world plugin...\n";
            this->world = _world;

#pragma region Gazebo Initialization
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
#pragma endregion

#pragma region ROS Initialization
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

            // Create and publish to ros ball state topic
            this->rosBallStatePublisher =
                this->rosNode->advertise<std_msgs::Float32MultiArray>(
                    "/table_tennis/ball/state", 10);

            // Create dqn command services
            this->resetWorldService = this->rosNode->advertiseService("/table_tennis/dqn/reset",
                                                                      &TrainingReturnPlugin::Reset,
                                                                      this);

            this->stepWorldService = this->rosNode->advertiseService("/table_tennis/dqn/step",
                                                                     &TrainingReturnPlugin::Step,
                                                                     this);

            this->getWorldStateService = this->rosNode->advertiseService("/table_tennis/dqn/state",
                                                                         &TrainingReturnPlugin::GetState,
                                                                         this);

            // Create agent command service client
            this->agentCommandServiceClient =
                rosNode->serviceClient<table_tennis::CommandAgent>("table_tennis/agent/command");

            this->ballPosition = ignition::math::Vector3d();
            this->ballVelocity = ignition::math::Vector3d();
#pragma endregion

            ros::AsyncSpinner spinner(0);
            spinner.start();

            std::cout << "World plugin loaded.\n";
        }

#pragma region Training Environment
        bool Reset(table_tennis::ResetWorldRequest &req,
                   table_tennis::ResetWorldResponse &res)
        {
            // Reset and pause world
            //world->Reset();
            world->SetPaused(true);

            // Set random ball trajectory
            SetRandomTrajectory();

            // Set random ball destination
            SetBallDestination();

            // Build state
            std::vector<double> state{
                this->ballInitialPosition.X(),
                this->ballInitialPosition.Y(),
                this->ballInitialPosition.Z(),
                this->ballDestination.X(),
                this->ballDestination.Y(),
                this->ballDestination.Z(),
                this->ballInitialVelocity.X(),
                this->ballInitialVelocity.Y(),
                this->ballInitialVelocity.Z()};

            this->state = state;
            res.state = state;

            return true;
        }

        bool Step(table_tennis::StepWorldRequest &req,
                  table_tennis::StepWorldResponse &res)
        {
            ROS_INFO("Stepping world...");

            // Unpause world
            ROS_INFO("Unpausing...");
            world->SetPaused(false);

            // Control robot
            ROS_INFO("Sending control command to agent...");
            table_tennis::CommandAgent command;
            command.request.command = "receive";
            command.request.data = std::vector<double> { 1 };
            this->agentCommandServiceClient.call(command);

            if (!command.response.success)
                ROS_INFO("Control command failed.");
                return false;

            ROS_INFO("Control command success!");

            // Wait for ball to land
            ROS_INFO("Waiting for ball land...");
            bool landed = false;
            while (not landed)
            {
                switch (this->ballState)
                {
                case Landed:
                    ROS_INFO("Ball landed!");
                    landed = true;
                    break;

                case Floor:
                    ROS_INFO("Ball hit floor.");
                    return false;

                case Double_Bounce:
                    ROS_INFO("Ball double bounced.");
                    return false;

                case Net:
                    ROS_INFO("Ball hit net.");
                    return false;
                }
            }

            // calculate reward
            double xd = ballDestination.X();
            double yd = ballDestination.Y();
            double zd = ballDestination.Z();
            double xf = ballLandPosition.X();
            double yf = ballLandPosition.Y();
            double zf = ballLandPosition.Z();
            double vx = ballLandVelocity.X();
            double vy = ballLandVelocity.Y();
            double vz = ballLandVelocity.Z();

            double error = sqrt(pow(xf - xd, 2) + pow(yf - yd, 2) + pow(zf - zd, 2));
            double velreward = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2)) / 100;

            double reward = velreward - error;
            ROS_INFO("Reward: %f", reward);

            // send results
            res.state = state;
            res.reward = reward;
            res.done = true;

            return true;
        }

        bool GetState(table_tennis::GetWorldStateRequest &req,
                      table_tennis::GetWorldStateResponse &res)
        {
            res.state = state;
            return true;
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
            ignition::math::Pose3d pose(x, y, z, 0, vx, vy, vz);

            // Create a a pose message
            gazebo::msgs::Pose msg;
            gazebo::msgs::Set(&msg, pose);

            // Publish to ball state cmd topic
            ballStateCmdPublisher->Publish(msg);

            // Update ball state
            this->ballInitialPosition = ignition::math::Vector3d(x, y, z);
            this->ballInitialVelocity = ignition::math::Vector3d(vx, vy, vz);
            this->ballState = Launched;
        }

        void SetRandomTrajectory()
        {
            // Generate random bounce position
            double xb = ignition::math::Rand::DblUniform(x_left_lower, x_left_upper);
            double yb = ignition::math::Rand::DblUniform(y_lower + 0.1, y_upper - 0.1);
            double zb = z_table;

            // Generate random starting position
            double x0 = ignition::math::Rand::DblUniform(x_right_lower, x_right_upper);
            double y0 = ignition::math::Rand::DblUniform(y_lower + 0.1, y_upper - 0.1);
            double z0 = ignition::math::Rand::DblUniform(z_lower, z_upper);

            // Generate velocity
            double vz = ignition::math::Rand::DblUniform(z_vel_lower, z_vel_upper);
            math::Vector3 vel = GenerateTrajectory(x0, y0, z0, xb, yb, zb, vz);

            ROS_INFO(
                "Setting ball to p:{%f, %f, %f} v:{%f, %f, %f}, targeting:b: { %f, %f, %f } ",
                x0, y0, z0, vel.x, vel.y, vel.z, xb, yb, zb);

            // Set ball state
            SetBallState(x0, y0, z0, vel.x, vel.y, vel.z);
        }

        void SetBallDestination()
        {
            double x = ignition::math::Rand::DblUniform(x_right_lower, x_right_upper);
            double y = ignition::math::Rand::DblUniform(y_lower, y_upper);
            double z = z_table;

            this->ballDestination = ignition::math::Vector3d(x, y, z);
        }
#pragma endregion

    private:
#pragma region Training
        void ReportReward()
        {
        }
#pragma endregion

#pragma region Callbacks
        // Store ball state info when published
        void OnBallStateRetrieved(ConstPosePtr &msg)
        {
            // Extract sub-messages from message
            const msgs::Pose *pose_msg = msg.get();
            msgs::Vector3d pos_msg = pose_msg->position();
            msgs::Quaternion vel_msg = pose_msg->orientation();

            // Extract and store data from messages
            ballPosition.Set(
                pose_msg->position().x(),
                pose_msg->position().y(),
                pose_msg->position().z());
            ballVelocity.Set(
                vel_msg.x(),
                vel_msg.y(),
                vel_msg.z());

            // Publish to ros
            if (this->rosBallStatePublisher.getNumSubscribers() > 0)
            {
                std_msgs::Float32MultiArray rosmsg;
                rosmsg.data = std::vector<float>{
                    (float)ballPosition.X(),
                    (float)ballPosition.Y(),
                    (float)ballPosition.Z(),
                    (float)ballVelocity.X(),
                    (float)ballVelocity.Y(),
                    (float)ballVelocity.Z()};
                this->rosBallStatePublisher.publish(rosmsg);
            }

            // If z height under threshold
            if (ballPosition.Z() < z_threshold)
            {
                if (this->ballState == Returned)
                {
                    ballLandPosition = ballPosition;
                    ballLandVelocity = ballVelocity;
                    this->ballState = Landed;
                }
                else
                {
                    this->ballState = Floor;
                }
            }
        }

        void OnNetContact(ConstContactsPtr &msg)
        {
            if (msg != NULL)
            {
                if (msg->contact_size() > 0)
                {
                    // Update collision vector
                    this->ballState = Net;
                }
            }
        }

        void OnRacketContact(ConstContactsPtr &msg)
        {
            if (msg != NULL)
            {
                if (msg->contact_size() > 0)
                {
                    // Update collision vector
                    this->ballState = Returned;
                }
            }
        }

        void OnTableContact(ConstContactsPtr &msg)
        {
            if (msg != NULL)
            {
                if (msg->contact_size() > 0)
                {
                    // Update collision vector
                    switch (ballState)
                    {
                    case Launched:
                        this->ballState = Bounced;
                        break;

                    case Bounced:
                        this->ballState = Double_Bounce;

                    case Returned:
                        this->ballState = Landed;
                    }
                }
            }
        }
#pragma endregion

#pragma region Math
        math::Vector3 GenerateTrajectory(double x0, double y0, double z0,
                                         double xb, double yb, double zb, double vz)
        {
            // Formula for getting velocity given initial and bounce pos
            double g = -9.81;
            double tb = -vz / g - sqrt(pow(vz, 2) - 2 * g * (z0 - zb)) / g;
            double vx = (xb - x0) / tb;
            double vy = (yb - y0) / tb;

            return math::Vector3(vx, vy, vz);
        }

        math::Vector3 GenerateRandomDestination(bool leftOrRight)
        {
            // Generate random bounce position
            math::Vector3 destination;
            if (leftOrRight)
            {
                double x = ignition::math::Rand::DblUniform(x_left_lower, x_left_upper);
                double y = ignition::math::Rand::DblUniform(y_lower, y_upper);
                double z = z_table;
                destination = math::Vector3(x, y, z);
            }
            else
            {
                double x = ignition::math::Rand::DblUniform(x_right_lower, x_right_upper);
                double y = ignition::math::Rand::DblUniform(y_lower, y_upper);
                double z = z_table;
                destination = math::Vector3(x, y, z);
            }
            return destination;
        }
#pragma endregion

    private:
#pragma region World info
        // Table size
        const double length = 2.74101;
        const double width = 1.525;

        // Left side bounds
        const double x_left_lower = -1 * length / 2;
        const double x_left_upper = -0.5;

        // Right side bounds
        const double x_right_lower = 0.25;
        const double x_right_upper = 1 * length / 2;

        // y bounds
        const double y_lower = -1 * width / 2;
        const double y_upper = width / 2;

        // z bounds
        const double z_table = 0.760;
        const double z_lower = 1.5;
        const double z_upper = 1.8;

        // z velocity bounds
        const double z_vel_lower = 0;
        const double z_vel_upper = 3;

        // World z threshold
        const double z_threshold = 0.1;
#pragma endregion

#pragma region State info
        // World state
        std::vector<double> state;
        ignition::math::Vector3d ballInitialPosition;
        ignition::math::Vector3d ballDestination;
        ignition::math::Vector3d ballInitialVelocity;

        // Ball state info
        ignition::math::Vector3d ballPosition;
        ignition::math::Vector3d ballVelocity;

        enum ball
        {
            Launched,
            Bounced,
            Returned,
            Landed,
            Net,
            Double_Bounce,
            Floor
        };

        ball ballState = Launched;

        // Ball landed
        ignition::math::Vector3d ballLandPosition;
        ignition::math::Vector3d ballLandVelocity;

        // Robot configuration

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
        ros::Publisher rosBallStatePublisher;

        // ROS Services
        ros::ServiceServer resetWorldService;
        ros::ServiceServer stepWorldService;
        ros::ServiceServer getWorldStateService;
        ros::ServiceClient agentCommandServiceClient;
#pragma endregion
    };

    GZ_REGISTER_WORLD_PLUGIN(TrainingReturnPlugin)
} // namespace gazebo