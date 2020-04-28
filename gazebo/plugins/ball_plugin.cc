#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>

namespace gazebo
{
class BallPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Initialize communication node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->GetName());

        // Create and subscribe to world commands topic
        std::string worldTopicName = "~/table_tennis";
        this->worldSubscriber = this->node->Subscribe(worldTopicName, 
            &BallPlugin::OnWorldCommand, this);

        // Create cmd ball state topic
        std::string stateCmdTopicName = "~/" + this->model->GetName() + 
            "/ball_state_cmd";
        this->stateCmdSubscriber = this->node->Subscribe(stateCmdTopicName,
            &BallPlugin::OnStateCmd, this);

        // Create and advertise ball state topic
        std::string stateTopicName = "~/" + this->model->GetName() + "/ball_state";
        this->statePublisher = this->node->Advertise<msgs::Pose>(stateTopicName);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&BallPlugin::OnUpdate, this));
    }


private:
    // Called by the world update start event
    void OnUpdate()
    {
        if (statePublisher->HasConnections())
        {
            PublishState();
        }
    }

    // Handle receiving a command from the world
    void OnWorldCommand(ConstGzStringPtr &msg)
    {
        std::string command = msg->data();
        if (command.compare("state") == 0)
        {
            if (statePublisher->HasConnections())
            {
                PublishState();
            }
        }
    }

    // Handle command to set the state of the ball
    void OnStateCmd(ConstPosePtr &msg)
    {
        // Extract sub-messages from message
        const msgs::Pose* pose_msg = msg.get();
        msgs::Vector3d pos_msg = pose_msg->position();
        msgs::Quaternion vel_msg = pose_msg->orientation();

        // Extract data from messages
        ignition::math::Pose3d pose (
            pos_msg.x(), pos_msg.y(), pos_msg.z(), 0, 0, 0
        );
        ignition::math::Vector3d vel (
            vel_msg.x(), vel_msg.y(), vel_msg.z()
        );

        // Set ball state
        this->model->SetWorldPose(pose);
        this->model->SetLinearVel(vel);
    }

    // Publish the state (pos, vel) of the ball
    void PublishState()
    {
        math::Pose pose = this->model->GetWorldPose();
        math::Vector3 pos = pose.pos;
        math::Vector3 vel = this->model->GetWorldLinearVel();

        msgs::Pose msg;
        msgs::Set(&msg, 
            ignition::math::Pose3d(pos.x, pos.y, pos.z, 
                vel.x, vel.y, vel.z));

        this->statePublisher->Publish(msg);
    }

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Node for communication with world
    transport::NodePtr node;

    // Publishers
    transport::PublisherPtr statePublisher;

    // Subscribers
    transport::SubscriberPtr worldSubscriber;
    transport::SubscriberPtr stateCmdSubscriber;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BallPlugin)
} // namespace gazebo
