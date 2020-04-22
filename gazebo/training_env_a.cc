#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <thread>
//#include "ros/ros.h"
//#include "ros/callback_queue.h"
//#include "ros/subscribe_options.h"
//#include "std_msgs/Float32.h"

namespace gazebo
{
  class TrainingEnvA : public WorldPlugin
  {
    public: TrainingEnvA() : WorldPlugin()
    {
      printf("Hello World!\n");
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
    }

  };

  GZ_REGISTER_WORLD_PLUGIN(TrainingEnvA)
}
