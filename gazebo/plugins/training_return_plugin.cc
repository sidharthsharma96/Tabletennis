#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>

namespace gazebo
{
class TrainingReturnPlugin : public WorldPlugin
{
public:
  TrainingReturnPlugin() : WorldPlugin()
  {
    printf("Hello World!\n");
  }

public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    this->world = _world;
  }

  void Reset()
  {
    // reset robot

    // remove any balls
  }

  void Step()
  {
    // spawn ball

    // set ball trajectory 

    // control robot

    // collect state info

    // calculate reward
  }

  void GetBallInfo()
  {
    // get current ball pose

    // get current ball velocity
    
  }

  private:
    physics::WorldPtr world;
};

GZ_REGISTER_WORLD_PLUGIN(TrainingReturnPlugin)
} // namespace gazebo