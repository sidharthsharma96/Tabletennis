#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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
};

GZ_REGISTER_WORLD_PLUGIN(TrainingReturnPlugin)
} // namespace gazebo