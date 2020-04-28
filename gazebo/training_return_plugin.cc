#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>

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

  ignition::math::Vector3d GetBallInfo()
  {
    // get current ball pose

    // get current ball velocity
    
  }

  void SpawnBall()
  {
    sdf::ElementPtr 
    world->InsertModelFile("model://table_tennis_ball");
    world->
  }

  private:
    physics::WorldPtr world;
};

GZ_REGISTER_WORLD_PLUGIN(TrainingReturnPlugin)
} // namespace gazebo