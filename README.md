# Tabletennis
A simulation of the Kuka iiwa 7 robot playing table tennis. The simulation is run in ROS and Gazebo.

# Overview
The project is a ROS1 (Kinetic) / Gazebo workspace with two main pieces under `src/`:
- `iiwa_stack` - the vendored KUKA iiwa ROS driver/description/MoveIt config used to simulate and control the arm.
- `table_tennis` - the project's own package:
  - `plugins/ball_plugin.cc` - a Gazebo model plugin that publishes the ball's pose/velocity and lets it be set externally.
  - `plugins/training_return_plugin.cc` - a Gazebo world plugin that randomizes ball launches and target landing spots, tracks net/table/floor contacts, and exposes ROS services (`reset`, `step`, `state`, `demo`) for driving an episode.
  - `scripts/ik_solver.cpp` - KDL-based inverse/forward kinematics services for the iiwa arm.
  - `scripts/table_tennis_agent.py` - subscribes to ball state, estimates an intercept point, and commands a joint trajectory to return the ball.
  - `nn/` - a small PyTorch model (`action_net.pt` / `dqn.py`) intended to choose the paddle's return velocity and orientation.

# Dependencies
In order to run the simulation, you must have ROS Kinetic installed.
You can install it following the instructions [here](http://wiki.ros.org/kinetic/Installation).

You will also need a few additional packages:
- [ros_control](http://wiki.ros.org/ros_control)
- [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)


# Building
Clone the repository into a folder, then run these commands from a terminal in that folder.
- rosdep install --from-paths src --ignore-src -r -y
- catkin build

# Source workspace
Before running any commands using the package, you must first source the workspace by running this command in the root workspace folder:
- . activate

# Launching the world
Running the command below will start up the simulation in Gazebo
- roslaunch table_tennis table_tennis_world.launch

After starting up the world, you can do a few things:

## Simulate the robot playing table tennis
Run this command:
- rosservice call /table_tennis/demo "[10]"

replacing 10 with the number of iterations to simulate

## Move the robot to a Operational Space pose
Run this command:
- rosrun table_tennis request_pose.py _pos:='[x,y,z]' _rot:='[1,0,0,0,1,0,0,0,1]'

replacing x,y,z with the operational space coordinates, and the elements in _rot with those of a valid rotational matrix. You can omit _pos or _rot to keep it unchanged in the trajectory.

## Plan paths by clicking and dragging the end-effector using RViz
Close the simulation, if you have it running.
Launch by running this command:
- roslaunch table_tennis table_tennis_world.launch rviz:=true

This will launch RViz as well, letting you plan paths using its gui.

## Move the robot incrementally in Operational Space
Run this command:
- rosrun table_tennis trajectory_test.py _dx:=x _dy:=y _dz:=z

replacing x,y,z with relative distances to move the end-effector in operational space.

# Known limitations
- The reinforcement-learning training loop in `training_return_plugin.cc` (`reset`/`step` services) has parts of its world-reset logic commented out, so it isn't fully wired up for training runs.
- The trained network's predicted paddle velocity/orientation is currently overridden with hardcoded values in `table_tennis_agent.py`, so `nn/action_net.pt` isn't actually driving the return shot yet.

# Troubleshooting

## Models don't load, or load as giant boxes
If you don't see any of the table tennis related models, or they appear as giant cubes instead, try:
Moving the folders in the Tabletennis/src/table_tennis/models folder into your .gazebo/models folder.

## Simulation is not launching
Make sure you have the required packages installed. The simulation crashing on launch once in a while, so try again.