# Tabletennis
A simulation of the Kuka iiwa 7 robot playing table tennis. The simulation is run in ROS and Gazebo.

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
- rosrun table_tennis trajevtory_test.py _dx:=x _dy:=y _dz:=z

replacing x,y,z with relative distances to move the end-effector in operational space.

# Troubleshooting

## Models don't load, or load as giant boxes
If you don't see any of the table tennis related models, or they appear as giant cubes instead, try:
Moving the folders in the Tabletennis/src/table_tennis/models folder into your .gazebo/models folder.

## Simulation is not launching
Make sure you have the required packages installed. The simulation crashing on launch once in a while, so try again.