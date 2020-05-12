cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(table_tennis)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  control_msgs
  actionlib
  message_generation
)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

add_service_files(
    FILES 
    CommandAgent.srv
    SolveIKPose.srv
    SolveIKVelocity.srv
    ResetWorld.srv
    StepWorld.srv
    GetWorldState.srv
)

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${actionlib_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})
link_directories(${kdl_parser_LIBRARY_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(ball_plugin SHARED plugins/ball_plugin.cc)
target_link_libraries(ball_plugin ${GAZEBO_LIBRARIES})

add_library(training_return_plugin SHARED plugins/training_return_plugin.cc)
target_link_libraries(training_return_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

# Add executable test scripts
add_executable(spawn_ball tests/spawn_ball.cc)
target_link_libraries(spawn_ball ${GAZEBO_LIBRARIES})

add_executable(iiwa_ros_test tests/iiwa_ros_test.cpp)
target_link_libraries(iiwa_ros_test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(iiwa_ros_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(kdl_test tests/kdl_test.cpp)
target_link_libraries(kdl_test ${catkin_LIBRARIES})

add_executable(ik_solver scripts/ik_solver.cpp)
target_link_libraries(ik_solver ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
