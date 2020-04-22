cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ball_rand_vel SHARED ball_rand_vel.cc)
target_link_libraries(ball_rand_vel ${GAZEBO_LIBRARIES})

add_library(training_env_a SHARED training_env_a.cc)
target_link_libraries(training_env_a ${GAZEBO_LIBRARIES})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
