cmake_minimum_required(VERSION 3.0.2)
project(feedback_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rosconsole
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(feedback_controller src/feedback_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(feedback_controller ${catkin_LIBRARIES})

