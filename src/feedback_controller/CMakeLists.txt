cmake_minimum_required(VERSION 2.8.3)
project(feedback_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/feedback_controller.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(feedback_controller src/feedback_controller.cpp)
target_link_libraries(feedback_controller ${catkin_LIBRARIES})
