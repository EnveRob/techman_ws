cmake_minimum_required(VERSION 2.8.3)
project(arduino_listener)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/arduino_listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(arduino_listener src/arduino_listener.cpp)
target_link_libraries(arduino_listener ${catkin_LIBRARIES})
