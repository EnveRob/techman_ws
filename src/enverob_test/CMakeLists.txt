project(enverob_test)

find_package(catkin REQUIRED
    COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    DEPENDS
    EIGEN3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
add_executable(Enverob_test_Cpp_tutorial src/Enverob_test.cpp)
add_dependencies(Enverob_test_Cpp_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Enverob_test_Cpp_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})