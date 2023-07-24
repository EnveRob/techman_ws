#ifndef ARM_MOVE_H
#define ARM_MOVE_H
// #define FORCE_X_THRESHOLD 360.0

#include <ros/ros.h>
#include "force_feedback.h"

namespace arm_move
{
    // 宣告以moveit控制機械手臂的關節角度的函式
    void setJointangle(
        ros::NodeHandle nh,
        moveit::planning_interface::MoveGroupInterface &move_group,
        moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
        std::vector<double> joint_group_positions = std::vector<double>{0, 0, 0, 0, 0, 0}); // joint1~6分别代表六个关节的角度,单位为弧度
    // 宣告將機械手臂移動到指定位置的函式
    void setTargetPosition(
        ros::NodeHandle nh,
        moveit::planning_interface::MoveGroupInterface &move_group,
        moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
        tf::StampedTransform target);

    void setRelativePosition(
        ros::NodeHandle nh,
        moveit::planning_interface::MoveGroupInterface &move_group,
        moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
        std::string reference_frame,
        std::vector<double> value_adjust // x, y, z, theta
    );

    void setRelativePosition(
        ros::NodeHandle nh,
        moveit::planning_interface::MoveGroupInterface &move_group,
        moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
        std::string reference_frame,
        std::vector<double> value_adjust, // x, y, z, theta
        force_feedback::ForceCallback &feedback_controller);
}

#endif
