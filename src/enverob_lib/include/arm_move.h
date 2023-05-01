#ifndef ARM_MOVE_H
#define ARM_MOVE_H
#define PLANNING_GROUP "tmr_arm"         // 机械臂规划组的名称
#define END_EFFECTOR_LINK "gripper_link" // 机械臂终端执行器的名称

#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace arm_move
{
    // 宣告以moveit控制機械手臂的關節角度的函式
    void setJointangle(
        moveit::planning_interface::MoveGroupInterface &move_group,
        moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
        std::vector<double> joint_group_positions = std::vector<double>{0, 0, 0, 0, 0, 0}); // joint1~6分别代表六个关节的角度,单位为弧度
    // 宣告將機械手臂移動到指定位置的函式
    void setTargetPosition(moveit::planning_interface::MoveGroupInterface &move_group,
                           moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
                           tf::StampedTransform target);
}

#endif
