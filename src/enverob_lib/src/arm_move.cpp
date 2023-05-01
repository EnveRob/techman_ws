#include <arm_move.h>

namespace arm_move
{
  void setJointangle(
      moveit::planning_interface::MoveGroupInterface &move_group,
      moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
      std::vector<double> joint_group_positions // joint1~6分别代表六个关节的角度,单位为弧度
  )
  {
    std::cout << "current direction: " << joint_group_positions[0] << std::endl;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();

    // 等待機械手臂到達目標位置
    bool joints_reached_goal = false;
    std::vector<double> current_joint_positions;
    while (joints_reached_goal != 1)
    {
      current_joint_positions = move_group.getCurrentJointValues();
      for (size_t i = 0; i < joint_group_positions.size(); ++i)
      {
        if (std::abs(abs(current_joint_positions[i] - joint_group_positions[i])) > 0.01)
        {
          joints_reached_goal = false;
          break;
        }
        else
        {
          joints_reached_goal = true;
        }
      }
    }
    std::cout << "joints_reached_goal" << std::endl;
  }

  void setTargetPosition(
      moveit::planning_interface::MoveGroupInterface &move_group,
      moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
      tf::StampedTransform target)
  {
    // 获取机械臂的当前姿态
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(
        move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
        joint_group_positions);

    // 创建要移动到的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.position.x = target.getOrigin().getX();
    target_pose.position.y = target.getOrigin().getY();
    target_pose.position.z = target.getOrigin().getZ();
    target_pose.orientation.x = target.getRotation().getX();
    target_pose.orientation.y = target.getRotation().getY();
    target_pose.orientation.z = target.getRotation().getZ();
    target_pose.orientation.w = target.getRotation().getW();

    // 将目标位置转换为机械臂的姿态
    move_group.setPoseTarget(target_pose, END_EFFECTOR_LINK);
    ROS_INFO("\ntarget_pose \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
             target_pose.position.x, target_pose.position.y, target_pose.position.z,
             target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    // 规划机械臂的运动路径
    moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      move_group.execute(my_plan);
      ROS_INFO("Set Target Position");
    }
    else
    {
      ROS_ERROR("Failed to plan: %d", error_code.val);
    }
  }
}