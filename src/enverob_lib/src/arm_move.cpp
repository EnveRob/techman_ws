#include "std_msgs/UInt8.h"
#include "arm_move.h"
#include <tf/transform_listener.h>

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
    move_group.setPoseTarget(target_pose);
    ROS_INFO("\ntarget_pose \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
             target_pose.position.x, target_pose.position.y, target_pose.position.z,
             target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    // 规划机械臂的运动路径
    moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      move_group.execute(my_plan);
      ROS_INFO("Set Target Position");

      // 等待機械手臂到達目標位置
      geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
      while (current_pose.pose != target_pose)
      {
        current_pose = move_group.getCurrentPose();
      }
      ROS_INFO("Target Position Reached");
    }
    else
    {
      ROS_ERROR("Failed to plan: %d", error_code.val);
    }
  }

  // void setRelativePosition(
  //     moveit::planning_interface::MoveGroupInterface &move_group,
  //     moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
  //     tf::StampedTransform target_pose,
  //     std::string reference_frame,
  //     std::vector<double> value_adjust = {0, 0, 0, 0} // x, y, z, theta
  // )
  // {
  //   tf::StampedTransform reference_transform;
  //   tf::TransformListener listener;
  //   while (ros::ok())
  //   {
  //     try
  //     {
  //       listener.waitForTransform("base", reference_frame, ros::Time(0), ros::Duration(3.0));
  //       listener.lookupTransform("base", reference_frame, ros::Time(0), reference_transform);
  //       break;
  //     }
  //     catch (tf::TransformException &ex)
  //     {
  //       ROS_ERROR("%s", ex.what());
  //       ros::Duration(1.0).sleep();
  //     }
  //   }

  //   // 計算target_pose
  //   geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  //   target_pose.header.frame_id = reference_frame;

  //   tf::Vector3 reference_position = reference_transform.getOrigin();
  //   tf::Vector3 current_position;
  //   current_position.setX(current_pose.pose.position.x);
  //   current_position.setY(current_pose.pose.position.y);
  //   current_position.setZ(current_pose.pose.position.z);

  //   double r = tf::tfDistance(current_position, reference_position);
  //   target_pose.pose.position.y = r * cos(value_adjust[3]);
  //   target_pose.pose.position.z = -r * sin(value_adjust[3]);
  //   target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  //   // 輸出target_pose
  //   ROS_INFO("target_pose: \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f)",
  //            target_pose.pose.position.x,
  //            target_pose.pose.position.y,
  //            target_pose.pose.position.z,
  //            target_pose.pose.orientation.x,
  //            target_pose.pose.orientation.y,
  //            target_pose.pose.orientation.z,
  //            target_pose.pose.orientation.w);

  //   // 将目标位置转换为机械臂的姿态
  //   move_group.setPoseTarget(target_pose, end_effector_link);

  //   // 规划机械臂的运动路径
  //   moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);
  //   if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  //   {
  //     // 执行机械臂的运动路径
  //     move_group.execute(my_plan);
  //     ROS_INFO("Set Target Position");
  //   }
  //   else
  //   {
  //     // ROS_ERROR("Failed to plan and execute motion");
  //     ROS_ERROR("Failed to plan: %d", error_code.val);
  //   }

  //   target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-value_adjust[3], 0, 0);

  //   // 輸出target_pose
  //   ROS_INFO("target_pose: \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f)",
  //            target_pose.pose.position.x,
  //            target_pose.pose.position.y,
  //            target_pose.pose.position.z,
  //            target_pose.pose.orientation.x,
  //            target_pose.pose.orientation.y,
  //            target_pose.pose.orientation.z,
  //            target_pose.pose.orientation.w);

  //   // 将目标位置转换为机械臂的姿态
  //   move_group.setPoseTarget(target_pose, end_effector_link);

  //   // 规划机械臂的运动路径
  //   error_code = move_group.plan(my_plan);
  //   if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  //   {
  //     // 执行机械臂的运动路径
  //     move_group.execute(my_plan);
  //     ROS_INFO("Set Target Position");
  //   }
  //   else
  //   {
  //     // ROS_ERROR("Failed to plan and execute motion");
  //     ROS_ERROR("Failed to plan: %d", error_code.val);
  //   }

  //   target_pose.pose.position.y = 0;
  //   target_pose.pose.position.z = 0;

  //   // 輸出target_pose
  //   ROS_INFO("target_pose: \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f)",
  //            target_pose.pose.position.x,
  //            target_pose.pose.position.y,
  //            target_pose.pose.position.z,
  //            target_pose.pose.orientation.x,
  //            target_pose.pose.orientation.y,
  //            target_pose.pose.orientation.z,
  //            target_pose.pose.orientation.w);

  //   // 将目标位置转换为机械臂的姿态
  //   move_group.setPoseTarget(target_pose, end_effector_link);

  //   // 规划机械臂的运动路径
  //   error_code = move_group.plan(my_plan);
  //   if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  //   {
  //     // 循環執行計劃
  //     bool joints_reached_goal = false;
  //     std::vector<double> current_joint_positions;
  //     for (size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
  //     {
  //       // 執行當前點
  //       move_group.setJointValueTarget(my_plan.trajectory_.joint_trajectory.points[i].positions);
  //       move_group.move();

  //       // 檢查力量是否超過閾值
  //       ROS_INFO("feedback_controller.getForceValue().x: %.2f", feedback_controller.getForceValue().x);
  //       if (feedback_controller.getForceValue().x > FORCE_X_THRESHOLD)
  //       {
  //         ROS_WARN("Force exceeds threshold, stopping execution");
  //         break;
  //       }

  //       // 檢查是否到達目標位置
  //       joints_reached_goal = false;
  //       while (joints_reached_goal != true)
  //       {
  //         current_joint_positions = move_group.getCurrentJointValues();

  //         for (size_t j = 0; j < my_plan.trajectory_.joint_trajectory.points[i].positions.size(); ++j)
  //         {
  //           if (std::abs(abs(current_joint_positions[j] - my_plan.trajectory_.joint_trajectory.points[i].positions[j])) > 0.01) // 0.01为容差值
  //           {
  //             joints_reached_goal = false;
  //             break;
  //           }
  //           else
  //           {
  //             joints_reached_goal = true;
  //           }
  //         }
  //       }
  //       std::cout << "joints_reached_goal" << std::endl;
  //     }

  //     ROS_INFO("Motion execution complete.");
  //   }
  //   else
  //   {
  //     // ROS_ERROR("Failed to plan and execute motion");
  //     ROS_ERROR("Failed to plan: %d", error_code.val);
  //   }
  // }
}