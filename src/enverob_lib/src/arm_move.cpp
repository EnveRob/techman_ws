#include "std_msgs/UInt8.h"
#include "arm_move.h"
#include "new_frame.h"
#include <tf/transform_listener.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace arm_move
{
  void setJointangle(
      ros::NodeHandle nh,
      moveit::planning_interface::MoveGroupInterface &move_group,
      moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
      std::vector<double> joint_group_positions // joint1~6分别代表六个关节的角度,单位为弧度
  )
  {
    // 移動機械手臂
    std::cout << "current direction: " << joint_group_positions[0] << std::endl;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();

    // 等待機械手臂到達目標位置
    bool joints_reached_goal = false;
    std::vector<double> current_joint_positions;
    while (nh.ok() && joints_reached_goal != 1)
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
    ROS_INFO("joints_reached_goal");
  }

  void setTargetPosition(
      ros::NodeHandle nh,
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
    move_group.setStartState(*current_state);
    printf("target_pose \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f\n",
           target_pose.position.x, target_pose.position.y, target_pose.position.z,
           target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    // 规划机械臂的运动路径
    moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      move_group.execute(my_plan);
      ROS_INFO("Set Target Position");

      // 等待機械手臂到達目標位置
      bool reached_goal = false;
      double error = 0.05;
      geometry_msgs::PoseStamped current_pose;

      while (nh.ok() && reached_goal != 1)
      {
        current_pose = move_group.getCurrentPose();
        if (std::abs(current_pose.pose.position.x - target_pose.position.x) <= error &&
            std::abs(current_pose.pose.position.y - target_pose.position.y) <= error &&
            std::abs(current_pose.pose.position.z - target_pose.position.z) <= error)
        {
          reached_goal = true;
        }
        else
        {
          reached_goal = false;
        }
      }

      ROS_INFO("Target Position Reached");
    }
    else
    {
      ROS_ERROR("Failed to plan: %d", error_code.val);
    }
  }

  void setRelativePosition(
      ros::NodeHandle nh,
      moveit::planning_interface::MoveGroupInterface &move_group,
      moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
      std::string reference_frame,
      std::vector<double> value_adjust // x, y, z, theta
  )
  {
    // 获取机械臂的当前姿态
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(
        move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
        joint_group_positions);

    // 创建要移动到的目标位置
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = reference_frame;

    target_pose.pose.position.x = value_adjust[0];
    target_pose.pose.position.y = value_adjust[1];
    target_pose.pose.position.z = value_adjust[2];
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(value_adjust[3], 0, 0);

    // 輸出target_pose
    printf("target_pose \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f\n",
           target_pose.pose.position.x,
           target_pose.pose.position.y,
           target_pose.pose.position.z,
           target_pose.pose.orientation.x,
           target_pose.pose.orientation.y,
           target_pose.pose.orientation.z,
           target_pose.pose.orientation.w);

    // 将目标位置转换为机械臂的姿态
    move_group.setPoseTarget(target_pose);
    move_group.setStartState(*current_state);

    // 规划机械臂的运动路径
    moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      // 执行机械臂的运动路径
      move_group.execute(my_plan);
      ROS_INFO("Set Target Position");

      // 等待機械手臂到達目標位置
      bool reached_goal = false;
      double error = 0.05;
      geometry_msgs::PoseStamped current_pose;

      while (nh.ok() && reached_goal != 1)
      {
        current_pose = move_group.getCurrentPose();
        if (std::abs(current_pose.pose.position.x - target_pose.pose.position.x) <= error &&
            std::abs(current_pose.pose.position.y - target_pose.pose.position.y) <= error &&
            std::abs(current_pose.pose.position.z - target_pose.pose.position.z) <= error)
        {
          reached_goal = true;
        }
        else
        {
          reached_goal = false;
        }
      }

      ROS_INFO("Target Position Reached");
    }
    else
    {
      ROS_ERROR("Failed to plan: %d", error_code.val);
    }
  }

  void setRelativePosition(
      ros::NodeHandle nh,
      moveit::planning_interface::MoveGroupInterface &move_group,
      moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
      std::string reference_frame,
      std::vector<double> value_adjust, // x, y, z, theta
      force_feedback::ForceCallback &feedback_controller)
  {
    move_group.setPlannerId("PRM");
    ROS_WARN("Set Planner to PRM");

    // 获取机械臂的当前姿态
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(
        move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
        joint_group_positions);

    // 创建要移动到的目标位置
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = reference_frame;

    target_pose.pose.position.x = value_adjust[0];
    target_pose.pose.position.y = value_adjust[1];
    target_pose.pose.position.z = value_adjust[2];
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(value_adjust[3], 0, 0);

    // 輸出target_pose
    printf("target_pose \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f\n",
           target_pose.pose.position.x,
           target_pose.pose.position.y,
           target_pose.pose.position.z,
           target_pose.pose.orientation.x,
           target_pose.pose.orientation.y,
           target_pose.pose.orientation.z,
           target_pose.pose.orientation.w);

    // 将目标位置转换为机械臂的姿态
    move_group.setPoseTarget(target_pose);
    move_group.setStartState(*current_state);

    // ------------------------ 规划机械臂的运动路径，每一個控制點之間距離小於1mm ------------------------

    moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      // 均勻增加控制點
      double point_spacing = 0.01; // 每個控制點之間的距離
      moveit_msgs::RobotTrajectory trajectory = my_plan.trajectory_;
      std::vector<trajectory_msgs::JointTrajectoryPoint> new_points;
      new_points.push_back(trajectory.joint_trajectory.points[0]);
      for (int i = 1; i < trajectory.joint_trajectory.points.size(); ++i)
      {
        trajectory_msgs::JointTrajectoryPoint prev_point = trajectory.joint_trajectory.points[i - 1];
        trajectory_msgs::JointTrajectoryPoint curr_point = trajectory.joint_trajectory.points[i];
        std::vector<double> prev_joint_values = prev_point.positions;
        std::vector<double> curr_joint_values = curr_point.positions;
        double distance = 0.0;
        for (int j = 0; j < prev_joint_values.size(); ++j)
        {
          double diff = curr_joint_values[j] - prev_joint_values[j];
          distance += diff * diff;
        }
        distance = std::sqrt(distance);
        int num_points = std::ceil(distance / point_spacing); // 均勻增加控制點
        double time_diff = curr_point.time_from_start.toSec() - prev_point.time_from_start.toSec();
        double time_step = time_diff / num_points;
        for (int j = 1; j < num_points; ++j)
        {
          trajectory_msgs::JointTrajectoryPoint new_point;
          double alpha = static_cast<double>(j) / static_cast<double>(num_points);
          for (int k = 0; k < curr_joint_values.size(); ++k)
          {
            double new_joint_value = prev_joint_values[k] + alpha * (curr_joint_values[k] - prev_joint_values[k]);
            new_point.positions.push_back(new_joint_value);
          }
          new_point.time_from_start = ros::Duration(prev_point.time_from_start.toSec() + j * time_step);
          new_points.push_back(new_point);
        }
        new_points.push_back(curr_point);
      }

      for (int i = 1; i < new_points.size(); ++i)
      {
        double time_diff = (new_points[i].time_from_start - new_points[i - 1].time_from_start).toSec();
        if (time_diff > 0.001)
        {
          new_points[i].time_from_start = new_points[i - 1].time_from_start + ros::Duration(0.001);
        }
      }

      trajectory.joint_trajectory.points = new_points;
      my_plan.trajectory_ = trajectory;

      // 循環執行运动路径計劃
      bool joints_reached_goal = false;
      std::vector<double> current_joint_positions;
      for (size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
      {
        // 執行當前點
        move_group.setJointValueTarget(my_plan.trajectory_.joint_trajectory.points[i].positions);
        move_group.move();

        // 檢查力量是否超過閾值
        printf("feedback_controller.getForceValue().x: %.2f\n", feedback_controller.getForceValue().x);
        if (feedback_controller.getForceValue().x > FORCE_X_THRESHOLD)
        {
          ROS_WARN("Force exceeds threshold, stopping execution");
          break;
        }

        // 檢查是否到達目標位置
        joints_reached_goal = false;
        while (nh.ok() && joints_reached_goal != true)
        {
          current_joint_positions = move_group.getCurrentJointValues();

          for (size_t j = 0; j < my_plan.trajectory_.joint_trajectory.points[i].positions.size(); ++j)
          {
            if (std::abs(abs(current_joint_positions[j] - my_plan.trajectory_.joint_trajectory.points[i].positions[j])) > 0.01) // 0.01为容差值
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
        ROS_INFO("joints_reached_goal");
      }

      ROS_INFO("Motion execution complete.");
    }
    else
    {
      ROS_ERROR("Failed to plan: %d", error_code.val);
    }
    move_group.setPlannerId("RRTstar");
    ROS_WARN("Set Planner back to RRTstar");
  }
}