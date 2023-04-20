#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const double TARGET_FORCE_Y = 330.0;
const double TARGET_FORCE_Y_THRESHOLD = 20.0;
const double MOVEMENT_STEP = 0.005;
const double ROTATION_STEP = 0.1745;

moveit::planning_interface::MoveGroupInterface move_group("tmr_arm");
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
ros::Subscriber sub_force;
std::string end_effector_link = move_group.getEndEffectorLink();
std::string gripper_link = "gripper_link";

void forceCallback(const geometry_msgs::Vector3& msg)
{
  ROS_INFO("Force_x: %f", msg.x);
  ROS_INFO("Force_y: %f", msg.y);
  ROS_INFO("Force_z: %f", msg.z);
  
  move_arm(msg.y);
}

void move_arm(double force_value)
{
    // Get the current state of the robot
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(
    move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
    joint_group_positions);

    // 获取机械臂的当前姿态
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link);

    // Check the force value
    if (force_value > TARGET_FORCE_Y + TARGET_FORCE_Y_THRESHOLD)
    {
      current_pose.pose.position.y += MOVEMENT_STEP;
      move_group.setPoseTarget(current_pose, end_effector_link);
      move_group.move();

      // Rotate the end effector 10 degrees around the x-axis of the gripper_link
      geometry_msgs::PoseStamped rotated_pose = current_pose;
      rotated_pose.pose.position.y -= 0.02;
      rotated_pose.pose.position.x += 0.005;
      rotated_pose.pose.position.z += 0.005;
      rotated_pose.pose.orientation.x += 0.173648;
      rotated_pose.pose.orientation.w += 0.984808;
      move_group.setPoseTarget(rotated_pose, end_effector_link);

      // 规划机械臂的运动路径
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success)
      {
        // 执行机械臂的运动路径
        move_group.execute(my_plan);
        ROS_INFO("Set Target Position");
      }
      else
      {
        ROS_ERROR("Failed to plan and execute motion");
      }
    }
    else if (force_value < TARGET_FORCE_Y - TARGET_FORCE_Y_THRESHOLD)
    {
      current_pose.pose.position.y -= MOVEMENT_STEP;
      move_group.setPoseTarget(current_pose, end_effector_link);
      
      // 规划机械臂的运动路径
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success)
      {
        // 执行机械臂的运动路径
        move_group.execute(my_plan);
        ROS_INFO("Set Target Position");
      }
      else
      {
        ROS_ERROR("Failed to plan and execute motion");
      }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feedback_controller");
  ros::NodeHandle n;

  // 添加訂閱器
  sub_force = n.subscribe("/force_data", 10, forceCallback);

  ros::spin();

  return 0;
}
