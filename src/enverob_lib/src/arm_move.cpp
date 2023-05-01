#include <arm_move.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Quaternion.h>

// 宣告常數
static const std::string PLANNING_GROUP = "tmr_arm";         // 机械臂规划组的名称
static const std::string END_EFFECTOR_LINK = "gripper_link"; // 机械臂终端执行器的名称

// 宣告以moveit控制機械手臂的關節角度的函式
void setJointangle(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
    std::vector<double> joint_group_positions = std::vector<double>{0, 0, 0, 0, 0, 0}); // joint1~6分别代表六个关节的角度,单位为弧度

// 宣告將機械手臂移動到指定位置的函式
void setTargetPosition(moveit::planning_interface::MoveGroupInterface &move_group,
                       moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
                       tf::StampedTransform target);

int main(int argc, char **argv)
{
  // ------------------------ 初始化程式碼 ------------------------
  ros::init(argc, argv, "arm_move");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 创建一个监听器，监听所有tf变换，缓冲10s
  tf::TransformListener listener;
  tf::StampedTransform targetTransform;

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setGoalTolerance(0.005);
  move_group.setEndEffectorLink(END_EFFECTOR_LINK);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // ------------------------ 手臂往信箱移動 ------------------------
  // 將機械手臂移動到初始位置
  std::vector<double> target_joint{-M_PI, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
  setJointangle(move_group, my_plan, target_joint);

  // 旋轉機械手臂，尋找信箱
  double rotation_step = 0.1745;
  double current_direction = target_joint[0] + rotation_step;
  bool find_mailbox = 0;
  while (node_handle.ok() && current_direction <= 0)
  {
    target_joint[0] = current_direction;
    setJointangle(move_group, my_plan, target_joint);
    if (listener.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3)))
    {
      listener.lookupTransform("base", "mailbox", ros::Time(0), targetTransform);
      std::cout << "targetTransform.getOrigin().getX(): " << targetTransform.getOrigin().getX() << std::endl;
      std::cout << "targetTransform.getOrigin().getY(): " << targetTransform.getOrigin().getY() << std::endl;
      std::cout << "targetTransform.getOrigin().getZ(): " << targetTransform.getOrigin().getZ() << std::endl;
      std::cout << "targetTransform.getRotation().getX(): " << targetTransform.getRotation().getX() << std::endl;
      std::cout << "targetTransform.getRotation().getY(): " << targetTransform.getRotation().getY() << std::endl;
      std::cout << "targetTransform.getRotation().getZ(): " << targetTransform.getRotation().getZ() << std::endl;
      std::cout << "targetTransform.getRotation().getW(): " << targetTransform.getRotation().getW() << std::endl;
      find_mailbox = 1;
      break;
    }
    current_direction += rotation_step;
  }

  if (find_mailbox)
  {
    setTargetPosition(move_group, my_plan, targetTransform);
  }
  else
  {
    std::cout << "Can't find mailbox!" << std::endl;
  }

  return 0;
}

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