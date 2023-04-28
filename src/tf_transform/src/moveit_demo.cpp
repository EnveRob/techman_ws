#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Quaternion.h>

// 宣告以moveit控制機械手臂的關節角度的函式
void setJointangle(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
    std::vector<double> joint_group_positions = std::vector<double>{0, 0, 0, 0, 0, 0}); // joint1~6分别代表六个关节的角度,单位为弧度

// 宣告將機械手臂移動到指定位置的函式
void setTargetPosition(moveit::planning_interface::MoveGroupInterface &move_group,
                       moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
                       double x, double y, double z,
                       double qx, double qy, double qz, double qw);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_demo");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10); // 控制节点运行的频率,与loop.sleep共同使用
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 创建一个监听器，监听所有tf变换，缓冲10s
  tf::TransformListener listener;
  tf::StampedTransform targetTransform;

  // 指定机械臂规划组的名称
  static const std::string PLANNING_GROUP = "tmr_arm";

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setGoalTolerance(0.005);
  // move_group.setPlannerId("RRTstar");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // 將機械手臂移動到初始位置
  std::vector<double> target_joint{-M_PI, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
  setJointangle(move_group, my_plan, target_joint);

  double rotation_step = 0.1745;
  double current_direction = target_joint[0] + rotation_step;
  bool find_mailbox = 0;
  while (node_handle.ok() && current_direction <= 0)
  {
    target_joint[0] = current_direction;
    setJointangle(move_group, my_plan, target_joint);
    if (listener.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3))) // 等待3s，如果3s之后都还没收到消息，那么之前的消息就被丢弃掉。
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
    loop_rate.sleep();
  }

  if (find_mailbox)
  {
    // 呼叫 setTargetPosition 函式，將機械手臂移動到指定位置
    setTargetPosition(move_group, my_plan,
                      targetTransform.getOrigin().getX(),
                      targetTransform.getOrigin().getY(),
                      targetTransform.getOrigin().getZ(),
                      targetTransform.getRotation().getX(),
                      targetTransform.getRotation().getY(),
                      targetTransform.getRotation().getZ(),
                      targetTransform.getRotation().getW());
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
    std::vector<double> joint_group_positions) // joint1~6分别代表六个关节的角度,单位为弧度
{
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  bool joints_reached_goal = false;
  while (joints_reached_goal != 1)
  {
    std::vector<double> current_joint_positions = move_group.getCurrentJointValues();

    for (size_t i = 0; i < joint_group_positions.size(); ++i)
    {
      if (std::abs(abs(current_joint_positions[i] - joint_group_positions[i])) > 0.01) // 0.01为容差值
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
  std::cout << "current direction: " << joint_group_positions[0] << std::endl;
}

void setTargetPosition(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
{
  // 获取机械臂的当前姿态
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(
      move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
      joint_group_positions);
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  // 获取机械臂的末端执行器链接名称
  move_group.setEndEffectorLink("gripper_link");
  std::string end_effector_link = move_group.getEndEffectorLink();

  // 创建要移动到的目标位置
  geometry_msgs::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;

  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);
  ROS_INFO("\ntarget_pose \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
           target_pose.position.x, target_pose.position.y, target_pose.position.z,
           target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  // 规划机械臂的运动路径
  moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);
  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    // 执行机械臂的运动路径
    move_group.execute(my_plan);
    ROS_INFO("Set Target Position");
  }
  else
  {
    // ROS_ERROR("Failed to plan and execute motion");
    ROS_ERROR("Failed to plan: %d", error_code.val);
  }
}