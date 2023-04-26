#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Quaternion.h>

// 宣告以moveit控制機械手臂的關節角度的函式
void setJointangle(
  moveit::planning_interface::MoveGroupInterface &move_group, 
  moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
  std::vector<double> joint_group_positions = std::vector<double>{0, 0, 0, 0, 0, 0}); //joint1~6分别代表六个关节的角度,单位为弧度

// 宣告將機械手臂移動到指定位置的函式
void setTargetPosition(moveit::planning_interface::MoveGroupInterface &move_group, 
  moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
  double x, double y, double z, 
  double qx, double qy, double qz, double qw);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);                   //控制节点运行的频率,与loop.sleep共同使用
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //创建一个监听器，监听所有tf变换，缓冲10s
  tf::TransformListener listener;
  tf::StampedTransform targetTransform;

  // 指定机械臂规划组的名称
  static const std::string PLANNING_GROUP = "tmr_arm";

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setGoalTolerance(0.005);
  move_group.setPlannerId("RRTstar");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // 將機械手臂移動到初始位置
  std::vector<double> target_joint{3.14/2, 0, -3.14/2, 3.14/2, 0, -3.14/2};
  // std::vector<double> target_joint{0, 0, 0, 0, 3.14/2, -3.14/2};
  setJointangle(move_group, my_plan, target_joint);

  double rotation_step = 0.1745;
  double current_direction = -rotation_step;
  bool find_mailbox = 0;
  while (node_handle.ok() && current_direction>=-3.14)
  {
    target_joint[4] = current_direction;
    setJointangle(move_group, my_plan, target_joint);
    if (listener.waitForTransform("base","mailbox",ros::Time(0),ros::Duration(3))) //等待3s，如果3s之后都还没收到消息，那么之前的消息就被丢弃掉。
    {
      listener.lookupTransform("base","mailbox", ros::Time(0), targetTransform);
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
    current_direction -= rotation_step;
    loop_rate.sleep();
  }

  if(find_mailbox)
  {
    // 呼叫 setTargetPosition 函式，將機械手臂移動到指定位置
    setTargetPosition(move_group,  my_plan,
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
  std::vector<double> joint_group_positions) //joint1~6分别代表六个关节的角度,单位为弧度
{
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  bool joints_reached_goal = false;
  while(joints_reached_goal!=1)
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
  std::cout << "current direction: " << joint_group_positions[4] << std::endl;
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

  // tf::Quaternion q(target_pose.orientation.x, 
  //                  target_pose.orientation.y, 
  //                  target_pose.orientation.z, 
  //                  target_pose.orientation.w);

  // // 定義旋轉軸和旋轉角度
  // tf::Vector3 axis(1, 0, 0);
  // double angle = M_PI;

  // // 將旋轉軸和旋轉角度轉換為四元數
  // tf::Quaternion rotation;
  // rotation.setRotation(axis, angle);

  // q *= rotation;
  // q.normalize();

  // target_pose.orientation.x = q.x();
  // target_pose.orientation.y = q.y();
  // target_pose.orientation.z = q.z();
  // target_pose.orientation.w = q.w();
  
  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);
  std::cout << "target_pose" << std::endl;
  std::cout << target_pose.position.x << std::endl;
  std::cout << target_pose.position.y << std::endl;
  std::cout << target_pose.position.z << std::endl;
  std::cout << target_pose.orientation.x << std::endl;
  std::cout << target_pose.orientation.y << std::endl;
  std::cout << target_pose.orientation.z << std::endl;
  std::cout << target_pose.orientation.w << std::endl;

  // 规划机械臂的运动路径
  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  moveit_msgs::MoveItErrorCodes error_code = move_group.plan(my_plan);
  // if (success)
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