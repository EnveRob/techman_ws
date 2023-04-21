#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
<<<<<<< HEAD
#include <geometry_msgs/Quaternion.h>

// 宣告以moveit控制機械手臂的關節角度的函式
void setJointangle(
  moveit::planning_interface::MoveGroupInterface &move_group, 
  double joint1, double joint2, double joint3, 
  double joint4, double joint5, double joint6); //joint1~6分别代表六个关节的角度,单位为弧度

// 宣告將機械手臂移動到指定位置的函式
void setTargetPosition(moveit::planning_interface::MoveGroupInterface &move_group, 
  double x, double y, double z, 
  double qx, double qy, double qz, double qw);
=======
>>>>>>> add tf_transform/src/moveit_demo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);                   //控制节点运行的频率,与loop.sleep共同使用
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //创建一个监听器，监听所有tf变换，缓冲10s
  tf::TransformListener listener;
<<<<<<< HEAD
  tf::StampedTransform targetTransform;

  // 指定机械臂规划组的名称
  static const std::string PLANNING_GROUP = "tmr_arm";

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // 设置机械臂的允许误差
  move_group.setGoalTolerance(0.005);

  // 呼叫 setJointangle 函式，將機械手臂移動到初始位置
  // setJointangle(move_group, 3.14/2, 0, -3.14/2, 3.14/2, -3.14/2, 0);
  setJointangle(move_group, 3.14/2, 0, -3.14/2, 3.14/2, 0, 0);
  
  double current_direction = 0;
  while (node_handle.ok() && current_direction>=-3.14)
  {
    setJointangle(move_group, 3.14/2, 0, -3.14/2, 3.14/2, current_direction, 0);
    if (listener.waitForTransform("base","mailbox",ros::Time(0),ros::Duration(3))) //等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
    {
      listener.lookupTransform("base","mailbox", ros::Time(0), targetTransform);
      std::cout << "targetTransform.getOrigin().getX(): " << targetTransform.getOrigin().getX() << std::endl;
      std::cout << "targetTransform.getOrigin().getY(): " << targetTransform.getOrigin().getY() << std::endl;
      std::cout << "targetTransform.getOrigin().getZ(): " << targetTransform.getOrigin().getZ() << std::endl;
      std::cout << "targetTransform.getRotation().getX(): " << targetTransform.getRotation().getX() << std::endl;
      std::cout << "targetTransform.getRotation().getY(): " << targetTransform.getRotation().getY() << std::endl;
      std::cout << "targetTransform.getRotation().getZ(): " << targetTransform.getRotation().getZ() << std::endl;
      std::cout << "targetTransform.getRotation().getW(): " << targetTransform.getRotation().getW() << std::endl;
      break;
    }
    current_direction -= 0.314;
    loop_rate.sleep();
  }

  // 呼叫 setTargetPosition 函式，將機械手臂移動到指定位置
  setTargetPosition(move_group, 
    targetTransform.getOrigin().getX(), 
    targetTransform.getOrigin().getY(), 
    targetTransform.getOrigin().getZ(), 
    targetTransform.getRotation().getX(), 
    targetTransform.getRotation().getY(), 
    targetTransform.getRotation().getZ(), 
    targetTransform.getRotation().getW());

  return 0;
}

void setJointangle(
  moveit::planning_interface::MoveGroupInterface &move_group, 
  double joint1, double joint2, double joint3, 
  double joint4, double joint5, double joint6) //joint1~6分别代表六个关节的角度,单位为弧度
{
  // 設定大小為6的vector，用來存放六個關節角度
  std::vector<double> joint_group_positions(6);
  joint_group_positions[0] = joint1;
  joint_group_positions[1] = joint2;
  joint_group_positions[2] = joint3;
  joint_group_positions[3] = joint4;
  joint_group_positions[4] = joint5;
  joint_group_positions[5] = joint6;

  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();
  std::cout << "current direction: " << joint5 << std::endl;
}

void setTargetPosition(moveit::planning_interface::MoveGroupInterface &move_group, 
  double x, double y, double z, 
  double qx, double qy, double qz, double qw)
{
=======
  tf::StampedTransform laserTransform;
  
  //Request
  while (node_handle.ok())
  {
    if (listener.waitForTransform("base","mailbox",ros::Time(0),ros::Duration(10))) //等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
    {
      listener.lookupTransform("base","mailbox", ros::Time(0), laserTransform);
      std::cout << "laserTransform.getOrigin().getX(): " << laserTransform.getOrigin().getX() << std::endl;
      std::cout << "laserTransform.getOrigin().getY(): " << laserTransform.getOrigin().getY() << std::endl;
      std::cout << "laserTransform.getOrigin().getZ(): " << laserTransform.getOrigin().getZ() << std::endl;
      break;
    }
    loop_rate.sleep();
  }

  // 指定机械臂规划组的名称
  static const std::string PLANNING_GROUP = "tmr_arm";

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // 设置机械臂的允许误差
  move_group.setGoalTolerance(0.01);

>>>>>>> add tf_transform/src/moveit_demo
  // 获取机械臂的当前姿态
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(
<<<<<<< HEAD
  move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
  joint_group_positions);
=======
        move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
        joint_group_positions);
>>>>>>> add tf_transform/src/moveit_demo

  // 获取机械臂的末端执行器链接名称
  std::string end_effector_link = move_group.getEndEffectorLink();

  // 创建要移动到的目标位置
  geometry_msgs::Pose target_pose;
<<<<<<< HEAD
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;
  
=======
  target_pose.orientation.w = 1.0;
  target_pose.position.x = laserTransform.getOrigin().getX();
  target_pose.position.y = laserTransform.getOrigin().getY();
  target_pose.position.z = laserTransform.getOrigin().getZ();
  target_pose.orientation.x = laserTransform.getRotation().getX();
  target_pose.orientation.y = laserTransform.getRotation().getY();
  target_pose.orientation.z = laserTransform.getRotation().getZ();

>>>>>>> add tf_transform/src/moveit_demo
  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);

  // 规划机械臂的运动路径
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    // 执行机械臂的运动路径
    move_group.execute(my_plan);
<<<<<<< HEAD
    ROS_INFO("Set Target Position");
=======
>>>>>>> add tf_transform/src/moveit_demo
  }
  else
  {
    ROS_ERROR("Failed to plan and execute motion");
  }
<<<<<<< HEAD
}
=======

//   ros::shutdown();
  return 0;
}
>>>>>>> add tf_transform/src/moveit_demo
