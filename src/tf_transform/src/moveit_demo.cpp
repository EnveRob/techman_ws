#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);                   //控制节点运行的频率,与loop.sleep共同使用
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //创建一个监听器，监听所有tf变换，缓冲10s
  tf::TransformListener listener;
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
      std::cout << "laserTransform.getRotation().getX(): " << laserTransform.getRotation().getX() << std::endl;
      std::cout << "laserTransform.getRotation().getY(): " << laserTransform.getRotation().getY() << std::endl;
      std::cout << "laserTransform.getRotation().getZ(): " << laserTransform.getRotation().getZ() << std::endl;
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

  // 获取机械臂的当前姿态
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(
        move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
        joint_group_positions);

  // 获取机械臂的末端执行器链接名称
  std::string end_effector_link = move_group.getEndEffectorLink();

  // 创建要移动到的目标位置
  geometry_msgs::Pose target_pose;
  target_pose.position.x = laserTransform.getOrigin().getX();
  target_pose.position.y = laserTransform.getOrigin().getY();
  target_pose.position.z = laserTransform.getOrigin().getZ();

  // Set the target orientation
  // target_pose.orientation.w = 1.0;
  // target_pose.orientation.x = laserTransform.getRotation().getX();
  // target_pose.orientation.y = laserTransform.getRotation().getY();
  // target_pose.orientation.z = laserTransform.getRotation().getZ();
  tf2::Quaternion orientation;
  orientation.setRPY(
    laserTransform.getRotation().getX(), 
    laserTransform.getRotation().getY(), 
    laserTransform.getRotation().getZ()
  ); // Roll-Pitch-Yaw angles
  target_pose.orientation = tf2::toMsg(orientation);
  std::cout << "orientation w: " << target_pose.orientation.w << std::endl;
  std::cout << "orientation x: " << target_pose.orientation.x << std::endl;
  std::cout << "orientation y: " << target_pose.orientation.y << std::endl;
  std::cout << "orientation z: " << target_pose.orientation.z << std::endl;

  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);

  // 规划机械臂的运动路径
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    // 执行机械臂的运动路径
    move_group.execute(my_plan);
  }
  else
  {
    ROS_ERROR("Failed to plan and execute motion");
  }

//   ros::shutdown();
  return 0;
}