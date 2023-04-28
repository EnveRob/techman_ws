#include "ros/ros.h"
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>

const double TARGET_FORCE_Y = 330.0;
const double TARGET_FORCE_Y_THRESHOLD = 20.0;
const double MOVEMENT_STEP = 0.005;
const double ROTATION_STEP = 0.1745;

void move_arm(moveit::planning_interface::MoveGroupInterface &move_group, double force_value);

// 新增座標系函數
void newFrame(moveit::planning_interface::MoveGroupInterface &move_group,
              double x, double y, double z,
              double qx, double qy, double qz, double qw);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feedback_controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(10); // 控制节点运行的频率,与loop.sleep共同使用
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 指定机械臂规划组的名称
  static const std::string PLANNING_GROUP = "tmr_arm";

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setGoalTolerance(0.005);
  move_group.setPlannerId("LBKPIECE ");
  // move_group.setPlanningTime(10.0);

  // 设置机械臂的允许误差
  move_group.setGoalTolerance(0.001);

  move_arm(move_group, 300);

  return 0;
}

void move_arm(moveit::planning_interface::MoveGroupInterface &move_group, double force_value)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Get the current pose of the end effector link
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  // 获取机械臂的末端执行器链接名称
  move_group.setEndEffectorLink("gripper_link");
  std::string end_effector_link = move_group.getEndEffectorLink();

  // 得到mailbox的geometry_msgs
  tf::StampedTransform mailbox_transform;
  tf::TransformListener listener;
  while (ros::ok())
  {
    try
    {
      listener.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("base", "mailbox", ros::Time(0), mailbox_transform);
      break;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  // Calculate the target pose
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "mailbox";

  tf::Vector3 mailbox_position = mailbox_transform.getOrigin();
  tf::Vector3 current_position;
  current_position.setX(current_pose.pose.position.x);
  current_position.setY(current_pose.pose.position.y);
  current_position.setZ(current_pose.pose.position.z);

  double theta = M_PI / 4;
  double r = tf::tfDistance(current_position, mailbox_position);
  target_pose.pose.position.y = r * cos(theta);
  target_pose.pose.position.z = -r * sin(theta);
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  // 輸出target_pose
  ROS_INFO("target_pose: \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f)",
           target_pose.pose.position.x,
           target_pose.pose.position.y,
           target_pose.pose.position.z,
           target_pose.pose.orientation.x,
           target_pose.pose.orientation.y,
           target_pose.pose.orientation.z,
           target_pose.pose.orientation.w);

  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);

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

  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-theta, 0, 0);
  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);

  // 輸出target_pose
  ROS_INFO("target_pose: \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f)",
           target_pose.pose.position.x,
           target_pose.pose.position.y,
           target_pose.pose.position.z,
           target_pose.pose.orientation.x,
           target_pose.pose.orientation.y,
           target_pose.pose.orientation.z,
           target_pose.pose.orientation.w);

  // 规划机械臂的运动路径
  error_code = move_group.plan(my_plan);
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

  target_pose.pose.position.y = 0;
  target_pose.pose.position.z = 0;

  // 輸出target_pose
  ROS_INFO("target_pose: \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f)",
           target_pose.pose.position.x,
           target_pose.pose.position.y,
           target_pose.pose.position.z,
           target_pose.pose.orientation.x,
           target_pose.pose.orientation.y,
           target_pose.pose.orientation.z,
           target_pose.pose.orientation.w);

  // 将目标位置转换为机械臂的姿态
  move_group.setPoseTarget(target_pose, end_effector_link);

  // 规划机械臂的运动路径
  error_code = move_group.plan(my_plan);
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

void newFrame(moveit::planning_interface::MoveGroupInterface &move_group, std::string trarget_frame_id)
{
  // 设置目标位置所使用的参考坐标系
  static tf2_ros::StaticTransformBroadcaster static_br_tool2mailbox;
  geometry_msgs::TransformStamped tool2mailbox;

  // Get the current pose of the end effector link
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  tf::Quaternion q;
  q.setRPY(M_PI, 0, 0);
  tool2mailbox.transform.rotation.x = q.x();
  tool2mailbox.transform.rotation.y = q.y();
  tool2mailbox.transform.rotation.z = q.z();
  tool2mailbox.transform.rotation.w = q.w();
  tool2mailbox.transform.translation.x = current_pose.pose.position.x;
  tool2mailbox.transform.translation.y = current_pose.pose.position.y;
  tool2mailbox.transform.translation.z = current_pose.pose.position.z;

  // 设置 transform 的时间戳、参考系名称和子参考系名称
  tool2mailbox.header.stamp = ros::Time::now();
  tool2mailbox.header.frame_id = "gripper_link";
  tool2mailbox.child_frame_id = trarget_frame_id;

  // 发布 transform
  static_br_tool2mailbox.sendTransform(tool2mailbox);

  // 聽取轉換
  tf::TransformListener listener;
  tf::StampedTransform mailbox_wall;

  while (ros::ok())
  {
    try
    {
      listener.waitForTransform("base", trarget_frame_id, ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("base", trarget_frame_id, ros::Time(0), mailbox_wall);
      ROS_INFO("Transform: \n%.2f, %.2f, %.2f, \n%.2f, %.2f, %.2f, %.2f)",
               mailbox_wall.getOrigin().getX(),
               mailbox_wall.getOrigin().getY(),
               mailbox_wall.getOrigin().getZ(),
               mailbox_wall.getRotation().getX(),
               mailbox_wall.getRotation().getY(),
               mailbox_wall.getRotation().getZ(),
               mailbox_wall.getRotation().getW());
      break;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  static tf2_ros::StaticTransformBroadcaster static_br_base2mailboxWall;
  geometry_msgs::TransformStamped base2mailboxWall;
  base2mailboxWall.transform.rotation.x = mailbox_wall.getRotation().getX();
  base2mailboxWall.transform.rotation.y = mailbox_wall.getRotation().getY();
  base2mailboxWall.transform.rotation.z = mailbox_wall.getRotation().getZ();
  base2mailboxWall.transform.rotation.w = mailbox_wall.getRotation().getW();
  base2mailboxWall.transform.translation.x = mailbox_wall.getOrigin().getX();
  base2mailboxWall.transform.translation.y = mailbox_wall.getOrigin().getY();
  base2mailboxWall.transform.translation.z = mailbox_wall.getOrigin().getZ();

  // 设置 transform 的时间戳、参考系名称和子参考系名称
  base2mailboxWall.header.stamp = ros::Time::now();
  base2mailboxWall.header.frame_id = "base";
  base2mailboxWall.child_frame_id = trarget_frame_id;

  // 发布 transform
  static_br_base2mailboxWall.sendTransform(base2mailboxWall);
  static_br_tool2mailbox.sendTransform(std::vector<geometry_msgs::TransformStamped>());
}
