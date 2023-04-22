#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


const double TARGET_FORCE_Y = 330.0;
const double TARGET_FORCE_Y_THRESHOLD = 20.0;
const double MOVEMENT_STEP = 0.005;
const double ROTATION_STEP = 0.1745;

void move_arm(moveit::planning_interface::MoveGroupInterface &move_group, double force_value);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feedback_controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);                   //控制节点运行的频率,与loop.sleep共同使用
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 指定机械臂规划组的名称
  static const std::string PLANNING_GROUP = "tmr_arm";

  // 创建MoveGroupInterface对象，用于规划和控制机械臂
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // 设置机械臂的允许误差
  move_group.setGoalTolerance(0.005);

  move_arm(move_group, 300);

  return 0;
}

void move_arm(moveit::planning_interface::MoveGroupInterface &move_group, double force_value)
{
    // Get the current state of the robot
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(
    move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()),
    joint_group_positions);

    // 获取机械臂的末端执行器链接名称
    std::string end_effector_link = move_group.getEndEffectorLink();

    // 获取机械臂的当前姿态
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link);

    current_pose.pose.position.y += MOVEMENT_STEP;
    move_group.setPoseTarget(current_pose, end_effector_link);
    move_group.move();

    // --------------------------------------------------------------------------------
    // // Rotate the end effector 10 degrees around the x-axis of the gripper_link
    // current_pose = move_group.getCurrentPose(end_effector_link);
    // //輸出current_pose
    // ROS_INFO("Current Pose: %f, %f, %f, %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    // // rotated_pose.pose.position.y -= 0.02;
    // // rotated_pose.pose.position.x += 0.005;
    // // rotated_pose.pose.position.z += 0.005;
    // current_pose.pose.orientation.x -= 0.087;
    // current_pose.pose.orientation.w += 0.004;
    // std::cout << "current_pose.pose.orientation.x: " << current_pose.pose.orientation.x << std::endl;
    // std::cout << "current_pose.pose.orientation.w: " << current_pose.pose.orientation.w << std::endl;
    // move_group.setPoseTarget(current_pose, end_effector_link);
    // move_group.move();

    
    // --------------------------------------------------------------------------------    
    tf::TransformBroadcaster br; //创建tf广播器
    tf::TransformListener listener;

    // 定義一個固定的轉換 
    tf::Transform gripperRotaion;
    tf::Quaternion q;
    q.setRPY(ROTATION_STEP, 0, 0);
    gripperRotaion.setRotation(q);

    // 發布轉換
    br.sendTransform(tf::StampedTransform(gripperRotaion,ros::Time::now(),"gripper_link","rotated_frame"));

    // 設置轉換
    tf::StampedTransform rotaionTransform;
    listener.lookupTransform("base", "rotated_frame", ros::Time(0), rotaionTransform);
    ROS_INFO("Transform: \n %.2f, %.2f, %.2f, \n %.2f, %.2f, %.2f, %.2f)",
              rotaionTransform.getOrigin().x(),
              rotaionTransform.getOrigin().y(),
              rotaionTransform.getOrigin().z(),
              rotaionTransform.getRotation().w(),
              rotaionTransform.getRotation().x(),
              rotaionTransform.getRotation().y(),
              rotaionTransform.getRotation().z());

    // 宣告儲存gripper幾何姿態的變數
    geometry_msgs::PoseStamped rotated_pose;
    rotated_pose.pose.position.x = rotaionTransform.getOrigin().x();
    rotated_pose.pose.position.y = rotaionTransform.getOrigin().y();
    rotated_pose.pose.position.z = rotaionTransform.getOrigin().z();
    rotated_pose.pose.orientation.w = rotaionTransform.getRotation().w();
    rotated_pose.pose.orientation.x = rotaionTransform.getRotation().x();
    rotated_pose.pose.orientation.y = rotaionTransform.getRotation().y();
    rotated_pose.pose.orientation.z = rotaionTransform.getRotation().z();
    
    move_group.setPoseTarget(rotated_pose, end_effector_link);
    move_group.move();

    current_pose = move_group.getCurrentPose(end_effector_link);
    //輸出current_pose
    ROS_INFO("Transform: \n %.2f, %.2f, %.2f, \n  %.2f, %.2f, %.2f, %.2f)",
              current_pose.pose.position.x, 
              current_pose.pose.position.y, 
              current_pose.pose.position.z,
              current_pose.pose.orientation.w, 
              current_pose.pose.orientation.x, 
              current_pose.pose.orientation.y, 
              current_pose.pose.orientation.z);
}
