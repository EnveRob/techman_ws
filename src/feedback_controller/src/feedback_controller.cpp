#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const double TARGET_FORCE_Y = 330.0;
const double TARGET_FORCE_Y_THRESHOLD = 20.0;
const double MOVEMENT_STEP = 0.005;
const double ROTATION_STEP = 0.1745;

void setJointangle(
  moveit::planning_interface::MoveGroupInterface &move_group, 
  double joint1, double joint2, double joint3, 
  double joint4, double joint5, double joint6); //joint1~6分别代表六个关节的角度,单位为弧度
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
  setJointangle(move_group, 3.14/2, 0, -3.14/2, 3.14/2, 0, 0);
  std::cout << "Finish Set Joint Angle!!!" << std::endl;
  move_arm(move_group, 300);

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
    std::cout << "End Effector Link: " << end_effector_link << std::endl;

    // 获取机械臂的当前姿态
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link);
    //輸出current_pose
    ROS_INFO("Current Pose: %f, %f, %f, %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);

    current_pose.pose.position.y += MOVEMENT_STEP;
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(current_pose, end_effector_link);
    move_group.move();

    // Rotate the end effector 10 degrees around the x-axis of the gripper_link
    current_pose = move_group.getCurrentPose(end_effector_link);
    //輸出current_pose
    ROS_INFO("Current Pose: %f, %f, %f, %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    std::string reference_frame = "tool0";
    move_group.setPoseReferenceFrame(reference_frame);
    current_pose.pose.position.y -= 0.02;
    current_pose.pose.position.x += 0.01;
    // rotated_pose.pose.position.z += 0.005;
    // current_pose.pose.orientation.x -= 0.087;
    // current_pose.pose.orientation.w += 0.004;
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(current_pose, end_effector_link);
    move_group.move();
    std::cout << "current_pose.pose.orientation.x: " << current_pose.pose.orientation.x << std::endl;
    std::cout << "current_pose.pose.orientation.w: " << current_pose.pose.orientation.w << std::endl;

    

    current_pose = move_group.getCurrentPose(end_effector_link);
    //輸出current_pose
    ROS_INFO("Current Pose: %f, %f, %f, %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
}
