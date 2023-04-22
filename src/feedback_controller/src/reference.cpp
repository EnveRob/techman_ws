#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
 
int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "moveit_fk_demo");
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启线程
    spinner.start();
 
    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
 
    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
 
    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
 
    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);
 
    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);
 
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
 
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move(); //规划+运动
    sleep(1); //停1s钟
 
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    //四元数，设置末端姿态
    target_pose.orientation.x = 0.70692;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.70729;
    //设置末端位置
    target_pose.position.x = 0.2593;
    target_pose.position.y = 0.0636;
    target_pose.position.z = 0.1787;
 
    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();
    // 将目标位姿写入
    arm.setPoseTarget(target_pose);
 
    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);
 
    //输出成功与否的信息
    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   
 
    //让机械臂按照规划的轨迹开始运动
    if(success)
      arm.execute(plan);
    sleep(1);
 
    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
 
    ros::shutdown(); 
 
    return 0;
}
