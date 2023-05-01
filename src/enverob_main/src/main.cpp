#include <ros/ros.h>
#include "arm_move.h"
#include "new_frame.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

void cameraCallback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{
    // ------------------------ 初始化程式碼 ------------------------
    ros::init(argc, argv, "enverob_main");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 創建一個Subscriber，訂閱camera_data
    ros::Subscriber sub = nh.subscribe("camera_data", 10, &cameraCallback);

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
    arm_move::setJointangle(move_group, my_plan, target_joint);

    // 旋轉機械手臂，尋找信箱
    double rotation_step = 0.1745;
    double current_direction = target_joint[0] + rotation_step;
    bool find_mailbox = 0;
    while (nh.ok() && current_direction <= 0)
    {
        target_joint[0] = current_direction;
        arm_move::setJointangle(move_group, my_plan, target_joint);
        if (listener.waitForTransform("base", "mailbox", ros::Time(0), ros::Duration(3)))
        {
            listener.lookupTransform("base", "mailbox", ros::Time(0), targetTransform);
            ROS_INFO("targetTransform \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
                     targetTransform.getOrigin().x(), targetTransform.getOrigin().y(), targetTransform.getOrigin().z(),
                     targetTransform.getRotation().x(), targetTransform.getRotation().y(), targetTransform.getRotation().z(), targetTransform.getRotation().w());
            find_mailbox = 1;
            break;
        }
        current_direction += rotation_step;
    }

    if (find_mailbox)
    {
        arm_move::setTargetPosition(move_group, my_plan, targetTransform);
    }
    else
    {
        std::cout << "Can't find mailbox!" << std::endl;
    }
    // ------------------------  ------------------------

    return 0;
}

void cameraCallback(const std_msgs::String::ConstPtr &msg)
{
    // ------------------- 將相機資訊msg->data轉換成vector -------------------
    std::stringstream ss(msg->data.c_str());
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }
    std::vector<double> v;
    for (const auto &token : tokens)
    {
        v.push_back(std::stod(token));
    }

    ROS_INFO("Mailbox position relative to camera \n(x, y, z): %.2f, %.2f, %.2f, \n(Rx, Ry, Rz): %.2f, %.2f, %.2f",
             v[0], v[1], v[2],
             v[3], v[4], v[5]);

    // ------------------- 發布信箱座標 -------------------
    new_frame::fixedFrame_add(v, "camera", "mailbox");
}