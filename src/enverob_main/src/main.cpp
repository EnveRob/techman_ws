#include <ros/ros.h>
#include "main.h"
#include "arm_move.h"
#include "new_frame.h"
#include "force_feedback.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

bool find_mailbox = 1;

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
    force_feedback::ForceCallback forceSubsriber;

    // 创建一个监听器，监听所有tf变换，缓冲10s
    tf::TransformListener listener;
    tf::TransformListener listener_check;
    tf::StampedTransform targetTransform;

    // 创建MoveGroupInterface对象，用于规划和控制机械臂
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setGoalTolerance(0.002);
    move_group.setPlannerId("RRTstar");
    move_group.setEndEffectorLink(END_EFFECTOR_LINK);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // ------------------------ 手臂往距離信箱40公分處移動 ------------------------
    // 將機械手臂移動到初始位置，target_joint[0]的最小值是-M_PI*1.5
    // std::vector<double> target_joint{-M_PI, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    ROS_INFO("Move to initial position");
    std::vector<double> target_joint{-M_PI * 0.75, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    arm_move::setJointangle(move_group, my_plan, target_joint);

    // 旋轉機械手臂，尋找信箱
    find_mailbox = 0;
    double current_direction = target_joint[0] + ROTATION_STEP;
    // while (nh.ok() && current_direction <= 0)
    while (nh.ok() && current_direction >= -M_PI * 1.25)
    {
        target_joint[0] = current_direction;
        arm_move::setJointangle(move_group, my_plan, target_joint);
        if (listener.waitForTransform("base", "mailbox_40cm_rear", ros::Time(0), ros::Duration(3)))
        {
            listener.lookupTransform("base", "mailbox_40cm_rear", ros::Time(0), targetTransform);
            ROS_INFO("targetTransform \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
                     targetTransform.getOrigin().x(), targetTransform.getOrigin().y(), targetTransform.getOrigin().z(),
                     targetTransform.getRotation().x(), targetTransform.getRotation().y(), targetTransform.getRotation().z(), targetTransform.getRotation().w());
            find_mailbox = 1;
            break;
        }
        // current_direction += ROTATION_STEP;
        current_direction -= ROTATION_STEP;
    }

    if (find_mailbox)
    {
        arm_move::setTargetPosition(move_group, my_plan, targetTransform);
        find_mailbox = 0;
    }
    else
    {
        std::cout << "Can't find mailbox!" << std::endl;
        return 0;
    }

    // ------------------------ 手臂再次往距離信箱40公分處移動 ------------------------
    new_frame::waitforTransform("mailbox_40cm_rear", targetTransform);
    find_mailbox = 1;

    if (find_mailbox)
    {
        arm_move::setTargetPosition(move_group, my_plan, targetTransform);
        find_mailbox = 0;
    }
    else
    {
        std::cout << "Can't find mailbox!" << std::endl;
        return 0;
    }

    // ------------------------ 手臂向上至正對信箱口 ------------------------
    std::vector<double> mailbox_opening_40cm_rear_position;
    new_frame::fixedFrame_add(mailbox_opening_40cm_rear_position, "camera", "mailbox_opening_40cm_rear");
    new_frame::waitforTransform("mailbox_opening_40cm_rear", targetTransform);
    find_mailbox = 1;

    arm_move::setTargetPosition(move_group, my_plan, targetTransform);

    // // ------------------------ 手臂往信箱口移動，嘗試讓信接觸到信箱 ------------------------
    // std::vector<double> back_of_mailbox_transform;
    // new_frame::fixedFrame_add(back_of_mailbox_transform, "camera", "back_of_mailbox");

    // // ------------------------ 手臂往後移動10cm ------------------------

    return 0;
}

void cameraCallback(const std_msgs::String::ConstPtr &msg)
{
    if (find_mailbox == 0)
    {
        // ------------------------ 將相機資訊msg->data轉換成vector ------------------------
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

        ROS_INFO("Mailbox position related to camera \n(x, y, z): %.2f, %.2f, %.2f, \n(Rx, Ry, Rz): %.2f, %.2f, %.2f",
                 v[0], v[1], v[2],
                 v[3], v[4], v[5]);
        std::vector<double> mailbox_40cm_rear_transform{v[0], v[1], v[2]};
        tf::Quaternion q;
        q.setRPY(v[3], v[4], v[5]); // 以弧度為單位

        mailbox_40cm_rear_transform.push_back(q.x());
        mailbox_40cm_rear_transform.push_back(q.y());
        mailbox_40cm_rear_transform.push_back(q.z());
        mailbox_40cm_rear_transform.push_back(q.w());

        // ------------------------ 將座標改為z軸向上 ------------------------
        q.setX(mailbox_40cm_rear_transform[3]);
        q.setY(mailbox_40cm_rear_transform[4]);
        q.setZ(mailbox_40cm_rear_transform[5]);
        q.setW(mailbox_40cm_rear_transform[6]);

        // 定義旋轉軸和旋轉角度
        tf::Vector3 axis(1, 0, 0);
        double angle = -M_PI / 2;

        // 將旋轉軸和旋轉角度轉換為四元數
        tf::Quaternion rotation;
        rotation.setRotation(axis, angle);

        q *= rotation;
        q.normalize();

        mailbox_40cm_rear_transform[3] = q.x();
        mailbox_40cm_rear_transform[4] = q.y();
        mailbox_40cm_rear_transform[5] = q.z();
        mailbox_40cm_rear_transform[6] = q.w();

        // ------------------------ 將座標平移至信箱40公分處座標 ------------------------
        mailbox_40cm_rear_transform[3] -= 0.4;
        mailbox_40cm_rear_transform[3] -= GRIPPER_OFFSET_D;
        mailbox_40cm_rear_transform[2] += GRIPPER_OFFSET_H;

        // ------------------------ 發布信箱座標 ------------------------
        new_frame::fixedFrame_add(mailbox_40cm_rear_transform, "camera", "mailbox_40cm_rear");
    }
}