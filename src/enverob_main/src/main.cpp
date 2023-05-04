#include <ros/ros.h>
#include "main.h"
#include "arm_move.h"
#include "new_frame.h"
#include "force_feedback.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <tf/transform_listener.h>

bool find_mailbox = 1;
int searching_state = 1;

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
    ros::Publisher pub = nh.advertise<std_msgs::UInt8>("gripper_cmd", 1000);
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

    // ------------------------ 手臂往距離信箱30公分處移動 ------------------------
    ROS_INFO("Move to initial position");
    // 將機械手臂移動到初始位置，target_joint[0]的最小值是-M_PI*1.5
    std::vector<double> target_joint{-M_PI, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    // std::vector<double> target_joint{-M_PI * 0.75, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    arm_move::setJointangle(nh, move_group, my_plan, target_joint);

    // 旋轉機械手臂，尋找信箱
    find_mailbox = 0;
    double current_direction = target_joint[0] + ROTATION_STEP;
    // double current_direction = target_joint[0] - ROTATION_STEP;
    while (nh.ok() && current_direction <= 0)
    // while (nh.ok() && current_direction >= -M_PI * 1.25)
    {
        target_joint[0] = current_direction;
        arm_move::setJointangle(nh, move_group, my_plan, target_joint);
        if (listener.waitForTransform("base", "mailbox_30cm_rear", ros::Time(0), ros::Duration(0.5)))
        {
            listener.lookupTransform("base", "mailbox_30cm_rear", ros::Time(0), targetTransform);
            ROS_INFO("targetTransform \n(x, y, z): %.2f, %.2f, %.2f, \n(qx, qy, qz, qw): %.2f, %.2f, %.2f, %.2f",
                     targetTransform.getOrigin().x(), targetTransform.getOrigin().y(), targetTransform.getOrigin().z(),
                     targetTransform.getRotation().x(), targetTransform.getRotation().y(), targetTransform.getRotation().z(), targetTransform.getRotation().w());
            find_mailbox = 1;
            break;
        }
        current_direction += ROTATION_STEP;
        // current_direction -= ROTATION_STEP;
    }

    if (find_mailbox)
    {
        ROS_INFO("Move to 30cm rear of mailbox");
        find_mailbox = 0;
        arm_move::setTargetPosition(nh, move_group, my_plan, targetTransform);
    }
    else
    {
        ROS_ERROR("Can't find mailbox!");
        return 0;
    }

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 手臂再次往距離信箱30公分處移動 ------------------------
    ROS_INFO("Move to 30cm rear of mailbox again");
    new_frame::waitforTransform("mailbox_30cm_rear", targetTransform);
    searching_state = 2;
    find_mailbox = 1;

    if (find_mailbox)
    {
        find_mailbox = 0;
        arm_move::setTargetPosition(nh, move_group, my_plan, targetTransform);
    }
    else
    {
        std::cout << "Can't find mailbox!" << std::endl;
        return 0;
    }

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 手臂向上至正對信箱口 ------------------------
    ROS_INFO("Move to 30cm rear of mailbox opening");
    std::vector<double>
        mailbox_opening_30cm_rear_position = {0.0, 0.0, -0.2, 0.0, 0.0, 0.0, 1.0};
    new_frame::fixedFrame_add(mailbox_opening_30cm_rear_position, "mailbox_30cm_rear", "mailbox_opening_30cm_rear");
    new_frame::waitforTransform("mailbox_opening_30cm_rear", targetTransform);
    find_mailbox = 1;

    arm_move::setTargetPosition(nh, move_group, my_plan, targetTransform);

    std::vector<double> mailbox_opening_position = {0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 1.0};
    new_frame::fixedFrame_add(mailbox_opening_position, "mailbox_opening_30cm_rear", "mailbox_opening");

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ <TOCHECK> ------------------------
    // ------------------------ 手臂往信箱口移動，嘗試讓信接觸到信箱 ------------------------
    ROS_INFO("Move to mailbox opening");
    std::vector<double> movement = {0.0, -0.35, 0.0, 0.0}; // x, y, z, theta
    arm_move::setRelativePosition(nh, move_group, my_plan, "mailbox_opening_30cm_rear", movement, forceSubsriber);

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 手臂讓信接觸到信箱後建立新的"mail_touch_mailbox_back"座標系 ------------------------
    ROS_INFO("Create new frame: mail_touch_mailbox_back");
    std::vector<double> mail_touch_mailbox_back_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    new_frame::fixedFrame_add(mail_touch_mailbox_back_position, "gripper_link", "mail_touch_mailbox_back");
    new_frame::waitforTransform("mail_touch_mailbox_back", targetTransform);

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 手臂從"mail_touch_mailbox_back"（接觸到信箱後停下的位置）往後移動5cm ------------------------
    ROS_INFO("Move back 5cm");
    movement = {0.0, 0.05, 0.0, 0.0}; // x, y, z, theta
    arm_move::setRelativePosition(nh, move_group, my_plan, "mail_touch_mailbox_back", movement);

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 手臂以信箱口中心為旋轉軸，向上旋轉10度 ------------------------
    ROS_INFO("Rotate 10 degrees upward around mailbox opening");
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    tf::Vector3 current_position;
    current_position.setX(current_pose.pose.position.x);
    current_position.setY(current_pose.pose.position.y);
    current_position.setZ(current_pose.pose.position.z);

    tf::StampedTransform mailbox_opening_transform;
    new_frame::waitforTransform("mailbox_opening", mailbox_opening_transform);
    tf::Vector3 reference_position = mailbox_opening_transform.getOrigin();

    double r = tf::tfDistance(current_position, reference_position);
    double theta = M_PI / 18;

    movement = {0.0, r * cos(theta), -r * sin(theta), 0.0}; // x, y, z, theta
    arm_move::setRelativePosition(nh, move_group, my_plan, "mailbox_opening", movement);

    movement = {0.0, r * cos(theta), -r * sin(theta), -theta}; // x, y, z, theta
    arm_move::setRelativePosition(nh, move_group, my_plan, "mailbox_opening", movement);

    ROS_INFO("Move directly to mailbox opening");
    movement = {0.0, 0.00, 0.0, -theta}; // x, y, z, theta
    arm_move::setRelativePosition(nh, move_group, my_plan, "mailbox_opening", movement, forceSubsriber);

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 夾爪鬆開投入信件 ------------------------
    ROS_INFO("Gripper open");
    std_msgs::UInt8 gripper_msg;
    gripper_msg.data = 2;
    pub.publish(gripper_msg);
    ROS_INFO("Gripper open");

    // 輸入任意鍵以繼續
    std::cout << "Press any key to continue..." << std::endl;
    std::cin.ignore();

    // ------------------------ 手臂歸位 ------------------------
    ROS_INFO("Move to initial position");
    new_frame::waitforTransform("mailbox_opening_30cm_rear", targetTransform);
    arm_move::setTargetPosition(nh, move_group, my_plan, targetTransform);
    ROS_INFO("Move to initial position");
    target_joint = {-M_PI * 0.75, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    arm_move::setJointangle(nh, move_group, my_plan, target_joint);

    std::cout << "code end" << std::endl;

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
        std::vector<double> mailbox_30cm_rear_transform{v[0], v[1], v[2]};
        tf::Quaternion q;
        if (searching_state == 1)
        {
            q.setRPY(0, 0, 0); // 以弧度為單位
        }
        else
        {
            q.setRPY(v[3], v[4], v[5]); // 以弧度為單位
        }

        mailbox_30cm_rear_transform.push_back(q.x());
        mailbox_30cm_rear_transform.push_back(q.y());
        mailbox_30cm_rear_transform.push_back(q.z());
        mailbox_30cm_rear_transform.push_back(q.w());

        // ------------------------ 將座標改為z軸向上 ------------------------
        q.setX(mailbox_30cm_rear_transform[3]);
        q.setY(mailbox_30cm_rear_transform[4]);
        q.setZ(mailbox_30cm_rear_transform[5]);
        q.setW(mailbox_30cm_rear_transform[6]);

        // 定義旋轉軸和旋轉角度
        tf::Vector3 axis(1, 0, 0);
        double angle = -M_PI / 2;

        // 將旋轉軸和旋轉角度轉換為四元數
        tf::Quaternion rotation;
        rotation.setRotation(axis, angle);

        q *= rotation;
        q.normalize();

        mailbox_30cm_rear_transform[3] = q.x();
        mailbox_30cm_rear_transform[4] = q.y();
        mailbox_30cm_rear_transform[5] = q.z();
        mailbox_30cm_rear_transform[6] = q.w();

        // ------------------------ 將座標平移至信箱30公分處座標 ------------------------
        mailbox_30cm_rear_transform[2] -= 0.3;
        mailbox_30cm_rear_transform[2] -= GRIPPER_OFFSET_D;
        mailbox_30cm_rear_transform[1] += GRIPPER_OFFSET_H;

        // ------------------------ 發布信箱座標 ------------------------
        new_frame::fixedFrame_add(mailbox_30cm_rear_transform, "camera", "mailbox_30cm_rear");
    }
}