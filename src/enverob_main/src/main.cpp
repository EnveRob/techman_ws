#include <ros/ros.h>
#include "main.h"
#include "arm_move.h"
#include "new_frame.h"
#include "force_feedback.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
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
        if (listener.waitForTransform("base", "mailbox_40cm_rear", ros::Time(0), ros::Duration(0.5)))
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

        find_mailbox = 0;
        arm_move::setTargetPosition(move_group, my_plan, targetTransform);
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
        find_mailbox = 0;
        arm_move::setTargetPosition(move_group, my_plan, targetTransform);
    }
    else
    {
        std::cout << "Can't find mailbox!" << std::endl;
        return 0;
    }

    // ------------------------ 手臂向上至正對信箱口 ------------------------
    std::vector<double> mailbox_opening_40cm_rear_position = {0.0, 0.0, -0.2, 0.0, 0.0, 0.0, 1.0};
    new_frame::fixedFrame_add(mailbox_opening_40cm_rear_position, "mailbox_40cm_rear", "mailbox_opening_40cm_rear");
    new_frame::waitforTransform("mailbox_opening_40cm_rear", targetTransform);
    find_mailbox = 1;

    arm_move::setTargetPosition(move_group, my_plan, targetTransform);

    std::vector<double> mailbox_opening_position = {0.0, -0.4, 0.0, 0.0, 0.0, 0.0, 1.0};
    new_frame::fixedFrame_add(mailbox_opening_position, "mailbox_opening_40cm_rear", "mailbox_opening");

    // ------------------------ 手臂往信箱口移動，嘗試讓信接觸到信箱 ------------------------
    std::vector<double> movement = {0.0, -0.45, 0.0, 0.0}; // x, y, z, theta
    arm_move::setRelativePosition(move_group, my_plan, "mailbox_opening_40cm_rear", movement, forceSubsriber);

    // ------------------------ 手臂讓信接觸到信箱後建立新的"mail_touch_mailbox_back"座標系 ------------------------
    std::vector<double> mail_touch_mailbox_back_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    new_frame::fixedFrame_add(mail_touch_mailbox_back_position, "gripper_link", "mail_touch_mailbox_back");
    new_frame::waitforTransform("mail_touch_mailbox_back", targetTransform);

    // ------------------------ 手臂從"mail_touch_mailbox_back"（接觸到信箱後停下的位置）往後移動5cm ------------------------
    movement = {0.0, 0.05, 0.0, 0.0}; // x, y, z, theta
    arm_move::setRelativePosition(move_group, my_plan, "mail_touch_mailbox_back", movement);

    // ------------------------ 手臂以信箱口中心為旋轉軸，向上旋轉10度 ------------------------
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
    arm_move::setRelativePosition(move_group, my_plan, "mailbox_opening", movement);

    movement = {0.0, r * cos(theta), -r * sin(theta), -theta}; // x, y, z, theta
    arm_move::setRelativePosition(move_group, my_plan, "mailbox_opening", movement);

    movement = {0.0, 0.00, 0.0, -theta}; // x, y, z, theta
    arm_move::setRelativePosition(move_group, my_plan, "mailbox_opening", movement, forceSubsriber);

    // ------------------------ 夾爪鬆開投入信件 ------------------------
    std_msgs::UInt8 gripper_msg;
    gripper_msg.data = 2;
    pub.publish(gripper_msg);
    ROS_INFO("Gripper open");

    // ------------------------ 手臂歸位 ------------------------
    new_frame::waitforTransform("mailbox_opening_40cm_rear", targetTransform);
    arm_move::setTargetPosition(move_group, my_plan, targetTransform);
    ROS_INFO("Move to initial position");
    target_joint = {-M_PI * 0.75, -M_PI / 3, M_PI / 3 * 2, -M_PI / 3, M_PI / 2, -M_PI / 2};
    arm_move::setJointangle(move_group, my_plan, target_joint);

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